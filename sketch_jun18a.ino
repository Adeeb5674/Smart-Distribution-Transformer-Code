/*
  Smart Transformer – PF-enabled build (ESP32 + ADS1115 + ZMPT + ACS712)
  - Per-phase Vrms, Irms, Real Power (W), Apparent Power (VA), PF
  - Total P, S, PF
  - Auto phase alignment via cross-correlation (per phase)
  - ThingsBoard telemetry integrated
  - Relays kept OFF until TB is connected (safety)

  Protection Logic (Option A):
  - Load activation: any phase Irms ≥ 1.25 A -> Relays 1–3 ON
  - Overload: any phase Irms ≥ 2.50 A -> Relays 4–6 OFF for 60 s
  - Undervoltage: any phase Vrms < 20.0 V -> Relays 4–6 OFF for 60 s
*/

#include <Adafruit_ADS1X15.h>
#include <ZMPT101B.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include <ArduinoJson.h>
#include <math.h>
#include <Wire.h>

// ------------------------ WiFi & ThingsBoard ------------------------
constexpr char WIFI_SSID[] = "omega";
constexpr char WIFI_PASSWORD[] = "09000900";
constexpr char TOKEN[] = "kaz30jqlzu12pnx82hab";
constexpr char THINGSBOARD_SERVER[] = "192.168.4.23";
constexpr uint16_t THINGSBOARD_PORT = 1883U;

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient);

// ------------------------ Voltage Sensors (ZMPT on ADC) ------------
const uint8_t SENSOR_PIN0 = 32; // L1
const uint8_t SENSOR_PIN1 = 33; // L2
const uint8_t SENSOR_PIN2 = 34; // L3

#define VOLT_SENSITIVITY 440.75f
#define NOISE_THRESHOLD 5.0f
ZMPT101B voltageSensor0(SENSOR_PIN0);
ZMPT101B voltageSensor1(SENSOR_PIN1);
ZMPT101B voltageSensor2(SENSOR_PIN2);

// ESP32 ADC raw -> volts, then scaled to actual secondary Vrms
constexpr float ADC_TO_VOLTS = 3.3f / 4095.0f;
// Adjust after first calibration run so Vrms ≈ 24–26 V at no/nominal load
float VOLT_SCALE1 = 480.0f; // L1
float VOLT_SCALE2 = 480.0f; // L2
float VOLT_SCALE3 = 480.0f; // L3

// ------------------------ Current Sensors (ADS1115 + ACS712) -------
Adafruit_ADS1115 ads;
const int NUM_CHANNELS = 3;
const int CURR_CH[NUM_CHANNELS] = {0, 1, 2}; // ADS A0,A1,A2

// ACS712 sensitivity (Volts per Amp). Your tuned value ~0.07187 V/A
const float CURRENT_SENSITIVITY[NUM_CHANNELS] = {0.07187f, 0.07187f, 0.07187f};

// Offsets captured at startup (raw units)
float   voltOffsetADC[NUM_CHANNELS] = {0, 0, 0}; // ESP32 ADC baseline
int16_t currOffsetRAW[NUM_CHANNELS] = {0, 0, 0}; // ADS1115 baseline

// ------------------------ Temperature & Pot ------------------------
const int ONE_WIRE_BUS = 4;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);

const int TEMP_SMOOTHING_READINGS = 10;
float tempReadings[TEMP_SMOOTHING_READINGS] = {};
int   tempIndex = 0;
float smoothedTemp = NAN;

const int potPin = 36; // ADC
const int POT_SMOOTHING = 10;
int   potValues[POT_SMOOTHING] = {};
int   potIndex = 0;
float smoothedPotPercent = 0.0f;

// ------------------------ Relays -----------------------------------
const int relayPins[8] = {13, 14, 16, 17, 25, 26, 27, 19};
unsigned long relayOffTime[8] = {0}; // use [3] as the 60 s timer for relays 4–6

// ------------------------ States/Status Strings --------------------
bool   lastRelay456State = true;
String loadStatus  = "Connected";
String faultStatus = "No";      // "No", "Undervoltage", "Overload"
String T1_status   = "OFF";
String T2_status   = "OFF";
String charging_status = "OFF";
String pump_state      = "OFF";

// ------------------------ PF Sampling / Math -----------------------
const int NUM_SAMPLES = 200;      // waveform samples per cycle
const int SAMPLE_DELAY_US = 250;  // ~4 kHz sampling cadence
const int MAX_LAG = 12;           // cross-correlation search window (±lag samples)

// ------------------------ Noise Thresholds -------------------------
const float VOLTAGE_NOISE_THRESHOLD = 10.0f;  // Vrms below this treated as 0
const float CURRENT_NOISE_THRESHOLD = 0.065f; // Irms below this treated as 0 A

// ------------------------ NEW Protection Thresholds ----------------
const float LOAD_THRESHOLD      = 1.25f;  // any phase ≥ -> Relays 1–3 ON
const float OVERLOAD_THRESHOLD  = 2.50f;  // any phase ≥ -> Relays 4–6 OFF 60 s
const float UNDERVOLT_THRESHOLD = 20.0f;  // any phase < -> Relays 4–6 OFF 60 s

// ------------------------ Timing -----------------------------------
uint32_t previousDataSend = 0;
constexpr uint32_t TELEMETRY_SEND_INTERVAL = 2000U;

uint32_t lastSensorRead = 0;
constexpr uint32_t SENSOR_READ_INTERVAL = 150U;

// ------------------------ Telemetry struct -------------------------
struct SensorData {
  float voltages[3];   // Vrms per phase
  float currents[3];   // Irms per phase (A)
  float currents_mA[3];
  float VA[3];         // Apparent per phase
  float power[3];      // Real per phase
  float pf[3];         // PF per phase
  float totalVA;
  float totalPower;
  float totalPF;
  float temperature;
  float potentiometer;
};
SensorData currentData;

// ------------------------ Helpers ----------------------------------
void InitWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.print("Connecting to WiFi "); Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 12000) {
    delay(200); Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi");
    Serial.print("IP Address: "); Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect to WiFi");
  }
}

float readVoltageRMS_ZMPT(ZMPT101B& sensor) {
  float v = sensor.getRmsVoltage();
  return (v < NOISE_THRESHOLD) ? 0.0f : v;
}

// Update connection and a generic fault indicator,
// but do NOT overwrite a specific fault reason if one is active.
void monitorLoadStatus() {
  bool r456 = (digitalRead(relayPins[3]) == HIGH &&
               digitalRead(relayPins[4]) == HIGH &&
               digitalRead(relayPins[5]) == HIGH);
  loadStatus  = r456 ? "Connected" : "Disconnected";

  // If no specific fault reason was set, show simple Yes/No
  if (faultStatus == "No") {
    faultStatus = r456 ? "No" : "Yes";
  }
  lastRelay456State = r456;
}

// Cross-correlation to find best lag between v[] and i[]
int bestLag(const float* v, const float* i, int N, int maxLag) {
  double best = -1e18;
  int lagBest = 0;
  for (int lag = -maxLag; lag <= maxLag; lag++) {
    double corr = 0.0;
    for (int n = 0; n < N; n++) {
      int j = n + lag;
      if (j >= 0 && j < N) corr += (double)v[n] * (double)i[j];
    }
    if (corr > best) { best = corr; lagBest = lag; }
  }
  return lagBest;
}

// ------------------------ Setup Sensors / Relays --------------------
void setupSensors() {
  Serial.println("Initializing Sensors...");

  // ZMPT sensitivity (library path not used for PF, but keep it set)
  voltageSensor0.setSensitivity(VOLT_SENSITIVITY);
  voltageSensor1.setSensitivity(VOLT_SENSITIVITY);
  voltageSensor2.setSensitivity(VOLT_SENSITIVITY);

  // I2C hi-speed + ADS1115
  Wire.begin();
  Wire.setClock(400000);
  ads.setGain(GAIN_ONE); // ±4.096V
  if (!ads.begin()) {
    Serial.println("ADS1115 failed!");
    while (1) { delay(1000); }
  }

  // Temperature
  tempSensor.begin();
  for (int i = 0; i < TEMP_SMOOTHING_READINGS; i++) tempReadings[i] = NAN;

  // Pot smoothing
  for (int i = 0; i < POT_SMOOTHING; i++) potValues[i] = 0;

  // Offset calibration (no load)
  Serial.println("Calibrating offsets... keep secondary unloaded.");
  const int CALS = 200;
  long vSumADC[3] = {0,0,0};
  long iSumRAW[3] = {0,0,0};
  for (int n = 0; n < CALS; n++) {
    vSumADC[0] += analogRead(SENSOR_PIN0);
    vSumADC[1] += analogRead(SENSOR_PIN1);
    vSumADC[2] += analogRead(SENSOR_PIN2);
    iSumRAW[0] += ads.readADC_SingleEnded(CURR_CH[0]);
    iSumRAW[1] += ads.readADC_SingleEnded(CURR_CH[1]);
    iSumRAW[2] += ads.readADC_SingleEnded(CURR_CH[2]);
    delay(5);
  }
  for (int k = 0; k < 3; k++) {
    voltOffsetADC[k] = (float)vSumADC[k] / CALS;
    currOffsetRAW[k] = (int16_t)(iSumRAW[k] / CALS);
  }
  Serial.printf("Offsets: Vadc[%.1f, %.1f, %.1f], Iraw[%d, %d, %d]\n",
    voltOffsetADC[0], voltOffsetADC[1], voltOffsetADC[2],
    currOffsetRAW[0], currOffsetRAW[1], currOffsetRAW[2]);

  Serial.println("All sensors initialized.");
}

void setupRelays() {
  for (int i = 0; i < 8; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], LOW); // OFF at boot
  }
  Serial.println("Relays initialized (all OFF).");
}

// ------------------------ T1/T2 display flags ----------------------
void calculateT1T2Status() {
  float max_mA = max(currentData.currents_mA[0],
                 max(currentData.currents_mA[1], currentData.currents_mA[2]));
  T1_status = (max_mA >= 90.0f  && max_mA <= 2000.0f) ? "ON" : "OFF";
  T2_status = (max_mA >= 1000.0f && max_mA <= 2000.0f) ? "ON" : "OFF";
}

// ------------------------ Relay Control (UPDATED) -------------------
void controlRelays(bool systemEnabled) {
  // If not enabled (TB not connected), keep everything OFF for safety
  if (!systemEnabled) {
    for (int i = 0; i < 8; i++) digitalWrite(relayPins[i], LOW);
    charging_status = "OFF";
    pump_state = "OFF";
    faultStatus = "No";
    calculateT1T2Status();
    monitorLoadStatus();
    return;
  }

  unsigned long now = millis();

  // ---- New protection conditions ----
  bool underVoltage = (
    currentData.voltages[0] < UNDERVOLT_THRESHOLD ||
    currentData.voltages[1] < UNDERVOLT_THRESHOLD ||
    currentData.voltages[2] < UNDERVOLT_THRESHOLD
  );

  bool veryHigh = (
    currentData.currents[0] >= OVERLOAD_THRESHOLD ||
    currentData.currents[1] >= OVERLOAD_THRESHOLD ||
    currentData.currents[2] >= OVERLOAD_THRESHOLD
  );

  // If undervoltage OR overload -> Relays 4–6 OFF for 60 s
  if (underVoltage || veryHigh) {
    relayOffTime[3] = now;                          // use index 3 timer for group 4–6
    for (int i = 3; i < 6; i++) digitalWrite(relayPins[i], LOW);
    faultStatus = underVoltage ? "Undervoltage" : "Overload";
  } else if (now - relayOffTime[3] >= 60000UL) {
    // If protection window elapsed and no current fault, restore relays 4–6
    for (int i = 3; i < 6; i++) digitalWrite(relayPins[i], HIGH);
    // Only clear specific fault if relays are restored and no new fault
    faultStatus = "No";
  }

  // Relays 1–3: ON if any phase >= 1.25 A (load detection)
  bool highCurrent = (
    currentData.currents[0] >= LOAD_THRESHOLD ||
    currentData.currents[1] >= LOAD_THRESHOLD ||
    currentData.currents[2] >= LOAD_THRESHOLD
  );
  for (int i = 0; i < 3; i++) digitalWrite(relayPins[i], highCurrent ? LOW : HIGH);

  // Relay 7: charging window 20–28 V (avg of three phases)
  float vAvg = (currentData.voltages[0] + currentData.voltages[1] + currentData.voltages[2]) / 3.0f;
  bool chargeOn = (vAvg >= 20.0f && vAvg <= 28.0f);
  digitalWrite(relayPins[6], chargeOn ? HIGH : LOW);
  charging_status = chargeOn ? "ON" : "OFF";

  // Relay 8: pump >= 50 C
  bool pumpOn = (isfinite(currentData.temperature) && currentData.temperature >= 50.0f);
  digitalWrite(relayPins[7], pumpOn ? LOW : HIGH);
  pump_state = pumpOn ? "ON" : "OFF";

  calculateT1T2Status();
  monitorLoadStatus(); // preserves specific reason in faultStatus if present
}

// ------------------------ Waveform Read + PF Math ------------------
void readWaveformPF() {
  // buffers per phase
  float vBuf[3][NUM_SAMPLES];
  float iBuf[3][NUM_SAMPLES];

  // Collect waveform samples
  for (int n = 0; n < NUM_SAMPLES; n++) {
    // Voltage samples (ESP32 ADC)
    int rawV0 = analogRead(SENSOR_PIN0);
    int rawV1 = analogRead(SENSOR_PIN1);
    int rawV2 = analogRead(SENSOR_PIN2);

    // Convert to scaled volts per phase (remove offset then scale)
    vBuf[0][n] = ( (rawV0 - voltOffsetADC[0]) * ADC_TO_VOLTS ) * VOLT_SCALE1;
    vBuf[1][n] = ( (rawV1 - voltOffsetADC[1]) * ADC_TO_VOLTS ) * VOLT_SCALE2;
    vBuf[2][n] = ( (rawV2 - voltOffsetADC[2]) * ADC_TO_VOLTS ) * VOLT_SCALE3;

    // Current samples (ADS1115)
    int16_t rI0 = ads.readADC_SingleEnded(CURR_CH[0]);
    int16_t rI1 = ads.readADC_SingleEnded(CURR_CH[1]);
    int16_t rI2 = ads.readADC_SingleEnded(CURR_CH[2]);

    // Convert to volts, remove offset, then to Amps via sensitivity
    float vI0 = ads.computeVolts( (int32_t)rI0 - (int32_t)currOffsetRAW[0] );
    float vI1 = ads.computeVolts( (int32_t)rI1 - (int32_t)currOffsetRAW[1] );
    float vI2 = ads.computeVolts( (int32_t)rI2 - (int32_t)currOffsetRAW[2] );

    iBuf[0][n] = vI0 / CURRENT_SENSITIVITY[0];
    iBuf[1][n] = vI1 / CURRENT_SENSITIVITY[1];
    iBuf[2][n] = vI2 / CURRENT_SENSITIVITY[2];

    delayMicroseconds(SAMPLE_DELAY_US);
  }

  // Per-phase compute with auto lag
  float Vrms[3] = {0}, Irms[3] = {0}, P[3] = {0}, S[3] = {0}, PF[3] = {0};

  for (int ph = 0; ph < 3; ph++) {
    // Find best lag to align I to V
    int lag = bestLag(vBuf[ph], iBuf[ph], NUM_SAMPLES, MAX_LAG);

    double sumV2 = 0.0, sumI2 = 0.0, sumVI = 0.0;
    int count = 0;

    for (int n = 0; n < NUM_SAMPLES; n++) {
      int j = n + lag;
      if (j < 0 || j >= NUM_SAMPLES) continue;
      float v = vBuf[ph][n];
      float i = iBuf[ph][j];
      sumV2 += (double)v * (double)v;
      sumI2 += (double)i * (double)i;
      sumVI += (double)v * (double)i;
      count++;
    }

    if (count < 10) count = NUM_SAMPLES; // fallback avoid div0
    Vrms[ph] = sqrt(sumV2 / count);
    Irms[ph] = sqrt(sumI2 / count);

    // Apply noise thresholds
    if (!isfinite(Vrms[ph]) || Vrms[ph] < VOLTAGE_NOISE_THRESHOLD) {
      Vrms[ph] = 0.0f;
    }
    if (!isfinite(Irms[ph]) || Irms[ph] < CURRENT_NOISE_THRESHOLD) {
      Irms[ph] = 0.0f; 
      P[ph] = 0.0f; 
      S[ph] = 0.0f; 
      PF[ph] = 0.0f;
    } else {
      P[ph]  = sumVI / count;
      S[ph]  = Vrms[ph] * Irms[ph];
      PF[ph] = (S[ph] > 0.01f) ? (P[ph] / S[ph]) : 0.0f;
    }
  }

  // Pack into currentData
  for (int i = 0; i < 3; i++) {
    currentData.voltages[i]    = Vrms[i];
    currentData.currents[i]    = Irms[i];
    currentData.currents_mA[i] = Irms[i] * 1000.0f;
    currentData.VA[i]          = S[i];
    currentData.power[i]       = P[i];
    currentData.pf[i]          = PF[i];
  }
  currentData.totalPower = currentData.power[0] + currentData.power[1] + currentData.power[2];
  currentData.totalVA    = currentData.VA[0]    + currentData.VA[1]    + currentData.VA[2];
  currentData.totalPF    = (currentData.totalVA > 0.01f) ? (currentData.totalPower / currentData.totalVA) : 0.0f;

  // Temperature smoothing
  tempSensor.requestTemperatures();
  float rawT = tempSensor.getTempCByIndex(0);
  if (rawT > -50.0f && rawT < 150.0f && isfinite(rawT)) {
    if (!isfinite(smoothedTemp)) {
      smoothedTemp = rawT;
      for (int i = 0; i < TEMP_SMOOTHING_READINGS; i++) tempReadings[i] = rawT;
    } else {
      smoothedTemp -= tempReadings[tempIndex] / TEMP_SMOOTHING_READINGS;
      tempReadings[tempIndex] = rawT;
      smoothedTemp += tempReadings[tempIndex] / TEMP_SMOOTHING_READINGS;
      tempIndex = (tempIndex + 1) % TEMP_SMOOTHING_READINGS;
    }
    currentData.temperature = smoothedTemp;
  }

  // Pot smoothing (%)
  potValues[potIndex] = analogRead(potPin);
  potIndex = (potIndex + 1) % POT_SMOOTHING;
  int ps = 0; for (int i = 0; i < POT_SMOOTHING; i++) ps += potValues[i];
  float avg = (float)ps / POT_SMOOTHING;
  currentData.potentiometer = constrain((avg / 4095.0f) * 100.0f, 0.0f, 100.0f);
}

// ------------------------ Serial Dump ------------------------------
void printSerialData() {
  Serial.printf("V:%.1f,%.1f,%.1f\t", currentData.voltages[0], currentData.voltages[1], currentData.voltages[2]);
  Serial.printf("I:%.3f,%.3f,%.3f A\t", currentData.currents[0], currentData.currents[1], currentData.currents[2]);
  Serial.printf("W:%.2f,%.2f,%.2f|%.2f\t",
                currentData.power[0], currentData.power[1], currentData.power[2], currentData.totalPower);
  Serial.printf("VA:%.2f,%.2f,%.2f|%.2f\t",
                currentData.VA[0], currentData.VA[1], currentData.VA[2], currentData.totalVA);
  Serial.printf("PF:%.2f,%.2f,%.2f|%.2f\t",
                currentData.pf[0], currentData.pf[1], currentData.pf[2], currentData.totalPF);
  Serial.printf("T:%.1fC\tPot:%.1f%%\t", currentData.temperature, currentData.potentiometer);
  Serial.printf("Load:%s\tFault:%s\tCharging:%s\tPump:%s\tT1:%s\tT2:%s",
                loadStatus.c_str(), faultStatus.c_str(), charging_status.c_str(),
                pump_state.c_str(), T1_status.c_str(), T2_status.c_str());
  Serial.println();
}

// ------------------------ Setup / Loop -----------------------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("== Smart Transformer PF-enabled Init ==");
  InitWiFi();
  setupSensors();
  setupRelays();
  Serial.println("Init complete.");
  Serial.println("V(V)\tI(A)\tW|Wtot\tVA|VAtot\tPF|PFtot\tT\tPot\tLoad/Fault\tChg/Pump\tT1/T2");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) InitWiFi();

  bool tbConnected = tb.connected();
  if (!tbConnected) {
    Serial.println("Connecting to ThingsBoard...");
    tbConnected = tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT);
    Serial.println(tbConnected ? "Connected to ThingsBoard" : "Failed to connect to ThingsBoard");
  }

  // Read waveforms + PF roughly every SENSOR_READ_INTERVAL
  if (millis() - lastSensorRead >= SENSOR_READ_INTERVAL) {
    lastSensorRead = millis();
    readWaveformPF();                            // true PF
    controlRelays(tbConnected);                  // relays only active if TB connected
    printSerialData();
  }

  // Telemetry every TELEMETRY_SEND_INTERVAL
  if (tbConnected && (millis() - previousDataSend >= TELEMETRY_SEND_INTERVAL)) {
    previousDataSend = millis();

    // Per-phase volt, curr, W, VA, PF
    tb.sendTelemetryData("voltage1", currentData.voltages[0]);
    tb.sendTelemetryData("voltage2", currentData.voltages[1]);
    tb.sendTelemetryData("voltage3", currentData.voltages[2]);

    tb.sendTelemetryData("current1_A", currentData.currents[0]);
    tb.sendTelemetryData("current2_A", currentData.currents[1]);
    tb.sendTelemetryData("current3_A", currentData.currents[2]);

    tb.sendTelemetryData("power1_W", currentData.power[0]);
    tb.sendTelemetryData("power2_W", currentData.power[1]);
    tb.sendTelemetryData("power3_W", currentData.power[2]);
    tb.sendTelemetryData("power_total_W", currentData.totalPower);

    tb.sendTelemetryData("VA1", currentData.VA[0]);
    tb.sendTelemetryData("VA2", currentData.VA[1]);
    tb.sendTelemetryData("VA3", currentData.VA[2]);
    tb.sendTelemetryData("VA_total", currentData.totalVA);

    tb.sendTelemetryData("pf1", currentData.pf[0]);
    tb.sendTelemetryData("pf2", currentData.pf[1]);
    tb.sendTelemetryData("pf3", currentData.pf[2]);
    tb.sendTelemetryData("pf_total", currentData.totalPF);

    // Misc
    tb.sendTelemetryData("temperature_C", currentData.temperature);
    tb.sendTelemetryData("potentiometer_pct", currentData.potentiometer);

    // Statuses
    tb.sendTelemetryData("load_status", loadStatus.c_str());
    tb.sendTelemetryData("fault_status", faultStatus.c_str());   // now shows reason
    tb.sendTelemetryData("charging_status", charging_status.c_str());
    tb.sendTelemetryData("pump_state", pump_state.c_str());
    tb.sendTelemetryData("T1_status", T1_status.c_str());
    tb.sendTelemetryData("T2_status", T2_status.c_str());

    Serial.println("Telemetry sent.");
  }

  tb.loop();
  delay(5);
}
