/*
  ESP32 Firmware for Smart Distribution Transformer Prototype

  This code implements the control logic for the Smart Distribution Transformer
  prototype, including:
  1. Real-time monitoring of 3-phase voltage, current, and transformer oil temperature.
  2. Dynamic load management (switching Transformer 2 based on load current).
  3. 3-phase overcurrent protection (individual and global load disconnection).
  4. Smart cooling system activation based on oil temperature.
  5. Wireless data transmission to the Blynk IoT platform for remote monitoring.

  Hardware used:
  - ESP32 Microcontroller (e.g., NodeMCU ESP32, ESP32 DevKitC)
  - Freenove ESP32 Breakout Board (for easy pin access and power)
  - ZMPT101B Voltage Sensor Modules (3 units for 3 phases)
  - ACS712 Current Sensor Modules (3 units for 3 phases)
  - DS18B20 Temperature Sensor (1 unit for mineral oil temperature)
  - DCR-M70 DC 5V 8-Channel Solid State Relay Module
    - Used for PHASE_RELAY_PINS (dynamic load management/individual phase control)
    - Used for GLOBAL_RELAY_PINS (global load disconnection)
    - Used for TEMP_RELAY_PIN (cooling system activation)
  - Standard gaming PC radiator with fan
  - domoi Ultra-Quiet SC-300T Water Pump Set (used here for mineral oil circulation)

  NOTE: This prototype design immerses only the transformers in mineral oil.
  All sensitive electronic components (ESP32, sensors, relays) are housed in a
  dedicated DRY COMPARTMENT located above the oil level within the main casing.
  This ensures component compatibility, reduces EMI, and simplifies maintenance.
  Careful consideration of the domoi pump's compatibility with mineral oil
  and potential long-term effects on the radiator is crucial during physical
  implementation and testing.
*/

// ===================== Blynk & WiFi Configuration =====================
// These credentials link your ESP32 to your specific Blynk project.
// Replace with your actual Blynk Template ID, Template Name, and Auth Token.
#define BLYNK_TEMPLATE_ID "TMPL6S6gTBKzZ"
#define BLYNK_TEMPLATE_NAME "Smart Power Transformer" // Corrected: Name from template
#define BLYNK_AUTH_TOKEN "L8AVcMna8R-3uZCP5eTgfahX-A6xjxx1"

// Include necessary libraries for WiFi, Blynk, OneWire, DallasTemperature,
// ACS712, and ZMPT101B sensors.
#include <WiFi.h>              // Standard WiFi library for ESP32
#include <BlynkSimpleEsp32.h>  // Blynk library for ESP32 connectivity
#include <OneWire.h>           // Library for 1-Wire communication protocol (for DS18B20)
#include <DallasTemperature.h> // Library for DS18B20 temperature sensor
#include <ACS712.h>            // Custom library for ACS712 current sensor (might need to be installed)
#include <ZMPT101B.h>          // Custom library for ZMPT101B voltage sensor (might need to be installed)

// WiFi Credentials: Your Wi-Fi network's SSID and password.
// Replace with your actual Wi-Fi network details.
char auth[] = BLYNK_AUTH_TOKEN;    // Use the defined Auth Token for consistency
char ssid[] = "Galaxy S10ea74f";   // Your Wi-Fi Network Name
char pass[] = "addv5860";          // Your Wi-Fi Network Password

// Blynk Virtual Pins: Mappings for sending sensor data to the Blynk dashboard.
// These correspond to the Virtual Pins configured in your Blynk app.
#define BLYNK_V_TEMP   V0 // Virtual Pin for Temperature data
#define BLYNK_V_VOLT1  V1 // Virtual Pin for Phase 1 Voltage
#define BLYNK_V_VOLT2  V2 // Virtual Pin for Phase 2 Voltage
#define BLYNK_V_VOLT3  V3 // Virtual Pin for Phase 3 Voltage
#define BLYNK_V_CURR1  V4 // Virtual Pin for Phase 1 Current
#define BLYNK_V_CURR2  V5 // Virtual Pin for Phase 2 Current
#define BLYNK_V_CURR3  V6 // Virtual Pin for Phase 3 Current

// ===================== Hardware Configuration & Pin Assignments =====================

// DS18B20 Temperature Sensor:
// Pin connected to the 1-Wire data line of the DS18B20 sensor.
const int ONE_WIRE_BUS = 4;
OneWire oneWire(ONE_WIRE_BUS);          // Setup a 1-Wire instance to communicate with any 1-Wire devices
DallasTemperature sensors(&oneWire);    // Pass our 1-Wire reference to Dallas Temperature sensor library

// Temperature Smoothing:
// Number of readings to average for temperature smoothing.
// Reduced from 5 to 3 for faster response to temperature changes.
const int SMOOTHING_READINGS = 3;
float tempReadings[SMOOTHING_READINGS]; // Array to store recent temperature readings for smoothing
int currentIndex = 0;                   // Index for the current reading in the smoothing array (circular buffer)
float smoothedTemp = 0;                 // Variable to store the calculated smoothed temperature

// Protection Limits:
// Pin connected to the relay controlling the cooling system.
#define TEMP_RELAY_PIN        21
// Temperature threshold in Celsius. If smoothedTemp exceeds this, cooling system activates.
#define TEMP_LIMIT            50.0
// Current noise threshold in mA. Readings below this are considered noise (0 mA).
#define CURR_NOISE_THRESHOLD  60 // mA (increased from 50mA to filter more noise)
// Voltage noise threshold in Volts. Readings below this are considered noise (0 V).
#define VOLT_NOISE_THRESHOLD  6.0f // V (increased from 5V to filter more noise)

// ZMPT101B Voltage Sensors:
// Sensitivity factor for ZMPT101B. This value is crucial for accurate voltage conversion.
// It depends on the voltage divider resistors on the ZMPT101B module and the ESP32's ADC.
#define SENSITIVITY 440.75f
// Analog pins connected to the ZMPT101B modules for 3 phases.
const uint8_t PHASE_VOLT_PINS[3] = {32, 33, 34};
// Array of ZMPT101B objects, one for each phase.
ZMPT101B voltSensors[3] = {
  ZMPT101B(PHASE_VOLT_PINS[0]),
  ZMPT101B(PHASE_VOLT_PINS[1]),
  ZMPT101B(PHASE_VOLT_PINS[2])
};

// ACS712 Current Sensors:
// Analog pins connected to the ACS712 modules for 3 phases.
const uint8_t PHASE_CURR_PINS[3] = {35, 36, 39};
// Array of ACS712 objects, one for each phase.
// Constructor parameters: (pin, VCC_Voltage, ADC_Resolution, mV_per_Amp)
// - 3.3: ESP32's analog reference voltage
// - 4095: ESP32's 12-bit ADC resolution (0 to 4095)
// - 185: mV per Ampere for the 5A version of ACS712 (check your sensor's datasheet!)
ACS712 currSensors[3] = {
  ACS712(PHASE_CURR_PINS[0], 3.3, 4095, 185),
  ACS712(PHASE_CURR_PINS[1], 3.3, 4095, 185),
  ACS712(PHASE_CURR_PINS[2], 3.3, 4095, 185)
};

// Relay Pin Assignments:
// Pins connected to the relays for individual phase control (e.g., Transformer 2 switching).
const uint8_t PHASE_RELAY_PINS[3] = {13, 14, 16};
// Pins connected to the relays for global load disconnection (main protection).
const uint8_t GLOBAL_RELAY_PINS[3] = {17, 18, 19};

// Protection Thresholds (Current):
// Individual phase current limit in mA. If current on a phase exceeds this, it might be switched.
// In the current logic, this is used for individual phase relays, which might be Transformer 2 control.
const int INDIVIDUAL_LIMIT = 1000; // mA (1 Ampere)
// Global current limit in mA. If current on any phase exceeds this, all global relays open.
const int GLOBAL_LIMIT = 2000;      // mA (2 Amperes)
// Cooldown time in milliseconds after a global protection trip before it can be reset.
const unsigned long COOLDOWN_TIME = 60000; // 60 seconds (1 minute)

// Sensor Calibration Variables:
// Zero-point offsets for current sensors. Determined during calibration.
// AnalogRead(0A) should ideally be around 2048 for a 12-bit ADC with Vcc/2 bias.
int zeroPoints[3] = {2048, 2048, 2048};
// Scaling factors for current sensors (can be adjusted after initial testing/calibration).
float scaling[3] = {1.0, 1.0, 1.0};

// Global Protection State Variables:
// Flag to indicate if global protection is currently active.
bool globalProtectionActive = false;
// Timestamp when global protection was activated. Used for cooldown timer.
unsigned long globalProtectionStart = 0;

// ===================== Function Definitions =====================

/**
 * @brief Reads current and voltage data from all three phases.
 * This function optimizes sensor reading by performing all reads in one go,
 * reducing potential delays from multiple sensor accesses in the main loop.
 *
 * @param currents Array to store RMS current values (in mA) for each phase.
 * @param voltages Array to store RMS voltage values (in V) for each phase.
 */
void readSensors(float (&currents)[3], float (&voltages)[3]) {
  for (int i = 0; i < 3; i++) {
    // Current Sensor (ACS712) Reading and Conversion:
    // 1. Read raw analog-to-digital converter (ADC) value from the current sensor pin.
    // 2. Subtract the calibrated `zeroPoint` offset. This compensates for the sensor's
    //    output voltage when no current flows (bias).
    int rawCurrent = analogRead(PHASE_CURR_PINS[i]) - zeroPoints[i];
    
    // 3. Convert the raw ADC offset value to current in milliamps (mA).
    //    The formula used here:
    //    abs(rawCurrent) - takes the absolute value as current can flow in both directions.
    //    (3.3 / 4095.0) - converts ADC reading (0-4095) to voltage (0-3.3V).
    //    (1000 / 0.185) - converts voltage to mA, based on 185mV/A sensitivity.
    //    scaling[i] - applies an optional calibration factor for fine-tuning.
    currents[i] = abs(rawCurrent) * (3.3 / 4095.0) * (1000 / 0.185) * scaling[i];
    
    // Apply noise threshold: if the calculated current is very low, assume it's electrical noise
    // and set the current value to 0 mA to prevent spurious readings.
    if (currents[i] < CURR_NOISE_THRESHOLD) currents[i] = 0;

    // Voltage Sensor (ZMPT101B) Reading and Conversion:
    // `getRmsVoltage()` is a method provided by the ZMPT101B library that
    // calculates and returns the Root Mean Square (RMS) voltage.
    voltages[i] = voltSensors[i].getRmsVoltage();
    // Apply noise threshold: if the calculated voltage is very low, assume it's noise
    // and set the voltage value to 0 V.
    if (voltages[i] < VOLT_NOISE_THRESHOLD) voltages[i] = 0;
  }
}

/**
 * @brief Handles the global overcurrent protection logic for the transformer.
 * This function continuously monitors all three phases. If the current in any
 * phase exceeds the `GLOBAL_LIMIT`, it activates the global relays to disconnect
 * the main load, protecting the transformer and downstream equipment.
 * A cooldown period is implemented before the system can be reset, preventing
 * immediate re-engagement after a fault.
 *
 * @param currents Array of RMS current values (in mA) for each phase.
 */
void handleGlobalProtection(float currents[3]) {
  // Iterate through each phase to check for overcurrent.
  for (int i = 0; i < 3; i++) {
    // If current in any phase exceeds the `GLOBAL_LIMIT` AND global protection is not already active:
    if (currents[i] > GLOBAL_LIMIT && !globalProtectionActive) {
      globalProtectionActive = true;       // Set the flag to indicate global protection is now active.
      globalProtectionStart = millis();    // Record the current time for the cooldown timer.
      // Activate all global relays (set to LOW) to disconnect the main load.
      // (Assuming LOW signal activates the relay and opens the circuit).
      for (int r = 0; r < 3; r++) digitalWrite(GLOBAL_RELAY_PINS[r], LOW);
      Serial.println("!!! GLOBAL OVERCURRENT - MAIN LOAD DISCONNECTED !!!"); // Print an alert message to the Serial Monitor.
      break; // Exit the loop immediately once an overcurrent is detected to prevent redundant actions.
    }
  }

  // If global protection is currently active AND the `COOLDOWN_TIME` has elapsed since activation:
  if (globalProtectionActive && (millis() - globalProtectionStart >= COOLDOWN_TIME)) {
    globalProtectionActive = false; // Reset the flag, allowing global protection to be disengaged.
    // Deactivate all global relays (set to HIGH) to reconnect the main load.
    // (Assuming HIGH signal deactivates the relay and closes the circuit).
    for (int r = 0; r < 3; r++) digitalWrite(GLOBAL_RELAY_PINS[r], HIGH);
    Serial.println("Global protection reset - Main load reconnected."); // Inform the Serial Monitor that protection has reset.
  }
}

/**
 * @brief Handles individual phase protection logic, also used for dynamic load management (Transformer 2 switching).
 * This function checks if any phase current exceeds `INDIVIDUAL_LIMIT`. If it does, it activates
 * the `PHASE_RELAY_PINS` (e.g., to engage Transformer 2 or manage individual phase loads).
 * If all phase currents fall below `INDIVIDUAL_LIMIT`, these relays are deactivated.
 *
 * @param currents Array of RMS current values (in mA) for each phase.
 *
 * @note This implementation creates a simple ON/OFF behavior for the PHASE_RELAY_PINS based on
 * the `INDIVIDUAL_LIMIT`. For advanced "Dynamic Load Management" as described in the methodology,
 * which often involves hysteresis (different ON/OFF thresholds) and a sustained delay for
 * deactivation, further logic would be required here. Currently, T2 will engage if any phase
 * hits 1000mA, and disengage immediately when all phases drop below 1000mA.
 */
void handleIndividualProtection(float currents[3]) {
  bool overCurrent = false; // Flag to track if any phase is individually overcurrent.
  // Check if any phase current exceeds the `INDIVIDUAL_LIMIT`.
  for (int i = 0; i < 3; i++) {
    if (currents[i] > INDIVIDUAL_LIMIT) {
      overCurrent = true; // Set flag if an individual overcurrent condition is found.
      break; // Exit the loop as soon as one overcurrent phase is detected.
    }
  }
  
  // Control individual phase relays (e.g., for Transformer 2 switching):
  // If `overCurrent` is true (meaning at least one phase is over `INDIVIDUAL_LIMIT`),
  // set relays to LOW (e.g., engage T2 or open individual phase control circuit).
  // Otherwise (if `overCurrent` is false, meaning all currents are below `INDIVIDUAL_LIMIT`),
  // set relays to HIGH (e.g., disengage T2 or close individual phase control circuit).
  for (int i = 0; i < 3; i++) {
    digitalWrite(PHASE_RELAY_PINS[i], overCurrent ? LOW : HIGH); // LOW for active/engage, HIGH for inactive/disengage
  }
}


/**
 * @brief Performs a basic calibration for the current sensors (ACS712).
 * This function reads analog values from the current sensor pins when no load is
 * connected to determine the zero-current offset (bias) for each sensor.
 * It is absolutely crucial to run this routine with no loads connected to the transformer
 * to ensure accurate zero-point detection.
 */
void calibrateSensors() {
  Serial.println("Calibrating Current Sensors... (Please ensure no loads are connected to the transformer.)");
  delay(2000); // Pause for 2 seconds, giving the user time to disconnect any active loads.

  // Iterate through each current sensor to perform calibration.
  for (int i = 0; i < 3; i++) {
    long sum = 0; // Variable to accumulate raw ADC readings.
    // Read 500 samples from the current sensor pin. Averaging multiple readings helps
    // to reduce noise and get a more stable zero-point.
    for (int j = 0; j < 500; j++) {
      sum += analogRead(PHASE_CURR_PINS[i]); // Read the analog value.
      delay(2); // Small delay between readings to ensure distinct samples.
    }
    zeroPoints[i] = sum / 500; // Calculate the average and store it as the zero-point offset for this specific sensor.
    Serial.printf("Phase %d Current Sensor Zero Point: %d\n", i + 1, zeroPoints[i]); // Print the calibrated zero-point.
  }
  Serial.println("Calibration Complete."); // Indicate that the calibration process has finished.
}

// ===================== Setup Function =====================
/**
 * @brief Arduino setup function. This function runs only once when the ESP32 starts up.
 * It is responsible for initializing serial communication, starting sensor libraries,
 * configuring pin modes for output controls (relays), and performing initial sensor calibration.
 */
void setup() {
  Serial.begin(115200);      // Initialize serial communication at a baud rate of 115200 for debugging output to the Serial Monitor.
  analogReadResolution(12);  // Set the Analog-to-Digital Converter (ADC) resolution to 12 bits (0-4095) for more precise sensor readings on the ESP32.

  // Initialize Blynk connection:
  Serial.println("Connecting to Blynk...");
  // `Blynk.begin()` attempts to connect to the Blynk server using the provided authentication token, WiFi SSID, and password.
  Blynk.begin(auth, ssid, pass);

  // Initialize DS18B20 temperature sensor:
  sensors.begin(); // Start the DallasTemperature sensor library to detect and initialize the DS18B20 sensor.
  // Configure the `TEMP_RELAY_PIN` as an output pin. This pin controls the cooling system relay.
  pinMode(TEMP_RELAY_PIN, OUTPUT);
  // Initialize `TEMP_RELAY_PIN` to HIGH. Assuming HIGH means the cooling system is OFF (relay inactive)
  // This ensures a safe default state where cooling is not running initially.
  digitalWrite(TEMP_RELAY_PIN, HIGH); 

  // Initialize Voltage Sensor sensitivity and all Relay pins:
  // This loop iterates for each of the three phases.
  for (int i = 0; i < 3; i++) {
    // Apply the defined `SENSITIVITY` to each `ZMPT101B` voltage sensor for accurate readings.
    voltSensors[i].setSensitivity(SENSITIVITY);
    
    // Configure individual `PHASE_RELAY_PINS` as output pins. These control Transformer 2 switching or individual phase loads.
    pinMode(PHASE_RELAY_PINS[i], OUTPUT);
    // Initialize `PHASE_RELAY_PINS` to HIGH. Assuming HIGH means the phase relay is OFF (inactive/T2 disconnected)
    // for a safe initial state.
    digitalWrite(PHASE_RELAY_PAYS[i], HIGH); 
    
    // Configure global `GLOBAL_RELAY_PINS` as output pins. These control the main load disconnection for protection.
    pinMode(GLOBAL_RELAY_PINS[i], OUTPUT);
    // Initialize `GLOBAL_RELAY_PINS` to HIGH. Assuming HIGH means the global relay is OFF (inactive/main load connected)
    // for a safe initial state.
    digitalWrite(GLOBAL_RELAY_PINS[i], HIGH); 
  }

  // Run the sensor calibration routine. It is critically important that no active loads
  // are connected to the transformer during this calibration process to ensure accuracy.
  calibrateSensors();
  Serial.println("System Ready - Entering Loop."); // Print a message to the Serial Monitor indicating that setup is complete and the main loop will begin execution.
}

// ===================== Loop Function =====================
/**
 * @brief Arduino loop function. This function runs continuously after the `setup()` function completes.
 * It acts as the main execution cycle of the program, continuously handling Blynk communication,
 * reading sensor data, applying control logic for protection and cooling, and transmitting data.
 */
void loop() {
  Blynk.run(); // This essential function maintains the connection to the Blynk server and processes
               // any incoming commands from the Blynk app or data sent from the ESP32 to Blynk.
               // It should be called as frequently as possible to ensure responsiveness.

  // Static variable to track the last time sensor data was read.
  // Using `static` ensures its value persists across loop calls.
  // This implements a non-blocking delay mechanism, allowing other tasks to run.
  static unsigned long lastRead = 0;
  
  // Check if `200 milliseconds` have passed since the last sensor reading.
  // This defines the update rate for sensor data acquisition and control logic execution.
  if (millis() - lastRead >= 200) {
    lastRead = millis(); // Update the timestamp of the last sensor read to the current `millis()` value.

    float currents[3], voltages[3]; // Declare arrays to store the measured current and voltage values for each phase.
    readSensors(currents, voltages); // Call the `readSensors` function to populate `currents` and `voltages` arrays with fresh data.

    // Update temperature reading and apply smoothing:
    sensors.requestTemperatures();         // Command the `DS18B20` sensor to perform a temperature conversion.
    float rawTemp = sensors.getTempCByIndex(0); // Retrieve the temperature in Celsius from the first connected `DS18B20` sensor.
    
    // Simple moving average calculation for temperature smoothing:
    // This helps to filter out noise and provide a more stable temperature reading.
    // 1. Subtract the contribution of the oldest reading in the `tempReadings` buffer from the `smoothedTemp` total.
    smoothedTemp -= tempReadings[currentIndex] / SMOOTHING_READINGS;
    // 2. Store the new `rawTemp` reading in the circular buffer at the `currentIndex`.
    tempReadings[currentIndex] = rawTemp;
    // 3. Add the contribution of the new reading to the `smoothedTemp` total.
    smoothedTemp += tempReadings[currentIndex] / SMOOTHING_READINGS;
    // 4. Move to the next index in the circular buffer for the next reading, wrapping around if the end is reached.
    currentIndex = (currentIndex + 1) % SMOOTHING_READINGS;

    // Control cooling system based on smoothed temperature:
    // If the `smoothedTemp` exceeds the defined `TEMP_LIMIT`, activate the cooling relay by setting `TEMP_RELAY_PIN` to LOW.
    // Otherwise (if temperature is at or below the limit), deactivate the cooling relay by setting `TEMP_RELAY_PIN` to HIGH.
    digitalWrite(TEMP_RELAY_PIN, smoothedTemp > TEMP_LIMIT ? LOW : HIGH); // LOW for ON, HIGH for OFF

    // Handle overcurrent protections:
    handleGlobalProtection(currents);     // Execute the global overcurrent protection logic.
    handleIndividualProtection(currents); // Execute the individual phase protection/dynamic load management logic.

    // Send sensor data to Blynk virtual pins for remote monitoring and visualization on the Blynk dashboard.
    Blynk.virtualWrite(BLYNK_V_TEMP, smoothedTemp); // Send smoothed temperature to Virtual Pin V0.
    for (int i = 0; i < 3; i++) {
      Blynk.virtualWrite(BLYNK_V_VOLT1 + i, voltages[i]); // Send voltages to Virtual Pins V1, V2, V3.
      Blynk.virtualWrite(BLYNK_V_CURR1 + i, currents[i]); // Send currents to Virtual Pins V4, V5, V6.
    }

    // Print current sensor data to the Serial Monitor for local debugging and real-time feedback.
    // This formatted string includes temperature, all three voltages, and all three currents.
    Serial.printf("Temp: %.1fC | V: %.1f/%.1f/%.1fV | I: %.0f/%.0f/%.0fmA\n",
                  smoothedTemp, voltages[0], voltages[1], voltages[2],
                  currents[0], currents[1], currents[2]);
  }
}