/**
 * @file SPEEDOMETER.ino
 * @brief OptiVelo - ESP32 Optical Bicycle Speedometer, Tachometer & Odometer with OLED Display
 * 
 * Hardware:
 *   - Microcontroller: ESP32 (NodeMCU / ESP32-WROOM-32)
 *   - Sensor: 4-pin TCRT5000 Reflective Infrared Sensor Module
 *       * VCC -> 3.3V (avoids 5V logic level mismatch with ESP32 GPIO)
 *       * GND -> GND
 *       * D0  -> GPIO 4 (Edge-triggered hardware interrupt input)
 *       * A0  -> GPIO 34 (Optional analog read for ambient light / calibration)
 *   - Wheel Marker: High-contrast retroreflective tape / Aluminum tape
 *   - Display: 0.96" or 1.3" I2C Monochrome OLED (SSD1306 / SH1106, 128x64)
 *       * VCC -> 3.3V (or 5V / VIN)
 *       * GND -> GND
 *       * SCK / SCL -> GPIO 22 (Default ESP32 I2C Clock)
 *       * SDA -> GPIO 21 (Default ESP32 I2C Data)
 * 
 * Libraries Required:
 *   - U8g2 by olikraus (Install via Arduino Library Manager)
 *   - Wire (Built-in)
 * 
 * Core Features:
 *   - Hardware edge-triggered interrupt service routine (ISR) placed in IRAM
 *   - Microsecond hardware debounce lockout to reject optical bounce / edge chatter
 *   - Thread-safe FreeRTOS critical sections for atomic ISR data retrieval
 *   - Real-time kinematic calculations:
 *       * Linear speed (v = ds / dt) in km/h and m/s
 *       * Rotational velocity (RPM and rad/s)
 *       * Total cumulative travel distance (m and km)
 *   - Automatic stopped-state zeroing on timeout (no spin detected)
 *   - High-contrast graphical OLED dashboard:
 *       * Inverted status header with [RIDING] / [STOPPED] indicator
 *       * Large high-visibility speedometer readout (km/h & m/s)
 *       * Cadence tachometer (RPM) and odometer (km)
 *       * Real-time graphical velocity progress gauge bar (0 to 50 km/h)
 *   - Optional ambient sunlight calibration helper via A0 pin
 */

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

// ==========================================
// 1. PINOUT CONFIGURATION
// ==========================================
constexpr int SENSOR_DO_PIN = 4;    // Digital interrupt pin (LM393 output from TCRT5000)
constexpr int SENSOR_AO_PIN = 34;   // Optional analog pin for optical ambient calibration (-1 to disable)
constexpr int STATUS_LED_PIN = 2;   // Onboard ESP32 status LED (blinks on valid wheel tick)

// I2C OLED Display Pins (VCC, GND, SCK, SDA)
constexpr int OLED_SDA_PIN  = 21;   // I2C Serial Data (SDA)
constexpr int OLED_SCL_PIN  = 22;   // I2C Serial Clock (SCK / SCL)

// ==========================================
// 2. BIKE & KINEMATIC PARAMETERS
// ==========================================
// Measure wheel rollout on ground: circumference in meters (e.g., 2.070 m for 26" x 2.0, 2.105 m for 700x25C)
constexpr float WHEEL_CIRCUMFERENCE_M = 2.070f; 

// Number of reflective markers adhered to the wheel rim/spoke (standard = 1)
constexpr uint8_t PULSES_PER_REV = 1;

// Distance covered per detected marker pulse (meters)
constexpr float DISTANCE_PER_PULSE_M = WHEEL_CIRCUMFERENCE_M / PULSES_PER_REV;

// ==========================================
// 3. TIMING & DEBOUNCE LIMITS
// ==========================================
// Debounce lockout in microseconds (e.g. 20,000 us = 20 ms).
// At 1 pulse/rev and 2.07 m circumference:
// 20 ms lockout corresponds to a theoretical max speed of ~372 km/h.
// Any pulses arriving closer than 20 ms are rejected as optical/edge noise.
constexpr uint32_t MIN_DEBOUNCE_INTERVAL_US = 20000; 

// Timeout in microseconds without pulses before speed/RPM resets to zero (e.g. 2.5 seconds).
// 2.5 seconds corresponds to a minimum measurable speed of ~2.98 km/h.
constexpr uint32_t STOP_TIMEOUT_US = 2500000;

// Telemetry output update interval in milliseconds
constexpr uint32_t TELEMETRY_INTERVAL_MS = 250; // 4 Hz serial refresh rate
constexpr uint32_t OLED_REFRESH_INTERVAL_MS = 100; // 10 Hz OLED screen refresh rate

// ==========================================
// 4. DISPLAY INITIALIZATION (U8g2)
// ==========================================
// Full buffer hardware I2C driver for SSD1306 128x64 OLED
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// ==========================================
// 5. INTERRUPT SERVICE ROUTINE (ISR) STATE
// ==========================================
// Spinlock critical section for atomic 32-bit multi-variable transfer across cores
static portMUX_TYPE isrMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t isrLastPulseTimeUs = 0; // Timestamp of previous valid pulse
volatile uint32_t isrPeriodUs        = 0; // Period between the last two valid pulses (dt)
volatile uint32_t isrTotalPulses     = 0; // Cumulative pulse counter for odometer
volatile bool     isrPulseFlag       = false; // Signals new edge to main loop

/**
 * @brief Edge-triggered Hardware Interrupt Service Routine
 * Executes in IRAM for deterministic low-latency execution on ESP32.
 * The TCRT5000 LM393 D0 pin pulls LOW when retroreflective tape is detected.
 */
void IRAM_ATTR onWheelSensorPulse() {
    uint32_t nowUs = micros();

    // Hardware edge-triggered debounce lockout filter
    if (nowUs - isrLastPulseTimeUs >= MIN_DEBOUNCE_INTERVAL_US) {
        if (isrLastPulseTimeUs > 0) {
            isrPeriodUs = nowUs - isrLastPulseTimeUs;
        }
        isrLastPulseTimeUs = nowUs;
        isrTotalPulses++;
        isrPulseFlag = true;
    }
}

// ==========================================
// 6. RUNTIME STATE VARIABLES
// ==========================================
float currentSpeedKmh   = 0.0f;
float currentSpeedMs    = 0.0f;
float currentRpm        = 0.0f;
float currentRadSec     = 0.0f;
float totalDistanceM    = 0.0f;
float totalDistanceKm   = 0.0f;

uint32_t lastTelemetryTimeMs = 0;
uint32_t lastOledUpdateMs    = 0;
uint32_t lastLedPulseMs      = 0;

// ==========================================
// 7. OLED RENDERING ENGINE
// ==========================================
/**
 * @brief Draws the complete dashboard on the 128x64 OLED screen.
 */
void renderOledDashboard() {
    u8g2.clearBuffer();

    // ----------------------------------------------------
    // Section 1: Inverted Header Bar
    // ----------------------------------------------------
    u8g2.setDrawColor(1);
    u8g2.drawBox(0, 0, 128, 11);
    u8g2.setDrawColor(0); // White text inside black header box
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(3, 9, "OPTIVELO");

    // Dynamic Riding State
    if (currentSpeedKmh > 0.5f) {
        u8g2.drawStr(78, 9, "[RIDING]");
    } else {
        u8g2.drawStr(72, 9, "[STOPPED]");
    }

    u8g2.setDrawColor(1); // Restore standard drawing color

    // ----------------------------------------------------
    // Section 2: Large Speedometer Display
    // ----------------------------------------------------
    char speedStr[10];
    snprintf(speedStr, sizeof(speedStr), "%.1f", currentSpeedKmh);
    u8g2.setFont(u8g2_font_logisoso24_tf); // High-contrast large digits
    u8g2.drawStr(4, 38, speedStr);

    // Units label (km/h)
    u8g2.setFont(u8g2_font_7x14B_tf);
    u8g2.drawStr(78, 27, "km/h");

    // Secondary speed (m/s)
    char msStr[12];
    snprintf(msStr, sizeof(msStr), "%.1fm/s", currentSpeedMs);
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(80, 38, msStr);

    // Divider line
    u8g2.drawHLine(0, 42, 128);

    // ----------------------------------------------------
    // Section 3: Cadence (RPM) and Trip Distance
    // ----------------------------------------------------
    u8g2.setFont(u8g2_font_6x10_tf);
    char cadStr[16];
    snprintf(cadStr, sizeof(cadStr), "RPM:%3.0f", currentRpm);
    u8g2.drawStr(3, 52, cadStr);

    char distStr[16];
    if (totalDistanceKm < 10.0f) {
        snprintf(distStr, sizeof(distStr), "DST:%4.2fkm", totalDistanceKm);
    } else {
        snprintf(distStr, sizeof(distStr), "DST:%4.1fkm", totalDistanceKm);
    }
    u8g2.drawStr(64, 52, distStr);

    // ----------------------------------------------------
    // Section 4: Speedometer Gauge Bar (0 to 50 km/h)
    // ----------------------------------------------------
    u8g2.drawFrame(0, 56, 128, 7);
    int gaugeWidth = (int)((constrain(currentSpeedKmh, 0.0f, 50.0f) / 50.0f) * 124.0f);
    if (gaugeWidth > 0) {
        u8g2.drawBox(2, 58, gaugeWidth, 3);
    }

    u8g2.sendBuffer();
}

// ==========================================
// 8. SETUP
// ==========================================
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println(F("=================================================="));
    Serial.println(F("   OptiVelo - Optical ESP32 Speedo & Tachometer   "));
    Serial.println(F("=================================================="));
    Serial.printf("Wheel Circumference: %.3f m\n", WHEEL_CIRCUMFERENCE_M);
    Serial.printf("Pulses per Rev:      %u\n", PULSES_PER_REV);
    Serial.printf("Debounce Lockout:    %u us\n", MIN_DEBOUNCE_INTERVAL_US);
    Serial.printf("Stop Timeout:        %.1f s\n", STOP_TIMEOUT_US / 1000000.0f);
    Serial.println(F("--------------------------------------------------"));

    // Configure GPIOs
    pinMode(SENSOR_DO_PIN, INPUT_PULLUP);
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);

    if (SENSOR_AO_PIN >= 0) {
        pinMode(SENSOR_AO_PIN, INPUT);
    }

    // Initialize I2C Bus for OLED
    Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
    Wire.setClock(400000); // 400 kHz Fast I2C

    // Initialize OLED Display
    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_helvB10_tf);
    u8g2.drawStr(24, 22, "OptiVelo");
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(12, 38, "Optical Cyclometer");
    u8g2.drawStr(18, 52, "Sensor Initialized");
    u8g2.sendBuffer();
    delay(1500);

    // Attach edge-triggered hardware interrupt
    // TCRT5000 D0 transitions to FALLING when infrared reflection is detected
    attachInterrupt(digitalPinToInterrupt(SENSOR_DO_PIN), onWheelSensorPulse, FALLING);

    Serial.println(F("[SYSTEM] Hardware interrupt attached to GPIO 4 (FALLING)."));
    Serial.println(F("[SYSTEM] OLED Display initialized on GPIO 21 (SDA) & GPIO 22 (SCK)."));
    Serial.println(F("[SYSTEM] Ready. Spin wheel to measure velocity...\n"));
}

// ==========================================
// 9. MAIN LOOP
// ==========================================
void loop() {
    uint32_t nowUs = micros();
    uint32_t nowMs = millis();

    // ----------------------------------------------------
    // Step A: Atomically retrieve ISR telemetry in critical section
    // ----------------------------------------------------
    uint32_t localPeriodUs;
    uint32_t localLastPulseUs;
    uint32_t localTotalPulses;
    bool localPulseTriggered;

    portENTER_CRITICAL(&isrMux);
    localPeriodUs        = isrPeriodUs;
    localLastPulseUs     = isrLastPulseTimeUs;
    localTotalPulses     = isrTotalPulses;
    localPulseTriggered  = isrPulseFlag;
    isrPulseFlag         = false; // Clear flag
    portEXIT_CRITICAL(&isrMux);

    // ----------------------------------------------------
    // Step B: Pulse feedback indicator (LED flash)
    // ----------------------------------------------------
    if (localPulseTriggered) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        lastLedPulseMs = nowMs;
    }
    if (nowMs - lastLedPulseMs > 30) {
        digitalWrite(STATUS_LED_PIN, LOW); // Turn off LED after 30ms pulse
    }

    // ----------------------------------------------------
    // Step C: Speed and Tachometer Kinematic Calculations
    // ----------------------------------------------------
    // Check if the wheel has stopped (timeout condition)
    if (localLastPulseUs == 0 || (nowUs - localLastPulseUs > STOP_TIMEOUT_US)) {
        currentSpeedKmh = 0.0f;
        currentSpeedMs  = 0.0f;
        currentRpm      = 0.0f;
        currentRadSec   = 0.0f;
    } else if (localPeriodUs > 0) {
        // Delta t for a single pulse in seconds
        float dtSec = (float)localPeriodUs / 1000000.0f;

        // Linear Speed: v = ds / dt
        currentSpeedMs  = DISTANCE_PER_PULSE_M / dtSec;
        currentSpeedKmh = currentSpeedMs * 3.6f;

        // Rotational Velocity (Tachometer):
        // Period per full revolution = dtSec * PULSES_PER_REV
        float revPeriodSec = dtSec * PULSES_PER_REV;
        currentRpm    = 60.0f / revPeriodSec;
        currentRadSec = currentRpm * (2.0f * PI / 60.0f);
    }

    // Cumulative Travel Distance (Odometer):
    totalDistanceM  = (float)localTotalPulses * DISTANCE_PER_PULSE_M;
    totalDistanceKm = totalDistanceM / 1000.0f;

    // ----------------------------------------------------
    // Step D: Periodic OLED Screen Refresh (10 Hz)
    // ----------------------------------------------------
    if (nowMs - lastOledUpdateMs >= OLED_REFRESH_INTERVAL_MS) {
        lastOledUpdateMs = nowMs;
        renderOledDashboard();
    }

    // ----------------------------------------------------
    // Step E: Periodic Telemetry Stream (Serial, 4 Hz)
    // ----------------------------------------------------
    if (nowMs - lastTelemetryTimeMs >= TELEMETRY_INTERVAL_MS) {
        lastTelemetryTimeMs = nowMs;

        Serial.printf("[OptiVelo] Speed: %5.1f km/h (%4.1f m/s) | RPM: %5.1f | ω: %4.1f rad/s | Dist: %6.1f m (%5.2f km) | Pulses: %u",
                      currentSpeedKmh,
                      currentSpeedMs,
                      currentRpm,
                      currentRadSec,
                      totalDistanceM,
                      totalDistanceKm,
                      localTotalPulses);

        // Optional diagnostic reading from analog pin A0 to inspect ambient sunlight / contrast
        if (SENSOR_AO_PIN >= 0) {
            int rawAnalog = analogRead(SENSOR_AO_PIN);
            Serial.printf(" | IR_A0: %4d", rawAnalog);
        }

        Serial.println();
    }
}
