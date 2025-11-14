// Custom TX/RX Main
// Use BUILD_TX or BUILD_RX build flag to select mode

#include <Arduino.h>
#include "config.h"  // Unified configuration file
#include "protocol.h"

#ifdef BUILD_TX
#include "imu_handler.h"
#include "joystick_handler.h"
#endif

// Build mode check
#if !defined(BUILD_TX) && !defined(BUILD_RX)
  #error "Please define either BUILD_TX or BUILD_RX in platformio.ini"
#endif

#ifdef BUILD_TX
// ============================================================
// TRANSMITTER (TX) CODE
// ============================================================
// All TX configuration moved to config.h

// Control handlers
IMUHandler imu;
JoystickHandler joystick;

// Broadcast to all receivers (no pairing needed!)
uint8_t receiverMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// RC channel values (CRSF format: 172-1811, center: 992)
uint16_t rcChannels[16] = {
  992, 992, 172, 992, 992, 992, 992, 992,  // Channels 1-8
  992, 992, 992, 992, 992, 992, 992, 992   // Channels 9-16
};

// Arming logic
bool isArmed = false;
unsigned long auxSwitchTime = 0;  // Time when AUX last switched or will switch
bool currentAuxState = false;  // false = LOW (disarmed), true = HIGH (armed)

// Bench test variables
float benchTestPhase = 0.0;
const float BENCH_TEST_SPEED = 0.5;  // Radians per second

// AUX cycling parameters
const unsigned long AUX_STATE_DURATION = 10000;  // 10 seconds per state
const unsigned long PRE_SWITCH_PAUSE = 1000;     // 1 second pause before switching

// Timing
unsigned long lastRcSendMs = 0;
unsigned long lastHeartbeatMs = 0;
unsigned long lastStatsMs = 0;
unsigned long lastTxErrorMs = 0;

const unsigned long RC_SEND_INTERVAL = 1000 / RC_SEND_FREQUENCY_HZ;  // Auto-calculated from frequency
const unsigned long ERROR_PRINT_INTERVAL = 1000; // Max 1 error per second

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n=================================");
  Serial.println("Custom Protocol Transmitter (TX)");
  Serial.println("=================================\n");
  
  // Initialize protocol (TX mode, broadcast)
  if (!CustomProtocol_Init(true, receiverMac)) {
    Serial.println("[TX] Protocol init failed!");
    while (1) {
      delay(1000);
    }
  }
  
  Serial.println("[TX] Ready to transmit!");
  Serial.println("[TX] Broadcasting to all receivers");
  Serial.printf("[TX] RC commands: %d Hz | Heartbeat: 2 Hz | Stats: 1 Hz\n", RC_SEND_FREQUENCY_HZ);
  
  if (USE_IMU_JOYSTICK_INPUT) {
    Serial.println("\n*** IMU + JOYSTICK CONTROL MODE ***");
    Serial.println("Control Scheme (FPV Drone Style):");
    Serial.println("  IMU (Tilt):  Roll + Pitch");
    Serial.println("  Joystick:    Throttle + Yaw");
    
    // Display selected IMU type
    if (IMU_TYPE == IMU_BNO085) {
      Serial.println("  IMU Type:    BNO085 (Advanced 9-axis with sensor fusion)");
    } else {
      Serial.println("  IMU Type:    MPU6050 (Basic 6-axis)");
    }
    Serial.println();
    
    // Initialize IMU for roll/pitch control
    if (!imu.begin(IMU_SDA_PIN, IMU_SCL_PIN, IMU_TYPE)) {
      Serial.println("[TX] IMU initialization failed!");
      while (1) {
        delay(1000);
      }
    }
    
    // Configure IMU sensitivity
    imu.setRollSensitivity(ROLL_SENSITIVITY);
    imu.setPitchSensitivity(PITCH_SENSITIVITY);
    
    // Initialize Joystick for throttle/yaw control
    joystick.begin(JOYSTICK_THROTTLE_PIN, JOYSTICK_YAW_PIN, JOYSTICK_BUTTON_PIN);
    joystick.setSmoothing(true, 0.3);  // Enable smoothing for stable control
    
    Serial.println("[TX] All controls initialized!");
    Serial.println();
    
  } else if (BENCH_TEST_MODE) {
    Serial.println("\n*** BENCH TEST MODE ACTIVE ***");
    Serial.println("AUX1 cycles every 10 seconds (disarmed <-> armed)");
    Serial.println("Throttle behavior:");
    Serial.println("   - AUX LOW (disarmed): constant 25% throttle");
    Serial.println("   - AUX HIGH (armed): ramps 0% -> 50% over 10 seconds");
    Serial.println("   - 1s before switch: throttle -> 0%, sticks center");
    Serial.println("Starting DISARMED (AUX1 LOW, 25% throttle)");
    auxSwitchTime = millis();
  } else {
    Serial.println("\n*** MANUAL MODE ***");
    Serial.println("Using static channel values");
  }
  
  Serial.println();
}

void loop() {
  unsigned long now = millis();
  
  // Update protocol (handles timeouts, etc.)
  CustomProtocol_Update();
  
  // Handle AUX1 state cycling in bench test mode
  if (BENCH_TEST_MODE) {
    unsigned long timeSinceSwitch = now - auxSwitchTime;
    
    // Time to switch AUX state (every 10 seconds)
    if (timeSinceSwitch >= AUX_STATE_DURATION) {
      currentAuxState = !currentAuxState;  // Toggle AUX state
      auxSwitchTime = now;
      
      if (currentAuxState) {
        Serial.println("\nAUX1 -> HIGH (ARMED)\n");
      } else {
        Serial.println("\nAUX1 -> LOW (DISARMED)\n");
      }
    }
  }
  
  // Send RC commands at configured frequency
  if (now - lastRcSendMs >= RC_SEND_INTERVAL) {
    lastRcSendMs = now;
    
    if (USE_IMU_JOYSTICK_INPUT) {
      // Update control inputs
      imu.update();
      joystick.update();
      
      // Get normalized values from controllers
      // Roll & Pitch: From IMU (tilt control)
      float rollNormalized = imu.getRoll();    // -1.0 to 1.0
      float pitchNormalized = imu.getPitch();  // -1.0 to 1.0
      
      // Throttle & Yaw: From Joystick
      float throttleNormalized = joystick.getThrottle();  // 0.0 to 1.0
      float yawNormalized = joystick.getYaw();            // -1.0 to 1.0
      
      // Map to CRSF RC channel values (172-1811, center: 992, range: ±819)
      rcChannels[0] = (uint16_t)(rollNormalized * 819.0 + 992.0);      // Roll: centered
      rcChannels[1] = (uint16_t)(pitchNormalized * 819.0 + 992.0);     // Pitch: centered
      rcChannels[2] = (uint16_t)(172.0 + throttleNormalized * 1639.0); // Throttle: 0-100%
      rcChannels[3] = (uint16_t)(yawNormalized * 819.0 + 992.0);       // Yaw: centered
      
      // AUX1: Controlled by joystick button (toggle arming state)
      if (joystick.getButtonPressed()) {
        isArmed = !isArmed;  // Toggle armed state on button press
        Serial.printf("\n[ARMING] %s\n\n", isArmed ? "*** ARMED ***" : "*** DISARMED ***");
      }
      rcChannels[4] = isArmed ? 1811 : 172;  // AUX1: Armed state
      rcChannels[5] = 992;   // AUX2: Center
      
    } else if (BENCH_TEST_MODE) {
      // Check if we're in transition period (1 second before AUX switch)
      unsigned long timeSinceSwitch = now - auxSwitchTime;
      bool inTransition = (timeSinceSwitch >= (AUX_STATE_DURATION - PRE_SWITCH_PAUSE));
      
      if (!inTransition) {
        // Normal operation - update phase and movement
        float dt = RC_SEND_INTERVAL / 1000.0;  // Time delta in seconds
        benchTestPhase += BENCH_TEST_SPEED * dt;
        if (benchTestPhase > 6.28318) benchTestPhase -= 6.28318;  // Wrap at 2*PI
      }
      // else: In transition - pause movement (don't update benchTestPhase)
      
      // Calculate smooth sine waves for each axis
      float rollValue = sin(benchTestPhase);                    // Roll oscillates
      float pitchValue = sin(benchTestPhase + 1.57);            // Pitch 90° out of phase
      float yawValue = sin(benchTestPhase * 0.5);               // Yaw slower
      
      // During transition: force everything to center/minimum
      if (inTransition) {
        rollValue = 0.0;
        pitchValue = 0.0;
        yawValue = 0.0;
      }
      
      // Convert -1..1 to CRSF range (172-1811, center 992)
      rcChannels[0] = (uint16_t)(rollValue * 400.0 + 992.0);     // Roll: ±400 from center
      rcChannels[1] = (uint16_t)(pitchValue * 400.0 + 992.0);    // Pitch: ±400 from center
      rcChannels[3] = (uint16_t)(yawValue * 300.0 + 992.0);      // Yaw: ±300 from center
      
      // AUX1 controls arming (channel 4)
      rcChannels[4] = currentAuxState ? 1811 : 172;  // HIGH = armed, LOW = disarmed
      
      // Update isArmed based on AUX1 position
      isArmed = currentAuxState;
      
      // Throttle logic:
      // - During transition (1s before switch): 0% throttle
      // - AUX LOW (disarmed): constant 25% throttle
      // - AUX HIGH (armed): ramp from 0% to 50% over 10 seconds
      float throttlePercent = 0.0;
      const float CRSF_RANGE = 1811.0 - 172.0;  // Total CRSF range
      
      if (inTransition) {
        // Transition: 0% throttle (minimum)
        throttlePercent = 0.0;
      } else if (!isArmed) {
        // Disarmed (AUX LOW): constant 25% throttle
        throttlePercent = 0.25;
      } else {
        // Armed (AUX HIGH): linear ramp 0% to 50% over 10 seconds
        float timeInArmedState = timeSinceSwitch / 1000.0;  // Convert to seconds
        float maxTime = (AUX_STATE_DURATION - PRE_SWITCH_PAUSE) / 1000.0;  // 9 seconds
        throttlePercent = (timeInArmedState / maxTime) * 0.5;  // 0% to 50%
        if (throttlePercent > 0.5) throttlePercent = 0.5;  // Cap at 50%
      }
      
      // Convert throttle percentage to CRSF value
      rcChannels[2] = (uint16_t)(172.0 + throttlePercent * CRSF_RANGE);
      
      rcChannels[5] = 992;  // AUX2: Center
      
    } else {
      // Manual mode - static values (for real joystick input)
      rcChannels[0] = 992;  // Roll center
      rcChannels[1] = 992;  // Pitch center
      rcChannels[2] = 172;  // Throttle minimum
      rcChannels[3] = 992;  // Yaw center
    }
    
    // Send RC command (errors are rate-limited to prevent spam)
    if (!CustomProtocol_SendRcCommand(rcChannels)) {
      // Rate-limit error messages
      if (now - lastTxErrorMs >= ERROR_PRINT_INTERVAL) {
        Serial.println("[TX] Failed to send RC command (rate-limited errors)");
        lastTxErrorMs = now;
      }
    }
  }
  
  // Send heartbeat at 2 Hz
  if (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL) {
    lastHeartbeatMs = now;
    
    if (CustomProtocol_SendHeartbeat()) {
      // Serial.println("[TX] Heartbeat sent");
    }
  }
  
  // Print statistics every second
  if (now - lastStatsMs >= STATS_INTERVAL) {
    lastStatsMs = now;
    
    ProtocolStats stats;
    CustomProtocol_GetStats(&stats);
    
    Serial.println("\n--- TX Statistics ---");
    Serial.printf("Packets Sent: %lu | Received: %lu | Loss: %.1f%%\n", 
                  stats.packetsSent, stats.packetsReceived, stats.packetLossPercent);
    Serial.printf("Link: %s | Last RX: %lu ms ago\n", 
                  stats.linkActive ? "ACTIVE" : "DOWN", stats.lastPacketMs);
    
    // Display telemetry if received
    if (stats.packetsReceived > 0) {
      TelemetryPayload telemetry;
      CustomProtocol_GetTelemetry(&telemetry);
      
      // Show voltage/current or N/A if not available
      if (telemetry.voltage > 0.1) {
        Serial.printf("Battery: %.2fV ", telemetry.voltage);
      } else {
        Serial.print("Battery: N/A ");
      }
      
      if (telemetry.current > 0.1) {
        Serial.printf("%.2fA | ", telemetry.current);
      } else {
        Serial.print("N/A | ");
      }
      
      Serial.printf("RSSI: %ddBm | LQ: %d%%\n", 
                    telemetry.rssi, telemetry.linkQuality);
    }
    
    // Display current RC channels being sent
    Serial.printf("TX Sending RC: R:%d P:%d T:%d Y:%d | AUX1:%d AUX2:%d\n",
                  rcChannels[0], rcChannels[1], rcChannels[2], rcChannels[3],
                  rcChannels[4], rcChannels[5]);
    
    // If using IMU+Joystick, show current sensor readings
    if (USE_IMU_JOYSTICK_INPUT) {
      // Get IMU angle data for debugging
      float rollAngle, pitchAngle;
      imu.getAngles(rollAngle, pitchAngle);
      
      // Get joystick data for debugging
      uint16_t throttleRaw, yawRaw;
      joystick.getRawValues(throttleRaw, yawRaw);
      bool buttonState = joystick.getButton();
      
      Serial.printf("IMU Angles (°): Roll:%.1f Pitch:%.1f | Normalized: R:%.2f P:%.2f\n",
                    rollAngle, pitchAngle, imu.getRoll(), imu.getPitch());
      Serial.printf("Joystick ADC: Throttle:%d Yaw:%d | Output: T:%.2f Y:%.2f | Button:%s\n",
                    throttleRaw, yawRaw, joystick.getThrottle(), joystick.getYaw(),
                    buttonState ? "PRESSED" : "Released");
    }
    
    Serial.println();
  }
  
  // Small delay to prevent watchdog issues
  delay(1);
}

#endif // BUILD_TX

#ifdef BUILD_RX
// ============================================================
// RECEIVER (RX) CODE
// ============================================================
// All RX configuration moved to config.h

#include "universal_bridge.h"

// Universal protocol bridge (handles all protocols)
UniversalBridge* protocolBridge = nullptr;

// Timing
unsigned long lastTelemetrySendMs = 0;
unsigned long lastStatsMs = 0;
unsigned long lastRxErrorMs = 0;
unsigned long lastProtocolUpdateMs = 0;
unsigned long lastLedToggleMs = 0;

const unsigned long ERROR_PRINT_INTERVAL = 1000;    // Max 1 error per second

// LED state
bool ledState = false;

// Telemetry values (will be read from FC or sensors)
// For now using simulated values - replace with actual sensor/FC readings
float batteryVoltage = 0.0;  // Will show N/A if 0
float batteryCurrent = 0.0;  // Will show N/A if 0
int16_t rssi = -45;           // Simulated RSSI
uint8_t linkQuality = 100;    // Calculated from packet stats

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Initialize LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // Start with LED off
  
#if !ENABLE_DEBUG_OUTPUT
  Serial.println("\n\n=================================");
  Serial.println("Custom Protocol Receiver (RX)");
  Serial.println("=================================\n");
#endif
  
  // Initialize protocol (RX mode, will auto-pair with any TX)
  if (!CustomProtocol_Init(false, nullptr)) {
#if !ENABLE_DEBUG_OUTPUT
    Serial.println("[RX] Protocol init failed!");
#endif
    while (1) {
      delay(1000);
    }
  }
  
#if !ENABLE_DEBUG_OUTPUT
  Serial.println("[RX] Protocol initialized successfully");
  Serial.flush();
#endif
  delay(200);
  
  // Initialize universal protocol bridge (automatically selects protocol from config)
  protocolBridge = new UniversalBridge();
  
  if (!protocolBridge || !protocolBridge->init()) {
#if !ENABLE_DEBUG_OUTPUT
    Serial.println("[RX] Protocol initialization failed!");
#endif
    while (1) {
      delay(1000);
    }
  }
  
#if !ENABLE_DEBUG_OUTPUT
  Serial.printf("[RX] Protocol: %s\n", protocolBridge->getName());
#endif
  
#if !ENABLE_DEBUG_OUTPUT
  Serial.println("\n[RX] Ready to receive!");
  Serial.println("[RX] Listening for any transmitter (broadcast mode)");
  Serial.println();
  Serial.flush();
#endif
  
  delay(100);
}

void loop() {
  unsigned long now = millis();
  
  // Update protocol (handles timeouts, parsing, etc.)
  CustomProtocol_Update();
  
  // Update LED status indicator
  if (CustomProtocol_IsLinkActive()) {
    // Link active - LED solid ON
    digitalWrite(LED_PIN, HIGH);
    ledState = true;
  } else {
    // No link - LED blinking
    if (now - lastLedToggleMs >= LED_BLINK_RATE) {
      lastLedToggleMs = now;
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    }
  }
  
  // Update protocol bridge
  if (protocolBridge) {
    protocolBridge->update();
    
    // Send RC channels to FC via selected protocol
    if (protocolBridge->isActive()) {
      if (CustomProtocol_IsLinkActive()) {
        // Get RC channels from protocol
        uint16_t channels[16];
        CustomProtocol_GetRcChannels(channels);
        
        // Send to flight controller via selected protocol
        protocolBridge->sendRcChannels(channels);
      } else {
        // Send failsafe values
        uint16_t failsafeChannels[16] = {
          992, 992, 172, 992, 172, 172, 172, 172,  // Safe values
          172, 172, 172, 172, 172, 172, 172, 172
        };
        protocolBridge->sendRcChannels(failsafeChannels);
      }
    }
  }
  
  // Send telemetry at 10 Hz (only if link is active)
  if (CustomProtocol_IsLinkActive() && (now - lastTelemetrySendMs >= TELEMETRY_SEND_INTERVAL)) {
    lastTelemetrySendMs = now;
    
    // TODO: Read real telemetry from flight controller or sensors
    // For now, sending placeholder values (0.0 will display as N/A on TX)
    batteryVoltage = 0.0;  // TODO: Read from ADC or FC
    batteryCurrent = 0.0;  // TODO: Read from current sensor or FC
    
    // Calculate link quality based on packet stats
    ProtocolStats stats;
    CustomProtocol_GetStats(&stats);
    linkQuality = (stats.linkActive) ? 100 : 0;
    
    // Send telemetry
    if (!CustomProtocol_SendTelemetry(batteryVoltage, batteryCurrent, rssi, linkQuality)) {
#if !ENABLE_DEBUG_OUTPUT
      // Debug mode - print errors
      if (now - lastRxErrorMs >= ERROR_PRINT_INTERVAL) {
        Serial.println("[RX] Failed to send telemetry");
        lastRxErrorMs = now;
      }
#endif
    }
  }
  
#if !ENABLE_DEBUG_OUTPUT
  // Debug mode - Print statistics every second
  if (now - lastStatsMs >= STATS_INTERVAL) {
    lastStatsMs = now;
    
    ProtocolStats stats;
    CustomProtocol_GetStats(&stats);
    
    Serial.println("\n--- RX Statistics ---");
    if (protocolBridge) {
      Serial.printf("Output Protocol: %s | Frames Sent: %lu\n",
                    protocolBridge->getName(), protocolBridge->getFramesSent());
    }
    Serial.printf("Packets RX: %lu | TX: %lu | Loss: %.1f%% | CRC Err: %lu\n", 
                  stats.packetsReceived, stats.packetsSent, 
                  stats.packetLossPercent, stats.crcErrors);
    Serial.printf("Link: %s | Last RX: %lu ms ago\n", 
                  stats.linkActive ? "ACTIVE" : "FAILSAFE", stats.lastPacketMs);
    
    // Display RC channels if link is active
    if (stats.linkActive) {
      uint16_t channels[16];
      CustomProtocol_GetRcChannels(channels);
      
      Serial.printf("RX Received RC: R:%d P:%d T:%d Y:%d | AUX1:%d AUX2:%d\n",
                    channels[0], channels[1], channels[2], channels[3],
                    channels[4], channels[5]);
      Serial.printf("All 16 channels: %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
                    channels[0], channels[1], channels[2], channels[3],
                    channels[4], channels[5], channels[6], channels[7],
                    channels[8], channels[9], channels[10], channels[11],
                    channels[12], channels[13], channels[14], channels[15]);
    } else {
      Serial.println("*** FAILSAFE MODE - No signal ***");
    }
    
    Serial.println();
  }
#endif
  
  // Small delay to prevent watchdog issues
  delay(1);
}

#endif // BUILD_RX
