#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#ifndef RGB_PIN
#define RGB_PIN 21  
#endif
#define NUM_PIXELS 1

Adafruit_NeoPixel led(NUM_PIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);

// CRSF protocol constants
#define CRSF_SYNC_BYTE 0xC8
#define CRSF_FRAMETYPE_RC_CHANNELS 0x16
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8
#define CRSF_ADDRESS_BROADCAST 0x00

// UART pins for flight controller
constexpr int FC_TX_PIN = 43;  // ESP32 TX -> connects to FC RX (A10 for UART1)
constexpr int FC_RX_PIN = 44;  // ESP32 RX (not used but needed for init)
constexpr int BAUD_RATE = 420000;
HardwareSerial CRSF_FC(1);

// RC channels data - CRSF uses 11-bit values (0-2047, center=1024)
// Standard RC values: 172=1000us(min), 992=1500us(center), 1811=2000us(max)
// 
// AETR1 Channel Mapping (configured in Betaflight):
// CH1 = Aileron (Roll)    - stick left/right
// CH2 = Elevator (Pitch)  - stick forward/back
// CH3 = Throttle          - stick up/down
// CH4 = Rudder (Yaw)      - stick rotate
// CH5 = AUX1 (Arm switch) - arm/disarm
//
uint16_t rcChannels[16] = {
  992,  // CH1 - Aileron/Roll (center = no roll)
  992,  // CH2 - Elevator/Pitch (center = no pitch)
  172,  // CH3 - Throttle (min = 0% throttle) ⚠️ SAFETY
  992,  // CH4 - Rudder/Yaw (center = no yaw)
  172,  // CH5 - AUX1/Arm (low = DISARMED) ⚠️ SAFETY
  992,  // CH6 - AUX2
  992,  // CH7 - AUX3
  992,  // CH8 - AUX4
  992,  // CH9
  992,  // CH10
  992,  // CH11
  992,  // CH12
  992,  // CH13
  992,  // CH14
  992,  // CH15
  992   // CH16
};

uint32_t testStartTime = 0;
int testPhase = 0;
uint32_t packetsSent = 0;

uint8_t crc8(const uint8_t* data, size_t len) {
  uint8_t c = 0x00;
  for (size_t i = 0; i < len; ++i) {
    c ^= data[i];
    for (int b = 0; b < 8; ++b)
      c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0xD5) : (uint8_t)(c << 1);
  }
  return c;
}

void updateTestCommands() {
  uint32_t elapsed = millis() - testStartTime;
  testPhase = (elapsed / 3000) % 5;
  
  switch(testPhase) {
    case 0:  // Neutral position
      rcChannels[0] = 992;  // Roll center
      rcChannels[1] = 992;  // Pitch center
      rcChannels[2] = 172;  // Throttle minimum
      rcChannels[3] = 992;  // Yaw center
      rcChannels[4] = 172;  // Arm switch OFF
      Serial.println("\n=== TEST PHASE 0: All neutral, DISARMED ===");
      break;
      
    case 1:  // Roll right
      rcChannels[0] = 1400; // Roll right
      rcChannels[1] = 992;  // Pitch center
      rcChannels[2] = 172;  // Throttle minimum
      rcChannels[3] = 992;  // Yaw center
      rcChannels[4] = 172;  // Arm switch OFF
      Serial.println("\n=== TEST PHASE 1: Roll RIGHT ===");
      break;
      
    case 2:  // Pitch forward
      rcChannels[0] = 992;  // Roll center
      rcChannels[1] = 1400; // Pitch forward
      rcChannels[2] = 172;  // Throttle minimum
      rcChannels[3] = 992;  // Yaw center
      rcChannels[4] = 172;  // Arm switch OFF
      Serial.println("\n=== TEST PHASE 2: Pitch FORWARD ===");
      break;
      
    case 3:  // Yaw right
      rcChannels[0] = 992;  // Roll center
      rcChannels[1] = 992;  // Pitch center
      rcChannels[2] = 172;  // Throttle minimum
      rcChannels[3] = 1400; // Yaw right
      rcChannels[4] = 172;  // Arm switch OFF
      Serial.println("\n=== TEST PHASE 3: Yaw RIGHT ===");
      break;
      
    case 4:  // Try ARM (throttle still low for safety)
      rcChannels[0] = 992;  // Roll center
      rcChannels[1] = 992;  // Pitch center
      rcChannels[2] = 172;  // Throttle minimum (SAFE)
      rcChannels[3] = 992;  // Yaw center
      rcChannels[4] = 1811; // Arm switch HIGH (ARMED!)
      Serial.println("\n=== TEST PHASE 4: ARM SWITCH HIGH (throttle still low) ===");
      Serial.println("WARNING: This will ARM if Betaflight is configured!");
      break;
  }
  
  Serial.printf("Channels: R=%d P=%d T=%d Y=%d ARM=%d\n", 
                rcChannels[0], rcChannels[1], rcChannels[2], rcChannels[3], rcChannels[4]);
}

void sendRCChannelsToFC() {
  uint8_t frame[26];
  
  frame[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;  // Destination address
  frame[1] = 24;  // Frame length (type + payload + crc)
  frame[2] = CRSF_FRAMETYPE_RC_CHANNELS;  // Frame type
  
  // Pack 16 channels (11-bit each) into bytes
  frame[3]  = (uint8_t) ((rcChannels[0])       & 0x07FF);
  frame[4]  = (uint8_t) ((rcChannels[0])       & 0x07FF) >> 8 | (rcChannels[1] & 0x07FF) << 3;
  frame[5]  = (uint8_t) ((rcChannels[1])       & 0x07FF) >> 5 | (rcChannels[2] & 0x07FF) << 6;
  frame[6]  = (uint8_t) ((rcChannels[2])       & 0x07FF) >> 2;
  frame[7]  = (uint8_t) ((rcChannels[2])       & 0x07FF) >> 10 | (rcChannels[3] & 0x07FF) << 1;
  frame[8]  = (uint8_t) ((rcChannels[3])       & 0x07FF) >> 7 | (rcChannels[4] & 0x07FF) << 4;
  frame[9]  = (uint8_t) ((rcChannels[4])       & 0x07FF) >> 4 | (rcChannels[5] & 0x07FF) << 7;
  frame[10] = (uint8_t) ((rcChannels[5])       & 0x07FF) >> 1;
  frame[11] = (uint8_t) ((rcChannels[5])       & 0x07FF) >> 9 | (rcChannels[6] & 0x07FF) << 2;
  frame[12] = (uint8_t) ((rcChannels[6])       & 0x07FF) >> 6 | (rcChannels[7] & 0x07FF) << 5;
  frame[13] = (uint8_t) ((rcChannels[7])       & 0x07FF) >> 3;
  frame[14] = (uint8_t) ((rcChannels[8])       & 0x07FF);
  frame[15] = (uint8_t) ((rcChannels[8])       & 0x07FF) >> 8 | (rcChannels[9] & 0x07FF) << 3;
  frame[16] = (uint8_t) ((rcChannels[9])       & 0x07FF) >> 5 | (rcChannels[10] & 0x07FF) << 6;
  frame[17] = (uint8_t) ((rcChannels[10])      & 0x07FF) >> 2;
  frame[18] = (uint8_t) ((rcChannels[10])      & 0x07FF) >> 10 | (rcChannels[11] & 0x07FF) << 1;
  frame[19] = (uint8_t) ((rcChannels[11])      & 0x07FF) >> 7 | (rcChannels[12] & 0x07FF) << 4;
  frame[20] = (uint8_t) ((rcChannels[12])      & 0x07FF) >> 4 | (rcChannels[13] & 0x07FF) << 7;
  frame[21] = (uint8_t) ((rcChannels[13])      & 0x07FF) >> 1;
  frame[22] = (uint8_t) ((rcChannels[13])      & 0x07FF) >> 9 | (rcChannels[14] & 0x07FF) << 2;
  frame[23] = (uint8_t) ((rcChannels[14])      & 0x07FF) >> 6 | (rcChannels[15] & 0x07FF) << 5;
  frame[24] = (uint8_t) ((rcChannels[15])      & 0x07FF) >> 3;
  
  // Calculate CRC for frame (excluding address byte)
  frame[25] = crc8(&frame[2], 23);
  
  // Send to flight controller
  CRSF_FC.write(frame, 26);
  packetsSent++;
}

void setup() {
  Serial.begin(115200);
  delay(2000);  // Give time to open Serial Monitor
  
  Serial.println("\n\n========================================");
  Serial.println("CRSF Test Transmitter - DEBUG MODE");
  Serial.println("========================================");
  
  // Initialize LED FIRST to check if it works
  Serial.println("Initializing LED...");
  led.begin();
  led.clear();
  led.setBrightness(32);
  
  // Test LED with colors
  Serial.println("LED Test: RED");
  led.setPixelColor(0, led.Color(255, 0, 0));
  led.show();
  delay(500);
  
  Serial.println("LED Test: GREEN");
  led.setPixelColor(0, led.Color(0, 255, 0));
  led.show();
  delay(500);
  
  Serial.println("LED Test: BLUE");
  led.setPixelColor(0, led.Color(0, 0, 255));
  led.show();
  delay(500);
  
  Serial.println("LED working OK!\n");
  
  // Initialize UART for flight controller
  Serial.printf("Initializing UART1 - TX on GPIO %d (connects to FC A10/UART1 RX)\n", FC_TX_PIN);
  Serial.printf("Baud rate: %d\n", BAUD_RATE);
  
  // HardwareSerial.begin(baud, config, RX_pin, TX_pin)
  CRSF_FC.begin(BAUD_RATE, SERIAL_8N1, FC_RX_PIN, FC_TX_PIN);
  
  // Print a test frame
  Serial.println("\nTest CRSF frame (first packet):");
  uint8_t testFrame[26];
  testFrame[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
  testFrame[1] = 24;
  testFrame[2] = CRSF_FRAMETYPE_RC_CHANNELS;
  // Pack neutral values
  testFrame[3]  = (uint8_t) ((992) & 0x07FF);
  testFrame[4]  = (uint8_t) ((992) & 0x07FF) >> 8 | (992 & 0x07FF) << 3;
  testFrame[5]  = (uint8_t) ((992) & 0x07FF) >> 5 | (172 & 0x07FF) << 6;
  testFrame[6]  = (uint8_t) ((172) & 0x07FF) >> 2;
  testFrame[7]  = (uint8_t) ((172) & 0x07FF) >> 10 | (992 & 0x07FF) << 1;
  testFrame[8]  = (uint8_t) ((992) & 0x07FF) >> 7 | (172 & 0x07FF) << 4;
  testFrame[9]  = (uint8_t) ((172) & 0x07FF) >> 4 | (992 & 0x07FF) << 7;
  testFrame[10] = (uint8_t) ((992) & 0x07FF) >> 1;
  testFrame[11] = (uint8_t) ((992) & 0x07FF) >> 9 | (992 & 0x07FF) << 2;
  testFrame[12] = (uint8_t) ((992) & 0x07FF) >> 6 | (992 & 0x07FF) << 5;
  testFrame[13] = (uint8_t) ((992) & 0x07FF) >> 3;
  testFrame[14] = (uint8_t) ((992) & 0x07FF);
  testFrame[15] = (uint8_t) ((992) & 0x07FF) >> 8 | (992 & 0x07FF) << 3;
  testFrame[16] = (uint8_t) ((992) & 0x07FF) >> 5 | (992 & 0x07FF) << 6;
  testFrame[17] = (uint8_t) ((992) & 0x07FF) >> 2;
  testFrame[18] = (uint8_t) ((992) & 0x07FF) >> 10 | (992 & 0x07FF) << 1;
  testFrame[19] = (uint8_t) ((992) & 0x07FF) >> 7 | (992 & 0x07FF) << 4;
  testFrame[20] = (uint8_t) ((992) & 0x07FF) >> 4 | (992 & 0x07FF) << 7;
  testFrame[21] = (uint8_t) ((992) & 0x07FF) >> 1;
  testFrame[22] = (uint8_t) ((992) & 0x07FF) >> 9 | (992 & 0x07FF) << 2;
  testFrame[23] = (uint8_t) ((992) & 0x07FF) >> 6 | (992 & 0x07FF) << 5;
  testFrame[24] = (uint8_t) ((992) & 0x07FF) >> 3;
  testFrame[25] = crc8(&testFrame[2], 23);
  
  Serial.print("Frame bytes (hex): ");
  for (int i = 0; i < 26; i++) {
    Serial.printf("%02X ", testFrame[i]);
  }
  Serial.println();
  
  Serial.println("\n=== YOUR SETUP - UART1 ===");
  Serial.println("WIRING:");
  Serial.println("   ESP32 GPIO 43 (TX) → FC Pin A10 (UART1 RX) ✓");
  Serial.println("   ESP32 GPIO 44 (RX) → Not connected (we only TX)");
  Serial.println("   ESP32 GND → FC GND (CRITICAL!)");
  Serial.println("\nBETAFLIGHT CONFIGURATION:");
  Serial.println("Paste these commands in CLI tab:");
  Serial.println("   serial 0 64 115200 57600 0 115200");
  Serial.println("   set serialrx_provider = CRSF");
  Serial.println("   set serialrx_inverted = OFF");
  Serial.println("   set serialrx_halfduplex = OFF");
  Serial.println("   save");
  Serial.println("\nOR use GUI:");
  Serial.println("   - Ports tab: UART1 → Enable 'Serial RX'");
  Serial.println("   - Configuration tab: Receiver = SERIAL, Provider = CRSF");
  Serial.println("   - Save and Reboot");
  Serial.println("\nVERIFY:");
  Serial.println("   - Receiver tab should show GREEN BARS moving");
  Serial.println("   - If still 'NO SIGNAL', check:");
  Serial.println("     * Is UART1 already used for something else?");
  Serial.println("     * Type 'serial' in CLI to see current config");
  Serial.println("     * GND connection secure?");
  Serial.println("\n=== SAFETY ===");
  Serial.println("Throttle: MINIMUM (172 = 1000us)");
  Serial.println("Arm: OFF for first 4 phases, then HIGH in phase 4");
  Serial.println("Motors will NOT spin unless armed AND throttle raised");
  Serial.println("========================================\n");
  
  testStartTime = millis();
  updateTestCommands();
}

void loop() {
  static uint32_t lastSendTime = 0;
  static uint32_t lastTestUpdate = 0;
  static uint32_t lastStatusPrint = 0;
  
  // Update test commands every 3 seconds
  if (millis() - lastTestUpdate >= 3000) {
    lastTestUpdate = millis();
    updateTestCommands();
  }
  
  // Send RC channels at ~50Hz (every 20ms) - typical CRSF rate
  if (millis() - lastSendTime >= 20) {
    lastSendTime = millis();
    sendRCChannelsToFC();
    
    // Blink LED to show activity
    static bool ledState = false;
    ledState = !ledState;
    led.setPixelColor(0, ledState ? led.Color(0, 255, 0) : led.Color(0, 64, 0));
    led.show();
  }
  
  // Print status every second
  if (millis() - lastStatusPrint >= 1000) {
    lastStatusPrint = millis();
    Serial.printf("Running... Packets sent: %lu (at 50Hz)\n", packetsSent);
  }
}