# Custom TX/RX System - Complete Documentation

**A modular ESP32-based transmitter/receiver system for RC control with hybrid IMU + Joystick input.**

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Hardware Requirements](#hardware-requirements)
3. [Complete Pinout Guide](#complete-pinout-guide)
4. [Software Architecture](#software-architecture)
5. [Initial Setup](#initial-setup)
6. [Configuration](#configuration)
7. [Control Mapping](#control-mapping)
8. [Calibration](#calibration)
9. [Tuning & Optimization](#tuning--optimization)
10. [Troubleshooting](#troubleshooting)
11. [Advanced Features](#advanced-features)
12. [API Reference](#api-reference)

---

## System Overview

### Features

- **Hybrid Control System**: IMU (tilt) for roll/pitch + Joystick for throttle/yaw
- **Custom ESP-NOW Protocol**: Low-latency wireless communication
- **Modular Architecture**: Clean, reusable code modules
- **CRSF Bridge**: Direct output to flight controllers
- **Multiple Modes**: IMU+Joystick, Bench Test, Manual

### Control Scheme (FPV Drone Style)

| Input Device | Controls | Range | Description |
|--------------|----------|-------|-------------|
| **IMU Tilt Left/Right** | Roll | -100% to +100% | Tilt angle (holds position) |
| **IMU Tilt Forward/Back** | Pitch | -100% to +100% | Tilt angle (holds position) |
| **Joystick Up/Down** | Throttle | 0% to 100% | Center/rest = 0%, Push up = 100% |
| **Joystick Left/Right** | Yaw | -100% to +100% | Left = -100%, Right = +100% |

---

## Hardware Requirements

### Transmitter (TX)

| Component | Model | Quantity | Notes |
|-----------|-------|----------|-------|
| **Microcontroller** | ESP32 (any variant) | 1 | Tested on M5 Stamp S3 |
| **IMU** | MPU6050 or BNO085 | 1 | I2C IMU (BNO085 recommended for better accuracy) |
| **Joystick** | 2-axis analog joystick | 1 | Output: 0-3.3V |
| **Power Supply** | 3.3V regulator or battery | 1 | Sufficient for ESP32 + sensors |

### Receiver (RX)

| Component | Model | Quantity | Notes |
|-----------|-------|----------|-------|
| **Microcontroller** | ESP32 (any variant) | 1 | Tested on M5 Stamp S3 |
| **UART Connection** | To flight controller | 1 | For CRSF output |
| **Power Supply** | 3.3V from FC or external | 1 | Powered via FC or BEC |

### Optional Components

- **Switches**: For AUX channels (arming, modes)
- **Battery Monitor**: For voltage/current telemetry
- **OLED Display**: For status display
- **Buttons**: For menu navigation

---

## Complete Pinout Guide

### Transmitter (TX) Pinout

#### ESP32 Standard Configuration

```
ESP32 Pin      | Connection                | Function              | Notes
---------------|---------------------------|-----------------------|------------------------
GPIO 4         | IMU SDA (MPU6050/BNO085)  | I2C Data              | Pull-up not required
GPIO 5         | IMU SCL (MPU6050/BNO085)  | I2C Clock             | Pull-up not required
GPIO 0 (ADC1)  | Joystick Throttle (VRY)   | Analog Input          | 0-3.3V range
GPIO 1 (ADC1)  | Joystick Yaw (VRX)        | Analog Input          | 0-3.3V range
GPIO 6         | Joystick Button (SW)      | Digital Input         | Controls AUX1 (arming)
3.3V           | Power (IMU + Stick)       | Power Supply          | Common 3.3V rail
GND            | Ground                    | Common Ground         | Connect all grounds
```

#### M5 Stamp S3 Specific

```
M5 Stamp S3    | Connection                | Function              | Notes
---------------|---------------------------|-----------------------|------------------------
GPIO 4         | IMU SDA (MPU6050/BNO085)  | I2C Data              | Grove connector
GPIO 5         | IMU SCL (MPU6050/BNO085)  | I2C Clock             | Grove connector
GPIO 0         | Joystick Throttle         | Analog Input          | ADC1 pin
GPIO 1         | Joystick Yaw              | Analog Input          | ADC1 pin
GPIO 6         | Joystick Button (SW)      | Digital Input         | Controls AUX1 (arming)
3V3            | Power                     | Power Supply          | 500mA max
GND            | Ground                    | Common Ground         | Multiple GND pins
```

#### Optional TX Pins

```
ESP32 Pin      | Connection           | Function              | Configuration Required
---------------|----------------------|-----------------------|------------------------
GPIO 3         | AUX Switch 2         | Digital Input         | Add in main.cpp
GPIO 7         | Mode Button          | Digital Input         | Add in main.cpp
GPIO 8         | LED Indicator        | Digital Output        | Add in main.cpp
GPIO 9         | Battery Voltage      | Analog Input (ADC)    | Add voltage divider
GPIO 10        | Additional Switch    | Digital Input         | Add in main.cpp
```

**Note:** GPIO 0, 1, 4, 5, 6 are already used for joystick and IMU.

### Receiver (RX) Pinout

#### ESP32 Standard Configuration

```
ESP32 Pin      | Connection           | Function              | Notes
---------------|----------------------|-----------------------|------------------------
GPIO 21        | FC UART TX           | CRSF Output           | RX-side transmit
GPIO 20        | FC UART RX           | CRSF Input (optional) | RX-side receive
GPIO 8         | Status LED           | Link Status Indicator | Green LED recommended
3.3V           | Power from FC        | Power Supply          | Or external BEC
GND            | Ground to FC         | Common Ground         | Critical connection
```

#### M5 Stamp S3 Specific

```
M5 Stamp S3    | Connection           | Function              | Notes
---------------|----------------------|-----------------------|------------------------
GPIO 21        | FC UART TX           | CRSF Output           | Connect to FC RX
GPIO 20        | FC UART RX           | CRSF Input (optional) | Connect to FC TX
GPIO 8         | Status LED           | Link Status Indicator | Green LED + 220Ω resistor
5V             | Power (if available) | Power Input           | With onboard regulator
GND            | Ground to FC         | Common Ground         | Connect all grounds
```

**IMPORTANT - CRSF Wiring to Flight Controller:**

```
ESP32 RX (GPIO 21) → Flight Controller RX Pin (CRSF/Serial RX)
ESP32 GND          → Flight Controller GND
ESP32 3.3V/5V      → Flight Controller 3.3V/5V (or external power)
```

⚠️ **Do NOT connect GPIO 43/44** - these are USB pins on many ESP32 boards!

---

## Software Architecture

### Project Structure

```
CustomTX/
├── src/
│   ├── main.cpp                 # Main TX/RX logic, protocol handling
│   ├── protocol.h/cpp           # Custom protocol implementation
│   ├── espnow_TX.h/cpp          # ESP-NOW transmitter
│   ├── espnow_RX.h/cpp          # ESP-NOW receiver
│   ├── imu_handler.h/cpp        # IMU module (roll/pitch)
│   ├── joystick_handler.h/cpp   # Joystick module (throttle/yaw)
│   └── crsf_bridge.h/cpp        # CRSF protocol bridge (RX only)
├── platformio.ini               # Build configuration
└── README.md                    # This file
```

### Module Responsibilities

| Module | Purpose | Used By |
|--------|---------|---------|
| **main.cpp** | System initialization, main loop, mode selection | TX & RX |
| **protocol** | Packet framing, CRC, link management | TX & RX |
| **espnow_TX** | ESP-NOW wireless transmission | TX only |
| **espnow_RX** | ESP-NOW wireless reception | RX only |
| **imu_handler** | MPU6050 reading, calibration, roll/pitch output | TX only* |
| **joystick_handler** | Analog stick reading, filtering, throttle/yaw output | TX only* |
| **crsf_bridge** | CRSF protocol output to flight controller | RX only |

**Note:** *Files marked "TX only" use `#ifdef BUILD_TX` guards and won't compile for RX builds. This prevents dependency issues (e.g., RX doesn't need IMU libraries).

---

## Initial Setup

### 1. Hardware Assembly

#### Transmitter (TX)

1. **Connect IMU (MPU6050 or BNO085):**
   - VCC → 3.3V
   - GND → GND
   - SDA → GPIO 4
   - SCL → GPIO 5
   - **Note**: BNO085 recommended for better accuracy with built-in sensor fusion

2. **Connect Joystick:**
   - VCC → 3.3V
   - GND → GND
   - VRx (Throttle) → GPIO 0
   - VRy (Yaw) → GPIO 1
   - SW (Button) → GPIO 6 (controls AUX1 arming)

3. **Power Supply:**
   - USB for development
   - LiPo battery + regulator for field use

#### Receiver (RX)

1. **Connect to Flight Controller:**
   - GPIO 21 → FC RX pin (CRSF/Serial RX)
   - GPIO 20 → FC TX pin (optional, for telemetry)
   - GND → FC GND
   - 3.3V/5V → FC power or external BEC

2. **Connect Status LED (Optional but Recommended):**
   - GPIO 8 → LED anode (longer leg) via 220Ω resistor
   - LED cathode (shorter leg) → GND
   - **Recommended:** Use a green LED for clear status indication
   - **LED behavior:**
     - 🟢 Solid ON = Link active, receiving signal ✓
     - 🟡 Blinking (500ms) = No signal / failsafe ⚠️
     - ⚫ Off = System not ready

3. **Configure Flight Controller:**
   - Enable CRSF protocol on the UART
   - Set baud rate to 420000 (handled automatically)
   - Assign channel mapping in FC firmware

### 2. Software Installation

#### Install PlatformIO

```bash
# VSCode Extension (recommended)
1. Install VSCode
2. Install PlatformIO IDE extension
3. Open project folder

# Or CLI
pip install platformio
```

#### Build and Upload

**For Transmitter:**
```bash
cd CustomTX
pio run -e tx --target upload
```

**For Receiver:**
```bash
cd CustomTX
pio run -e rx --target upload
```

**Important:** The transmitter and receiver use the same source code but compile differently:
- **TX build** includes IMU and joystick code (requires Adafruit libraries)
- **RX build** excludes IMU/joystick code (no extra dependencies)
- Build flags `BUILD_TX` and `BUILD_RX` control conditional compilation

#### Monitor Serial Output

```bash
# TX Monitor
pio device monitor -e tx

# RX Monitor
pio device monitor -e rx
```

---

## Configuration

### Transmitter Configuration (`main.cpp`)

#### Control Mode Selection

```cpp
// Choose control mode (set only ONE to true)
#define USE_IMU_JOYSTICK_INPUT true  // Hybrid IMU + Joystick (recommended)
#define BENCH_TEST_MODE false        // Auto-cycling test mode
```

#### IMU Type Selection

```cpp
// IMU Configuration - Choose your IMU type
#define IMU_TYPE IMU_BNO085          // Options: IMU_MPU6050 or IMU_BNO085
#define IMU_SDA_PIN 4                // I2C SDA pin
#define IMU_SCL_PIN 5                // I2C SCL pin
```

**IMU Comparison:**

| Feature | MPU6050 | BNO085 |
|---------|---------|--------|
| **Axes** | 6-axis (gyro + accel) | 9-axis (gyro + accel + mag) |
| **Sensor Fusion** | Manual calculation | Built-in (superior!) |
| **Accuracy** | Good | Excellent |
| **Drift** | Moderate | Minimal |
| **Price** | Lower | Higher |
| **Recommended** | Budget builds | Best performance |

**Which to choose:**
- **MPU6050**: Good for basic setups, lower cost
- **BNO085**: Best choice for accurate control, built-in sensor fusion eliminates drift

#### Pin Configuration

```cpp
// Joystick Configuration
#define JOYSTICK_THROTTLE_PIN 0      // Analog pin for throttle (up/down)
#define JOYSTICK_YAW_PIN 1           // Analog pin for yaw (left/right)
#define JOYSTICK_BUTTON_PIN 6        // Digital pin for button (SW pin, controls AUX1)
```

#### Update Rate Configuration

```cpp
#define RC_SEND_FREQUENCY_HZ 100     // Options: 50, 100, 150, 250, 500 Hz
```

**Recommended frequencies:**
- **50 Hz**: Maximum range, lowest latency priority
- **100 Hz**: Balanced (default, recommended)
- **150 Hz**: High update rate
- **250/500 Hz**: Racing, minimum latency

#### Sensitivity Configuration

```cpp
#define ROLL_SENSITIVITY 45.0        // Degrees of tilt for full stick deflection
#define PITCH_SENSITIVITY 45.0       // Degrees of tilt for full stick deflection
```

**Sensitivity Guide:**
- **High Sensitivity (30-35°)**: Aggressive, requires less tilt, racing
- **Medium Sensitivity (40-50°)**: Balanced, general flying (default: 45°)
- **Low Sensitivity (60-90°)**: Gentle, requires more tilt, beginner-friendly

### Receiver Configuration (`main.cpp`)

#### CRSF Output Configuration

```cpp
#define ENABLE_CRSF_OUTPUT true      // true = production, false = debug mode
#define CRSF_OUTPUT_FREQUENCY_HZ 100 // Must match TX frequency

// Pin definitions
#define CRSF_TX_PIN 21               // TX pin to FC RX
#define CRSF_RX_PIN 20               // RX pin from FC TX (optional)
#define LED_PIN 8                    // Status LED (green recommended)
#define LED_BLINK_RATE 500           // Blink rate in ms when no signal
```

#### Debug Mode

Set `ENABLE_CRSF_OUTPUT false` for USB debugging:
- Prints all received RC values to Serial
- Useful for testing TX without FC connected
- Re-enable for flight

---

## Control Mapping

### Channel Assignment (CRSF Format)

| Channel | Function | Source | Range | Center | Notes |
|---------|----------|--------|-------|--------|-------|
| **CH1** | Roll | IMU Tilt | 172-1811 | 992 | Tilt angle (holds position) |
| **CH2** | Pitch | IMU Tilt | 172-1811 | 992 | Tilt angle (holds position) |
| **CH3** | Throttle | Joystick | 172-1811 | N/A | 0-100% (no center) |
| **CH4** | Yaw | Joystick | 172-1811 | 992 | Centered axis |
| **CH5** | AUX1 | Joystick Button | 172-1811 | 172/1811 | Toggle: press to arm/disarm |
| **CH6** | AUX2 | (Reserved) | 172-1811 | 992 | Mode switch |
| **CH7-16** | AUX3-12 | (Available) | 172-1811 | 992 | Future use |

### Value Ranges

- **CRSF Min**: 172 (0%)
- **CRSF Center**: 992 (50%)
- **CRSF Max**: 1811 (100%)
- **CRSF Range**: ±819 from center

### Normalized Values

All control modules output normalized values:
- **Centered axes** (Roll, Pitch, Yaw): `-1.0` to `+1.0` (0.0 = center)
- **Throttle axis**: `0.0` to `1.0` (0.0 = down/rest position, 1.0 = full up)

**Note:** Throttle only uses the upper half of joystick travel (like a real drone controller):
- Joystick at center/rest = Minimum throttle (0%)
- Push joystick up = Increases from 0% to 100%
- Push joystick down = Stays at 0% (ignored)

---

## Calibration

### IMU Calibration

#### BNO085 (No Calibration Needed!)

The BNO085 has built-in sensor fusion and automatic calibration:

1. **Power on** the transmitter
2. System is ready immediately!
3. No manual calibration required

**Advantages:**
- No drift over time
- Automatic temperature compensation
- Superior accuracy out of the box

#### MPU6050 (Automatic Startup Calibration)

The MPU6050 calibrates automatically on startup:

1. **Power on** the transmitter
2. **Keep it still** on a level surface for ~2 seconds
3. Wait for **"Calibration complete!"** message
4. System is ready to use

**What it calibrates:**
- Gyro zero-rate offsets (X, Y, Z)
- Accelerometer offsets
- Gravity compensation on Z-axis

**If calibration fails:**
- Ensure IMU is completely still
- Check I2C wiring
- Restart the device

### Joystick Calibration (Manual, Optional)

If joystick center is off:

```cpp
// Add to setup() after joystick.begin()
joystick.calibrateCenter();  // Center both sticks first!
```

**Calibration procedure:**
1. Center both joystick axes
2. Power on (or call `calibrateCenter()`)
3. Wait 2 seconds
4. Test by moving sticks

**Fine-tuning in code:**

```cpp
// Adjust deadband to compensate for drift
joystick.setYawDeadband(0.08);        // Increase if yaw drifts
joystick.setThrottleDeadband(0.03);   // Increase if throttle jitters
```

---

## Tuning & Optimization

### IMU Sensitivity Tuning

Adjust how much tilt angle produces full stick deflection:

```cpp
// In main.cpp setup()
imu.setRollSensitivity(45.0);   // Lower = more sensitive (less tilt needed)
imu.setPitchSensitivity(45.0);  // Higher = less sensitive (more tilt needed)
```

**Finding your optimal sensitivity:**

1. Start with default (45° tilt)
2. Fly and note if controls feel too sensitive or too slow
3. Adjust in increments of 5-10°
4. Test again

| Feel | Adjustment |
|------|------------|
| Too sensitive, hard to control | Increase by 10-15° (e.g., 55-60°) |
| Too slow, not responsive | Decrease by 10-15° (e.g., 30-35°) |
| Perfect | No change needed |

### Joystick Smoothing

Enable/adjust smoothing for stable control:

```cpp
// In main.cpp setup()
joystick.setSmoothing(true, 0.3);  // true/false, alpha 0.0-1.0
```

**Alpha parameter:**
- **0.1**: Heavy smoothing (slow response, very stable)
- **0.3**: Moderate smoothing (default, balanced)
- **0.5**: Light smoothing (fast response, some filtering)
- **1.0**: No smoothing (instant response, may jitter)

### Joystick Deadband

Prevent drift near center and jitter at endpoints:

```cpp
joystick.setYawDeadband(0.05);      // 5% around center (default)
joystick.setThrottleDeadband(0.02); // 2% at endpoints (default)
```

**When to adjust:**
- **Yaw drifts** when stick is centered → Increase yaw deadband (0.08-0.10)
- **Throttle jitters** at min/max → Increase throttle deadband (0.03-0.05)
- **Controls feel sluggish** → Decrease deadband (0.02-0.03)

### Update Rate Optimization

Balance latency vs. range:

```cpp
#define RC_SEND_FREQUENCY_HZ 100  // Both TX and RX must match!
```

| Frequency | Latency | Range | Use Case |
|-----------|---------|-------|----------|
| 50 Hz | 20ms | Maximum | Long range, cruising |
| 100 Hz | 10ms | Good | General flying (default) |
| 150 Hz | 6.7ms | Good | Sport flying |
| 250 Hz | 4ms | Reduced | Racing |
| 500 Hz | 2ms | Minimum | Ultra-low latency racing |

---

## Troubleshooting

### IMU Issues

#### "Failed to find IMU chip!" (MPU6050 or BNO085)

**Causes & Solutions:**

1. **Wiring issue:**
   - Check all 4 connections (VCC, GND, SDA, SCL)
   - Ensure 3.3V power (NOT 5V for most modules)
   - Verify SDA → GPIO 4, SCL → GPIO 5

2. **Wrong I2C address:**
   - MPU6050: 0x68 or 0x69
   - BNO085: 0x4A or 0x4B (default 0x4A)
   - Use I2C scanner to verify

3. **Swapped SDA/SCL:**
   - Try swapping pins 4 and 5 in configuration

4. **Wrong IMU type selected:**
   - Verify `IMU_TYPE` matches your physical IMU
   - Check Serial Monitor for which IMU is being initialized

5. **Defective module:**
   - Test with Arduino and example sketch
   - Check for visible damage or cold solder joints

#### IMU Drift or Incorrect Readings

**For MPU6050:**

1. **Recalibrate:** Keep still during startup
2. **Check mounting:** IMU should be firmly attached, not loose
3. **Electromagnetic interference:** Keep away from motors/ESCs
4. **Temperature:** Let IMU warm up for 30 seconds
5. **Consider upgrading:** BNO085 eliminates most drift issues

**For BNO085:**

1. **Check mounting:** Should be firmly attached
2. **Let it settle:** First few seconds may show slight drift as fusion initializes
3. If persistent issues, check for hardware defect

#### Controls Inverted

**Fix in code (main.cpp, in loop()):**

```cpp
// To invert roll:
float rollNormalized = -imu.getRoll();  // Add negative sign

// To invert pitch:
float pitchNormalized = -imu.getPitch();  // Add negative sign
```

### Joystick Issues

#### Joystick Drift (Non-Zero When Centered)

**Solutions:**

1. **Increase deadband:**
   ```cpp
   joystick.setYawDeadband(0.10);  // Increase from 0.05 to 0.10
   ```

2. **Calibrate center:**
   ```cpp
   joystick.calibrateCenter();  // In setup(), sticks centered
   ```

3. **Check power supply:** Noisy power causes drift
   - Add capacitor (10µF) across VCC/GND at joystick

4. **Check wiring:** Loose connections cause intermittent drift

#### Joystick Not Responding

**Checks:**

1. **Verify ADC pins:**
   - Use ADC1 pins (GPIO 1-10) if WiFi is active
   - Check pinout for your specific ESP32 board

2. **Test with multimeter:**
   - Measure voltage at joystick output (should vary 0-3.3V)

3. **Print raw values:**
   ```cpp
   uint16_t throttleRaw, yawRaw;
   joystick.getRawValues(throttleRaw, yawRaw);
   Serial.printf("Raw: T=%d Y=%d\n", throttleRaw, yawRaw);
   ```
   Should read ~0 to ~4095 as stick moves

#### Joystick Too Sensitive/Not Sensitive

This is normal behavior - joysticks have linear response. Use smoothing:

```cpp
joystick.setSmoothing(true, 0.2);  // Heavy smoothing
```

Or apply exponential curve in main.cpp:

```cpp
float yaw = joystick.getYaw();
yaw = yaw * yaw * yaw;  // Cubic curve (more precision near center)
```

### Communication Issues

#### No Link / "Link: DOWN"

**TX Side:**

1. Check ESP-NOW initialization in Serial Monitor
2. Verify both TX and RX are on same WiFi channel
3. Ensure broadcast address is `FF:FF:FF:FF:FF:FF`

**RX Side:**

1. Check Serial Monitor for "Protocol initialized"
2. Verify RX is powered and running
3. Check distance (start close, <5m for testing)

#### High Packet Loss

**Solutions:**

1. **Reduce frequency:** Lower from 250Hz to 100Hz
2. **Check antenna:** Ensure PCB antenna not blocked
3. **Reduce obstacles:** Metal, walls, water absorb signal
4. **Check power:** Low voltage causes transmission issues

#### CRSF Output Not Working

**Checks:**

1. **Verify connections:**
   - ESP RX GPIO 21 → FC RX pin
   - Common ground connected

2. **FC configuration:**
   - UART enabled for CRSF/Serial RX
   - Baud rate: 420000 (auto-detected usually)
   - Correct UART selected (UART1, UART2, etc.)

3. **Debug mode:**
   ```cpp
   #define ENABLE_CRSF_OUTPUT false  // Test with Serial output first
   ```

4. **Check FC firmware:** Ensure CRSF protocol supported

---

## Advanced Features

### Spring-Loaded Throttle Mode

If throttle stick returns to center (like yaw):

```cpp
// In main.cpp setup()
joystick.setThrottleSpringLoaded(true);
```

Now throttle outputs -1.0 to +1.0 (centered), like yaw.

### Joystick Button Control (AUX1)

The joystick button is already configured to control AUX1 (arming) with **toggle behavior**:
- **Press button once** → Toggle to ARMED (AUX1 = HIGH / 1811)
- **Press button again** → Toggle to DISARMED (AUX1 = LOW / 172)
- **State is saved** → No need to hold button, it stays in the last state

This is configured on **GPIO 6** by default.

When you press the button, you'll see in Serial Monitor:
```
[ARMING] *** ARMED ***
```
or
```
[ARMING] *** DISARMED ***
```

### Adding Additional AUX Switches

Example: Add mode switch on GPIO 3 for AUX2

```cpp
// In main.cpp
#define MODE_SWITCH_PIN 3

void setup() {
  // ... existing setup ...
  pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);
}

void loop() {
  // ... in RC send block after joystick button code ...
  
  // Read switch (LOW = pressed)
  bool modeSwitch = (digitalRead(MODE_SWITCH_PIN) == LOW);
  rcChannels[5] = modeSwitch ? 1811 : 172;  // AUX2: mode switch
}
```

### Custom Exponential Curves

Apply expo for finer control near center:

```cpp
float applyExpo(float input, float expo) {
  // expo: 0.0 = linear, 0.5 = moderate, 1.0 = max expo
  return input * (1.0 - expo) + input * input * input * expo;
}

// In loop():
float roll = applyExpo(imu.getRoll(), 0.3);    // 30% expo
float yaw = applyExpo(joystick.getYaw(), 0.5); // 50% expo
```

### Dual Rates

Switch between sensitivity profiles:

```cpp
bool highRate = digitalRead(RATE_SWITCH_PIN);

if (highRate) {
  imu.setRollSensitivity(150.0);   // Fast
  imu.setPitchSensitivity(150.0);
} else {
  imu.setRollSensitivity(300.0);   // Slow
  imu.setPitchSensitivity(300.0);
}
```

### Telemetry Display

Access telemetry from RX:

```cpp
// In TX main loop
TelemetryPayload telemetry;
CustomProtocol_GetTelemetry(&telemetry);

Serial.printf("Battery: %.2fV %.2fA\n", telemetry.voltage, telemetry.current);
Serial.printf("RSSI: %ddBm LQ: %d%%\n", telemetry.rssi, telemetry.linkQuality);
```

---

## API Reference

### IMUHandler Class

```cpp
class IMUHandler {
public:
  // Initialization
  bool begin(uint8_t sdaPin, uint8_t sclPin, IMUType type = IMU_MPU6050);
  
  // Core functions
  void update();                    // Call every loop
  float getRoll();                  // Get roll (-1.0 to 1.0)
  float getPitch();                 // Get pitch (-1.0 to 1.0)
  
  // Configuration
  void setRollSensitivity(float sensitivity);   // degrees of tilt for full stick
  void setPitchSensitivity(float sensitivity);  // degrees of tilt for full stick
  
  // Debug
  void getRawGyro(float &x, float &y, float &z);     // degrees/sec (for debugging)
  void getAngles(float &roll, float &pitch);         // degrees (actual control angles)
  float getTiltAngle();                               // degrees
};

// IMU Types
enum IMUType {
  IMU_MPU6050 = 0,  // Basic 6-axis IMU
  IMU_BNO085 = 1    // Advanced 9-axis IMU with sensor fusion
};
```

### JoystickHandler Class

```cpp
class JoystickHandler {
public:
  // Initialization
  void begin(uint8_t throttlePin, uint8_t yawPin);
  
  // Core functions
  void update();                    // Call every loop
  float getThrottle();              // Get throttle (0.0 to 1.0)
  float getYaw();                   // Get yaw (-1.0 to 1.0)
  
  // Configuration
  void setYawDeadband(float deadband);         // 0.0 to 1.0
  void setThrottleDeadband(float deadband);    // 0.0 to 1.0
  void setSmoothing(bool enable, float alpha); // alpha: 0.0 to 1.0
  void setThrottleSpringLoaded(bool springLoaded);
  
  // Calibration
  void calibrateCenter();  // Calibrate at current stick position
  
  // Debug
  void getRawValues(uint16_t &throttle, uint16_t &yaw);  // ADC values
};
```

### Protocol Functions

```cpp
// Initialization
bool CustomProtocol_Init(bool isTx, uint8_t* peerMac);

// TX Functions
bool CustomProtocol_SendRcCommand(uint16_t* channels);
bool CustomProtocol_SendHeartbeat();
void CustomProtocol_GetTelemetry(TelemetryPayload* telemetry);

// RX Functions
bool CustomProtocol_SendTelemetry(float voltage, float current, int16_t rssi, uint8_t lq);
void CustomProtocol_GetRcChannels(uint16_t* channels);

// Common
void CustomProtocol_Update();  // Call every loop
bool CustomProtocol_IsLinkActive();
void CustomProtocol_GetStats(ProtocolStats* stats);
```

---

## Quick Reference Card

### Pin Summary

**TX:** IMU(4,5) + Joystick(0,1,6) + Power(3.3V) + GND  
**RX:** CRSF_TX(21) + CRSF_RX(20) + LED(8) + Power + GND

### Default Settings

- **Update Rate**: 100 Hz
- **IMU Sensitivity**: 45° tilt (roll & pitch)
- **Yaw Deadband**: 5%
- **Throttle Deadband**: 2%
- **Smoothing**: Enabled (α=0.3)

### Channel Mapping

| CH | Function | Source | Range |
|----|----------|--------|-------|
| 1 | Roll | IMU X | ±100% |
| 2 | Pitch | IMU Y | ±100% |
| 3 | Throttle | Stick V | 0-100% |
| 4 | Yaw | Stick H | ±100% |
| 5 | AUX1 | Button | Toggle |

### Common Issues

| Problem | Quick Fix |
|---------|-----------|
| IMU not found | Check wiring, try swapping SDA/SCL |
| Joystick drift | Increase deadband to 0.10 |
| Too sensitive | Increase sensitivity to 60° |
| Not sensitive enough | Decrease sensitivity to 30° |
| No link | Check both devices powered, same frequency |
| CRSF not working | Verify GPIO 21 → FC RX, enable CRSF in FC |

---

## Build System Notes

### Conditional Compilation

The project uses build flags to compile different code for TX and RX:

- **`BUILD_TX`**: Defined when building transmitter (enables IMU, joystick)
- **`BUILD_RX`**: Defined when building receiver (enables CRSF bridge)

Files that are TX-only (like `imu_handler.cpp` and `joystick_handler.cpp`) are wrapped with:

```cpp
#ifdef BUILD_TX
// TX-only code here
#endif
```

This prevents the RX build from requiring IMU libraries it doesn't need.

### Library Dependencies

**Transmitter requires:**
- Adafruit MPU6050 (for MPU6050 support)
- Adafruit BNO08x (for BNO085 support)
- Adafruit Unified Sensor
- Wire (I2C)
- WiFi (ESP-NOW)

**Receiver requires:**
- WiFi (ESP-NOW)

PlatformIO automatically manages these dependencies based on `platformio.ini`.

**Note:** Both IMU libraries are included, allowing you to switch between IMU types by simply changing the `IMU_TYPE` define in `main.cpp`.

---

## Support & Resources

### Documentation

- **This File**: Complete system reference
- **Code Comments**: Detailed inline documentation
- **Serial Output**: Real-time debug information

### Example Serial Output

```
*** IMU + JOYSTICK CONTROL MODE ***
[IMU] MPU6050 found!
[IMU] Calibration complete!
[Joystick] Initialized
[TX] Ready to transmit!

--- TX Statistics ---
Link: ACTIVE | Last RX: 10 ms ago
TX Sending RC: R:992 P:1050 T:856 Y:992 | AUX1:172 AUX2:992
IMU Angles (°): Roll:0.5 Pitch:12.3 | Normalized: R:0.01 P:0.27
Joystick ADC: Throttle:1856 Yaw:2048 | Output: T:0.42 Y:0.00
```

### Useful Commands

```bash
# Build only (check for errors)
pio run -e tx

# Upload to device
pio run -e tx --target upload

# Monitor serial output
pio device monitor

# Clean build
pio run --target clean
```

---

## License & Credits

This project uses:
- **ESP-NOW**: Espressif wireless protocol
- **CRSF Protocol**: TBS Crossfire/ELRS compatible
- **Adafruit MPU6050**: IMU sensor library
- **PlatformIO**: Build system
