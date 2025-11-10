# IMU TX - ESP32 Firmware

## What This Is

ESP32 firmware for hand-mounted IMU sensor that transmits orientation data to the computer via custom wireless protocol. Used for gesture-based drone control.

## Hardware

### Components

1. **ESP32-C3 Dev Board**
2. **MPU6050 6-DOF IMU** (I2C, ~$2-5)
3. **Button** (optional)
4. **Potentiometer** (optional, for throttle)
5. **LED** (status indicator)
6. **Battery** (LiPo 1S or power bank)
7. **Mounting** (wrist strap or glove)

### Wiring

```
ESP32-C3    MPU6050
--------    -------
GPIO 8  →   SDA
GPIO 9  →   SCL
3.3V    →   VCC
GND     →   GND

ESP32-C3    Other
--------    -----
GPIO 3  →   Button (to GND)
GPIO 4  →   Pot wiper (0-3.3V)
GPIO 10 →   LED + 220Ω resistor
```

### Mounting Options

**Option 1: Wrist Strap**
- Velcro or elastic band
- IMU on back of hand
- Battery on wrist

**Option 2: Glove Mount**
- Attach to cycling/work glove
- IMU on back of hand
- Battery in pocket, wired

**Option 3: 3D Printed Case**
- Design snap-fit case for IMU + ESP32
- Wrist mounting straps
- Battery compartment

**IMU Orientation**:
- X-axis along fingers (forward)
- Y-axis across palm (right)
- Z-axis perpendicular to palm (up)

## Software Setup

### Flash Firmware

```bash
cd IMU_TX

# Build and upload
pio run -t upload

# Monitor
pio device monitor
```

### Configuration

Edit `src/main.cpp` before building:

```cpp
// Network - connect to computer's WiFi hotspot
const char* WIFI_SSID = "UAV_CONTROL";  // Computer hotspot name
const char* WIFI_PASSWORD = "uav12345";

// Transmit rate
#define IMU_SEND_FREQUENCY_HZ 50  // 50 Hz is good balance

// Pins (adjust for board)
#define IMU_SDA_PIN 8
#define IMU_SCL_PIN 9
#define BUTTON_PIN 3
#define THROTTLE_INPUT_PIN 4
```

## Usage

### 1. Power On

Connect battery or USB. LED will blink during calibration.

### 2. Calibrate IMU

Keep hand still for 3 seconds after power on. MPU6050 will zero itself. LED turns solid when ready.

### 3. Connect to Computer

The ESP32 connects to the computer's WiFi hotspot and broadcasts IMU data via ESP-NOW.

Computer receives via `imu_protocol_receiver` ROS node.

### 4. Control Drone

Tilt hand to control drone orientation:
- Forward tilt → Drone pitches forward
- Roll hand → Drone rolls
- Rotate hand → Drone yaws
- Throttle from pot/slider

## Protocol Details

### Packet Format

Sends `PKT_IMU_DATA` (type 0x12) packets:

```c
struct ImuDataPayload {
  float qw, qx, qy, qz;  // Quaternion orientation
  float throttle;        // 0.0 to 1.0
  uint32_t timestamp;    // Milliseconds
  uint8_t flags;         // Bit 0: calibrated, Bit 1: button
};
```

Total: 25 bytes payload

### Transmission

- Rate: 50 Hz (configurable)
- Transport: ESP-NOW broadcast
- Target: Computer running ROS 2
- Failsafe: Heartbeat every 500ms

## Computer Setup

The computer needs:

1. **WiFi Hotspot** - Create hotspot named "UAV_CONTROL"
2. **ROS 2 Node** - `imu_protocol_receiver` to receive packets
3. **IMU Controller** - Processes gestures
4. **Rest of stack** - PID, safety, CRSF adapter

### Create WiFi Hotspot (Ubuntu)

```bash
# Using NetworkManager
nmcli dev wifi hotspot ssid UAV_CONTROL password uav12345

# Or use GUI settings → WiFi → Create Hotspot
```

### Launch ROS Stack

```bash
cd UAV-Controller
source ros2_ws/install/setup.bash

# For simulation
./scripts/run_imu_manual_sim.sh

# For hardware (after TX_RX integration)
./scripts/run_imu_manual_hw.sh transport:=udp
```

## Troubleshooting

**LED blinking**
- Wait 3-5 seconds for init
- Check I2C wiring (GPIO 8/9)
- Verify MPU6050 address is 0x68
- Try power cycling

**No data on computer**
- Check WiFi connection
- Verify computer hotspot is "UAV_CONTROL"
- Check ESP32 serial output for IP address
- Verify ROS `imu_protocol_receiver` is running

**Wrong orientation mapping**
- Check IMU mounting orientation
- Adjust in ROS config: `imu_controller_params.yaml`
- Set `roll_scale: -1.0` to reverse axis

**Delayed response**
- Increase transmit rate: `IMU_SEND_FREQUENCY_HZ 100`
- Check WiFi signal strength
- Reduce computer processing load

## Development

### Serial Monitor

```bash
pio device monitor

# Should show:
# - IMU calibration status
# - Quaternion values
# - Throttle reading
# - Packets sent
# - Link status
```

### Testing IMU TX Alone

```bash
# Power on IMU TX
pio device monitor

# Should see:
# - MPU6050 detected
# - Roll/Pitch angles
# - Packets sent
```

### Testing Protocol

```bash
# On computer, listen for packets
# (requires custom UDP listener or ROS node)

source ros2_ws/install/setup.bash
ros2 run manual_imu_controller imu_protocol_receiver

# In another terminal, check if data arrives
ros2 topic echo /imu/data
ros2 topic echo /manual/throttle
```

## Safety Notes

- Secure mounting - don't want IMU falling off mid-flight
- Calibrate before every session
- Check battery level
- Test range - stay within WiFi coverage
- Have manual RC override ready

## System Architecture

Complete data flow:

```
Hand Movement
     ↓
MPU6050 IMU (I2C)
     ↓
ESP32 IMU TX (this firmware)
     ↓ ESP-NOW broadcast
Computer WiFi
     ↓
imu_protocol_receiver (ROS node)
     ↓
imu_controller_node
     ↓
PID Controller → Safety → CRSF → TX_RX → Drone
```

---

**Next Steps:**
1. Wire up IMU to ESP32
2. Flash this firmware
3. Test IMU calibration
4. Connect computer to same WiFi
5. Run ROS stack
6. Move hand, see drone respond in sim
7. Once confident, try on real hardware

