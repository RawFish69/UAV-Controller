# Hardware Integration Guide

## System Overview

Three ESP32s in the complete system:

1. **IMU_TX** - Hand-mounted IMU transmitter (IMU_TX firmware)
2. **Command TX** - Relays computer commands to drone (TX_RX/TX + UDP mod)
3. **Drone RX** - On drone, outputs CRSF (TX_RX/RX, unchanged)

Data flow:
```
IMU_TX (hand) → Computer (ROS) → Command TX → Drone RX → Flight Controller
```

## Part 1: IMU_TX Hardware (Hand-Mounted)

### Components

- ESP32-C3 Dev Board
- MPU6050 IMU (I2C)
- Button (optional)
- Potentiometer for throttle (optional)
- LiPo battery or power bank
- Wrist strap or glove mount

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
GPIO 10 →   LED + resistor
```

### Flash Firmware

```bash
cd HAND_IMU_TX

# Edit src/main.cpp WiFi settings first:
# const char* WIFI_SSID = "UAV_CONTROL";
# const char* WIFI_PASSWORD = "uav12345";

# Build and upload
pio run -t upload

# Monitor
pio device monitor
```

### Calibrate IMU

Wave hand in figure-8 pattern until LED stops blinking (10-30 seconds).

### Test

```bash
# Should see in serial monitor:
# - IMU calibration status
# - Quaternion values changing with hand movement
# - Packets sent count increasing
```

## Part 2: Command TX Modification

The TX in TX_RX project needs modification to accept commands from ROS.

### Add UDP Server to TX Firmware

Edit `TX_RX/src/main.cpp` in the `#ifdef BUILD_TX` section:

```cpp
// At top with includes
#include <WiFi.h>
#include <WiFiUdp.h>

// With other TX config
WiFiUDP udp;
const int UDP_PORT = 9000;

// In setup():
void setup() {
  // ... existing setup ...
  
  // Setup WiFi AP
  WiFi.softAP("UAV_TX", "uav12345");
  Serial.printf("TX IP: %s\n", WiFi.softAPIP().toString().c_str());
  
  udp.begin(UDP_PORT);
  Serial.printf("UDP listening on port %d\n", UDP_PORT);
}

// In loop(), add before bench test code:
void loop() {
  unsigned long now = millis();
  CustomProtocol_Update();
  
  // Check for ROS commands via UDP
  int packetSize = udp.parsePacket();
  if (packetSize == 20) {  // DirectCommandPayload
    uint8_t buffer[20];
    udp.read(buffer, 20);
    
    float* floats = (float*)buffer;
    CustomProtocol_SendDirectCommand(
      floats[0],  // roll
      floats[1],  // pitch
      floats[2],  // yaw
      floats[3]   // throttle
    );
    
    lastRcSendMs = now;  // Mark as sent
  }
  
  // ... rest of existing code ...
}
```

### Flash Modified TX

```bash
cd TX_RX
pio run -e transmitter -t upload
```

## Part 3: ROS Computer Setup

### WiFi Hotspot

Create hotspot for hand IMU_TX to connect:

```bash
nmcli dev wifi hotspot ssid UAV_CONTROL password uav12345
```

### Build ROS Packages

```bash
cd UAV-Controller/ros2_ws
colcon build
source install/setup.bash
```

### Launch for Simulation

```bash
./scripts/run_imu_manual_sim.sh
```

This starts:
- IMU protocol receiver (waits for IMU_TX packets)
- IMU controller (gesture processing)
- PID controller (attitude tracking)
- Safety gate
- Simulator + RViz

### Launch for Real Hardware

```bash
./scripts/run_imu_manual_hw.sh transport:=udp udp_host:=192.168.4.1
```

Make sure:
- Command TX is running and reachable at 192.168.4.1
- Drone RX is powered on
- Flight controller is armed correctly

## Part 4: Integration Testing

### Test 1: IMU_TX to Computer

```bash
# Power on IMU_TX
# Wait for calibration

# On computer
source ros2_ws/install/setup.bash
ros2 run manual_imu_controller imu_protocol_receiver

# Check data arrives
ros2 topic echo /imu/data
ros2 topic echo /manual/throttle
```

### Test 2: Computer to Command TX

```bash
# Make sure TX is running with UDP mod

# On computer
source ros2_ws/install/setup.bash
ros2 run adapters_crsf crsf_adapter_node --ros-args \
  -p transport:=udp -p udp_host:=192.168.4.1 -p udp_port:=9000

# In another terminal, send test command
ros2 topic pub /cmd/final/rc common_msgs/msg/VirtualRC \
  '{roll: 0.0, pitch: 0.0, yaw: 0.0, throttle: 0.3}' -r 50

# Check TX serial monitor for received packets
```

### Test 3: Full Chain (Props OFF!)

```bash
# Power on all ESP32s:
# - IMU_TX on hand
# - Command TX
# - Drone RX

# On computer
./scripts/run_imu_manual_hw.sh transport:=udp udp_host:=192.168.4.1

# Move hand, check FC receives commands (no props!)
# Monitor all serial outputs
```

## Packet Formats

### IMU_TX → Computer (ImuDataPayload)

```c
struct ImuDataPayload {
  float qw, qx, qy, qz;  // Quaternion
  float throttle;        // 0.0 - 1.0
  uint32_t timestamp;
  uint8_t flags;         // Bit 0: calibrated
};
```

25 bytes, little-endian `<fffffIB`

### Computer → Command TX (DirectCommandPayload)

```c
struct DirectCommandPayload {
  float roll;       // -1.0 to 1.0
  float pitch;      // -1.0 to 1.0
  float yaw;        // -1.0 to 1.0
  float throttle;   // 0.0 to 1.0
  uint32_t timestamp;
};
```

20 bytes, little-endian `<ffffI`

### Command TX → Drone RX (Custom Protocol)

Uses existing CustomProtocol_SendDirectCommand() - no changes needed.

### Drone RX → FC (CRSF)

Standard CRSF RC channels - no changes needed.

## Troubleshooting

**IMU_TX not connecting**:
- Check computer hotspot "UAV_CONTROL" is active
- Verify WiFi credentials in IMU_TX firmware
- Check ESP32 serial output for connection status

**No IMU data in ROS**:
```bash
ros2 topic hz /imu/data  # Should be ~50 Hz
ros2 topic echo /imu/data  # Check values
```

**Command TX not receiving from ROS**:
- Verify TX IP: check serial output
- Ping test: `ping 192.168.4.1`
- Check ROS adapter: `ros2 topic hz /cmd/final/rc`

**Drone not responding**:
- Check all ESP32 serial outputs for errors
- Verify RX → FC CRSF connection (GPIO 21)
- Check FC configured for CRSF input
- Verify arming sequence

**Wrong axis mapping**:
Edit `imu_controller_params.yaml`:
```yaml
roll_scale: -1.0  # Reverse if needed
pitch_scale: -1.0
```

## Safety Checklist

Before every session:
- [ ] IMU_TX battery charged
- [ ] IMU calibrated (solid LED)
- [ ] All ESP32s powered and connected
- [ ] Computer hotspot active
- [ ] Props OFF for initial test
- [ ] Manual RC override ready
- [ ] Flight area clear

---

**The whole setup in one place. No jumping between docs.**

