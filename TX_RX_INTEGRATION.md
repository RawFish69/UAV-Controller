# TX_RX Integration Guide

## Overview

Your TX_RX ESP32 firmware uses a custom wireless protocol with two command formats. The ROS 2 system uses the **DirectCommandPayload** format which is cleaner and matches what controllers output.

## Packet Format

The `txrx_packer` sends 20-byte packets matching your `DirectCommandPayload` structure:

```c
struct DirectCommandPayload {
  float roll;       // -1.0 to 1.0
  float pitch;      // -1.0 to 1.0
  float yaw;        // -1.0 to 1.0
  float throttle;   // 0.0 to 1.0
  uint32_t timestamp;
};
```

Total: 4 floats (16 bytes) + 1 uint32 (4 bytes) = **20 bytes**, little-endian format `<ffffI`

## Adding ROS Input to TX Firmware

Your TX firmware currently runs in bench test mode. To receive commands from ROS, add either UDP or Serial input:

### Option 1: UDP Server (Recommended for Wireless)

Add this to your TX setup():

```cpp
#include <WiFi.h>
#include <WiFiUdp.h>

WiFiUDP udp;
const int UDP_PORT = 9000;

void setup() {
  // ... existing setup ...
  
  // Connect to WiFi (or create AP)
  WiFi.begin("your-ssid", "your-password");
  // or WiFi.softAP("UAV_TX", "password123");
  
  udp.begin(UDP_PORT);
  Serial.printf("UDP server listening on port %d\n", UDP_PORT);
}
```

Add this to your TX loop():

```cpp
void loop() {
  // ... existing code ...
  
  // Check for UDP packets from ROS
  int packetSize = udp.parsePacket();
  if (packetSize == 20) {  // DirectCommandPayload size
    uint8_t buffer[20];
    udp.read(buffer, 20);
    
    // Parse DirectCommandPayload
    float* floats = (float*)buffer;
    uint32_t* timestamp = (uint32_t*)(buffer + 16);
    
    float roll = floats[0];
    float pitch = floats[1];
    float yaw = floats[2];
    float throttle = floats[3];
    
    // Send via custom protocol to RX
    CustomProtocol_SendDirectCommand(roll, pitch, yaw, throttle);
  }
  
  // ... rest of your code ...
}
```

### Option 2: Serial Input (Wired Connection)

If your TX is connected via USB, parse serial input:

```cpp
void loop() {
  // ... existing code ...
  
  // Check for serial packets from ROS
  if (Serial.available() >= 20) {
    uint8_t buffer[20];
    Serial.readBytes(buffer, 20);
    
    // Parse DirectCommandPayload (same as above)
    float* floats = (float*)buffer;
    float roll = floats[0];
    float pitch = floats[1];
    float yaw = floats[2];
    float throttle = floats[3];
    
    CustomProtocol_SendDirectCommand(roll, pitch, yaw, throttle);
  }
  
  // ... rest of your code ...
}
```

## ROS 2 Configuration

The CRSF adapter is already configured to send DirectCommandPayload format. Just launch it with your TX's IP:

### UDP Mode (Wireless)
```bash
./scripts/run_crsf_link_pid.sh transport:=udp udp_host:=192.168.4.1 udp_port:=9000
```

### Serial Mode (USB Cable)
```bash
./scripts/run_crsf_link_pid.sh transport:=serial serial_port:=/dev/ttyUSB0 baud:=115200
```

## Data Flow

```
ROS 2 Controllers
    ↓
Safety Gate (converts to VirtualRC)
    ↓
CRSF Adapter (packs to DirectCommandPayload, 20 bytes)
    ↓ UDP or Serial
TX ESP32 (receives, calls CustomProtocol_SendDirectCommand)
    ↓ ESP-NOW wireless
RX ESP32 (converts to CRSF format)
    ↓ UART CRSF
Betaflight / Flight Controller
```

## Coordinate Frames

- **ROS Controllers**: Output FRD body rates (rad/s)
- **Safety Gate**: Converts to normalized values [-1, 1] for roll/pitch/yaw, [0, 1] for throttle
- **CRSF Adapter**: Packs to DirectCommandPayload (same ranges)
- **TX Firmware**: Receives floats, sends via custom protocol
- **RX Firmware**: Converts to CRSF uint16 format (172-1811) for FC

## Testing Without Hardware

You can test the ROS stack in simulation without modifying TX firmware:

```bash
# Run simulation with PID controller
./scripts/run_sim_pid.sh

# In another terminal, monitor outputs
ros2 topic echo /cmd/final/rc
```

This lets you verify the full ROS pipeline before integrating with hardware.

## Example TX Firmware Modifications

Here's a complete example integrating UDP input into your TX:

```cpp
#ifdef BUILD_TX
// Add at top with other includes
#include <WiFi.h>
#include <WiFiUdp.h>

// Add with other TX configuration
WiFiUDP udp;
const int UDP_PORT = 9000;
bool wifiConnected = false;

void setup() {
  // ... existing setup ...
  
  // Setup WiFi AP mode
  WiFi.softAP("UAV_TX", "uav12345");
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  
  udp.begin(UDP_PORT);
  wifiConnected = true;
  
  Serial.printf("UDP server ready on port %d\n", UDP_PORT);
  // ... rest of setup ...
}

void loop() {
  unsigned long now = millis();
  CustomProtocol_Update();
  
  // Check for ROS commands via UDP
  if (wifiConnected) {
    int packetSize = udp.parsePacket();
    if (packetSize == 20) {
      uint8_t buffer[20];
      udp.read(buffer, 20);
      
      float* floats = (float*)buffer;
      float roll = floats[0];
      float pitch = floats[1]; 
      float yaw = floats[2];
      float throttle = floats[3];
      
      // Send to RX via custom protocol
      CustomProtocol_SendDirectCommand(roll, pitch, yaw, throttle);
      
      lastRcSendMs = now;  // Update timing
    }
  }
  
  // If no UDP commands, fall back to bench test or static mode
  if (now - lastRcSendMs >= RC_SEND_INTERVAL) {
    if (BENCH_TEST_MODE && !wifiConnected) {
      // ... existing bench test code ...
    }
  }
  
  // ... rest of loop (heartbeat, stats, etc.) ...
}
#endif
```

## Troubleshooting

**Q: TX not receiving UDP packets**
- Check ESP32 IP: `WiFi.softAPIP()` or `WiFi.localIP()`
- Verify ROS machine can reach it: `ping 192.168.4.1`
- Check firewall isn't blocking UDP port 9000
- Monitor TX serial output for debug info

**Q: Commands not reaching flight controller**
- Verify RX is receiving: check RX stats output
- Ensure RX has `ENABLE_CRSF_OUTPUT true`
- Check CRSF wiring: RX GPIO21 → FC UART RX pin
- Verify FC is configured for CRSF input

**Q: Control feels wrong**
- Double-check frame conventions (FRD vs NED)
- Tune PID/LQR gains in ROS config files
- Verify safety gate isn't over-limiting commands

## Safety Notes

- Always bench test with props off first
- Verify failsafe works: kill ROS node, TX should send safe values
- Check AUX channel mapping matches your arming setup
- Have manual RC override ready
- Start with low gains and increase gradually

---

**Quick Integration Checklist:**
- [ ] Add UDP or Serial input to TX firmware
- [ ] Configure TX WiFi (AP or Station mode)
- [ ] Rebuild and upload TX firmware
- [ ] Connect ROS machine to TX network
- [ ] Launch ROS stack with correct IP/port
- [ ] Verify commands reaching TX (serial monitor)
- [ ] Test with props off!

