# Universal RC Protocol Visualizer

Real-time RC channel visualization supporting CRSF, SBUS, and iBus protocols!

---

## 🚀 Quick Start

### 1. Install Dependencies
```bash
pip install -r requirements.txt
```

### 2. Run Visualizer

**3D Web Visualizer (Recommended):**
```bash
python drone_visualizer.py COM3
# That's it! Switch protocols in the web UI
```

**Terminal Monitor:**
```bash
python crsf_serial.py COM3   # For CRSF
python sbus_serial.py COM3   # For SBUS
python ibus_serial.py COM3   # For iBus
```

### 3. Open Browser
```
http://localhost:5000
```

---

## 📡 Supported Protocols

| Protocol | Type | Baud Rate | Use Case | Status |
|----------|------|-----------|----------|--------|
| **CRSF** | Serial | 420000 | Crossfire/ELRS (default) | ✅ Supported |
| **SBUS** | Serial | 100000 | Futaba, FrSky | ✅ Supported |
| **iBus** | Serial | 115200 | FlySky | ✅ Supported |
| **PPM** | GPIO | N/A | Traditional receivers | ⚠️ Requires hardware bridge |

**Note:** PPM is GPIO-based (pulse timing), not serial data. To monitor PPM:
- Use CRSF/SBUS/iBus output from receiver instead, OR
- Build ESP32 PPM-to-Serial bridge (future feature)

All serial protocols are automatically converted to CRSF range (172-1811) for consistent visualization.

---

## ✨ Features

### 🎨 3D Web Visualizer (`drone_visualizer.py`)
- **Dynamic protocol switching** - Change CRSF/SBUS/iBus in web UI (no restart!)
- **Beautiful 3D graphics** - Full quadcopter model with realistic motors
- **Real-time telemetry** - Roll, pitch, yaw, throttle
- **Interactive camera** - Drag to rotate, scroll to zoom
- **Flight modes** - Switch between ANGLE and ACRO modes
- **Web-based** - Access from any device on your network

### 📟 Terminal Monitors (`*_serial.py`)
- **Protocol-specific monitors** - Simple dedicated tool for each protocol
- **Short commands** - Just `python crsf_serial.py COM3`
- **Lightweight** - No web browser needed
- **ASCII dashboard** - All 16 channels with visual bars
- **Real-time stats** - Frame rate, error count

---

## 🔧 Usage

### 3D Web Visualizer (Recommended)
```bash
python drone_visualizer.py COM3
```
Then open browser to `http://localhost:5000` and:
- ✅ Click protocol buttons (CRSF/SBUS/iBus) to switch
- ✅ Click ANGLE/ACRO to change flight mode
- ✅ Drag to rotate view, scroll to zoom

### Terminal Monitors (Lightweight)
```bash
python crsf_serial.py COM3  # For CRSF protocol
python sbus_serial.py COM3  # For SBUS protocol
python ibus_serial.py COM3  # For iBus protocol
```

**That's it!** No flags, no complicated arguments. Just the port name.

---

## 📁 Project Structure

```
Utils/
├── protocols/              # 📁 Protocol decoder modules
│   ├── __init__.py         # Protocol registry
│   ├── crsf_decoder.py     # CRSF decoder
│   ├── sbus_decoder.py     # SBUS decoder
│   ├── ibus_decoder.py     # iBus decoder
│   └── ppm_decoder.py      # PPM placeholder (needs hardware bridge)
├── drone_visualizer.py ⭐  # 3D web visualizer (CRSF/SBUS/iBus)
├── crsf_serial.py          # CRSF terminal monitor
├── sbus_serial.py          # SBUS terminal monitor
├── ibus_serial.py          # iBus terminal monitor
├── requirements.txt        # Python dependencies
└── README.md               # This file
```

---

## 🎮 Channel Mapping

All protocols display channels in CRSF format:

| Channel | Function | Range | Center | Description |
|---------|----------|-------|--------|-------------|
| **CH1** | Roll | 172-1811 | 992 | Left/Right tilt |
| **CH2** | Pitch | 172-1811 | 992 | Forward/Back tilt |
| **CH3** | Throttle | 172-1811 | 172 | Up/Down |
| **CH4** | Yaw | 172-1811 | 992 | Rotation |
| **CH5** | AUX1 | 172-1811 | 172/1811 | Arming |
| **CH6-16** | AUX2-12 | 172-1811 | 992 | Additional channels |

---

## 🌐 Web Interface Features

### Protocol Switching (NEW!)
Click **CRSF** / **SBUS** / **iBus** buttons to switch protocols on-the-fly!
- No need to restart the app
- Automatically reconnects with new protocol
- Perfect for testing different receiver configurations

### Flight Modes
- **ANGLE Mode**: Self-leveling, max ±45° tilt
- **ACRO Mode**: Rate control, full 360° rotation

### Connection Status
- **Green dot + "Connected"**: Receiving data
- **Red dot + "Disconnected"**: No data

### Interactive 3D View
- **Drag**: Rotate camera around drone
- **Scroll**: Zoom in/out
- **Real-time updates**: 20 Hz smooth animation

---

## 🐛 Troubleshooting

### Module Not Found
```bash
pip install flask pyserial
```

### Wrong Protocol
Make sure your receiver is outputting the protocol you're trying to decode:
- CRSF: 420000 baud
- SBUS: 100000 baud (8E2)
- iBus: 115200 baud

### No Data Received
- Check serial port name (COM3, /dev/ttyUSB0, etc.)
- Verify receiver is powered and transmitting
- **Web UI**: Click different protocol buttons to match your receiver
- **Terminal**: Use the matching monitor (crsf_serial.py for CRSF, etc.)

### Linux Permission Denied
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

---

## ❓ Why No PPM?

**PPM is GPIO pulses, not serial data!**

Your computer's USB port reads **serial data** (UART). PPM is **timed pulses on a GPIO pin** - completely different!

**What you can do:**
1. **Best solution**: Configure your receiver for CRSF/SBUS/iBus output (edit `TX_RX/src/config.h`)
2. **For flight**: Use PPM on your drone
3. **For debugging**: Reflash receiver with CRSF to monitor, then reflash back to PPM

Your universal receiver supports protocol switching by reflashing - takes 30 seconds!

## 🔌 How to Monitor Your Receiver

**Step 1:** Connect receiver to computer via USB (not to flight controller)

**Step 2:** Check which protocol your receiver is using:
- Open `TX_RX/src/config.h`
- Look for: `#define OUTPUT_PROTOCOL PROTOCOL_XXXX`

**Step 3:** Run matching tool:
```bash
# If PROTOCOL_CRSF:
python drone_visualizer.py COM3
# or
python crsf_serial.py COM3

# If PROTOCOL_SBUS:
python drone_visualizer.py COM3  # then click SBUS in UI
# or
python sbus_serial.py COM3

# If PROTOCOL_IBUS:
python drone_visualizer.py COM3  # then click iBus in UI
# or
python ibus_serial.py COM3

# If PROTOCOL_PPM:
# Change config.h to PROTOCOL_CRSF, reflash receiver, then monitor
```

---

## 💡 Pro Tips

- **Web UI protocol switching**: Change protocols without restarting! Just click in the UI
- **Network access**: Others can view 3D viz at `http://YOUR_IP:5000`
- **Red nose**: Points forward (front of drone)
- **Motor colors**: Front=Red, Back=Blue
- **Altitude**: Virtual (based on throttle, not real sensor)
- **Simple commands**: Just `python tool_name.py COM3` - that's it!

---

## 🎯 Use Cases

- **Testing TX/RX system** - Verify all channels work with any protocol
- **Tuning IMU sensitivity** - See real-time tilt angles
- **Debugging protocol output** - Monitor frame rate and errors
- **Protocol comparison** - Quickly switch protocols to test compatibility
- **Demos** - Show your system to others with cool 3D viz

**💡 Best Practice:** 
- **For development/testing**: Use CRSF (fastest, most features)
- **For flight**: Use whatever your FC needs (CRSF/SBUS/PPM/iBus)
- **Need to debug PPM config?**: Temporarily reflash with CRSF, debug, then reflash back

---

## 🔄 Command Summary

**One Tool for Everything:**
```bash
python drone_visualizer.py COM3
```
- Opens web UI at http://localhost:5000
- Switch protocols with buttons: **[CRSF]** **[SBUS]** **[iBus]**
- Switch flight modes: **[ANGLE]** **[ACRO]**
- Drag to rotate, scroll to zoom
- Works on your network too!

**Terminal Monitors (if you prefer CLI):**
```bash
python crsf_serial.py COM3  # CRSF only
python sbus_serial.py COM3  # SBUS only
python ibus_serial.py COM3  # iBus only
```

---

**Enjoy your universal drone visualizer! 🚁✨**
