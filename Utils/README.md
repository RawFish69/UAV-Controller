# Drone 3D Web Visualizer

Real-time 3D drone visualization in your browser! Decodes CRSF packets and displays beautiful 3D drone orientation using Three.js.

## 🚀 Quick Start

**1. Install dependencies:**
```bash
pip install -r requirements.txt
```

**2. Run the visualizer:**
```bash
python drone_visualizer.py COM3
```
*(Replace COM3 with your serial port)*

**3. Open your browser:**
```
http://localhost:5000
```

That's it! You'll see a beautiful 3D drone responding to your RC inputs in real-time.

---

## ✨ Features

### 🎨 Beautiful 3D Graphics
- Full 3D quadcopter model with realistic motors
- Red front indicator for easy orientation
- Smooth animations at 60 FPS
- Interactive camera controls (drag to rotate, scroll to zoom)

### 📊 Real-Time Telemetry
- Roll, Pitch, Yaw angles
- Throttle percentage with visual bars
- Frame rate and error monitoring
- Raw channel values (Ch1-4)
- Connection status indicator

### 🖱️ Interactive Controls
- **Drag** to rotate camera view
- **Scroll** to zoom in/out
- Fully responsive 3D scene

### 🌐 Web-Based
- No complex installations (matplotlib, numpy, etc.)
- Works in any modern browser
- Access from any device on your network
- Clean, professional interface

---

## 📦 What You Need

Just two Python packages:
- `pyserial` - For reading CRSF data
- `flask` - For the web server

Three.js is loaded from CDN - no download needed!

---

## 🎮 How It Works

1. **Python backend** reads CRSF packets from serial port
2. **Decodes** the 16-channel RC data (11-bit packed format)
3. **Converts** to roll/pitch/yaw angles
4. **Serves** via Flask web server
5. **Browser** polls for updates and renders in 3D using Three.js

The visualization updates 20 times per second, providing smooth real-time feedback.

---

## 🔌 Finding Your Serial Port

### Windows
```
COM3, COM4, COM5, etc.
Check: Device Manager → Ports (COM & LPT)
```

### Linux
```bash
ls /dev/ttyUSB*
ls /dev/ttyACM*
# Usually: /dev/ttyUSB0
```

### Mac
```bash
ls /dev/tty.*
# Usually: /dev/tty.usbserial-XXXX
```

---

## 🎯 Channel Mapping

| Channel | Control   | Range    | Center | Effect                          |
|---------|-----------|----------|--------|---------------------------------|
| 1       | Roll      | 172-1811 | 992    | ±45° left/right tilt            |
| 2       | Pitch     | 172-1811 | 992    | ±45° forward/back tilt          |
| 3       | Throttle  | 172-1811 | 172    | 0-3m altitude                   |
| 4       | Yaw       | 172-1811 | 992    | Rotation speed                  |

**CRSF Standard Values:**
- MIN: 172 (0%)
- MID: 992 (50%)  
- MAX: 1811 (100%)

---

## 🐛 Troubleshooting

**"No module named flask"**
```bash
pip install flask pyserial
```

**Can't see drone moving?**
- Check serial port is correct
- Verify ESP32 is transmitting CRSF
- Check browser console for errors (F12)
- Try refreshing the page

**"Address already in use"**
```bash
# Port 5000 is busy, kill the process or change port in code
```

**Permission denied (Linux)**
```bash
sudo usermod -a -G dialout $USER
# Then log out and log back in
```

**Firewall issues?**
- Allow Python/Flask through firewall
- Try accessing from same computer first
- Use `http://localhost:5000` not `127.0.0.1`

---

## 🌟 Pro Tips

- The visualizer works on your local network! Use `http://YOUR_IP:5000` from other devices
- Red cone points forward - that's the front of your drone
- Motor colors: Front=Red, Back=Gray, Sides=Blue
- The altitude is virtual based on throttle (not actual sensor data)
- Yaw accumulates over time - it keeps spinning!

---

## 📱 Mobile Friendly

Since it's web-based, you can view the visualization on your phone or tablet! Just connect to:
```
http://YOUR_COMPUTER_IP:5000
```

Perfect for monitoring your drone from across the room during testing.

---

## 🔧 Also Included

### Terminal Dashboard (`crsf_live.py`)

Still want a simple terminal view? Use the lightweight ASCII dashboard:

```bash
python crsf_live.py COM3
```

Shows all 16 channels with bars, no browser needed!

---

## 📝 File Structure

```
Utils/
├── drone_visualizer.py    # Web-based 3D visualizer
├── crsf_live.py           # Terminal dashboard
├── requirements.txt       # Just flask + pyserial
└── README.md              # This file
```

Clean and simple! Just 2 main files.

---

Made with ❤️ for the ESP_TX project

**Enjoy your 3D drone visualization! 🚁✨**
