## GPS Live Dashboard

### 1) Install Python deps

```bash
cd /home/cy/playground/embedded/adafruit_gps
python3 -m pip install -r requirements-dashboard.txt
```

### 2) Upload firmware to ESP32

```bash
~/.platformio/penv/bin/pio run -t upload
```

### 3) Start dashboard

```bash
streamlit run gps_dashboard.py -- --port /dev/ttyACM0 --baud 115200
```

Then open the local URL shown by Streamlit (usually `http://localhost:8501`).

### Notes

- If upload fails with lock errors, close any open monitor first.
- If no coordinates appear, wait for GPS fix (clear sky helps).
- Dashboard parses lines printed by `src/main.cpp` (`Fix`, `Satellites`, `Location`, `Altitude`, `Speed`).
