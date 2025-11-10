#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#ifndef RGB_PIN
#define RGB_PIN 21
#endif
#define NUMPIXELS 1
Adafruit_NeoPixel led(NUMPIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);

//params
static bool armed = true;

// CRSF basics
static const uint8_t CRSF_ADDR_FC = 0xC8; // Flight Controller address
static const uint8_t CRSF_TYPE_RC = 0x16; // RC Channel Data packet type (16 channels each 11 bits)

HardwareSerial CRSF(1); // Use UART1 for CRSF communication
constexpr int FC_TX_PIN = 43; // ESP32 TX -> connects to FC RX (A10 for UART1)
constexpr int FC_RX_PIN = 44; // ESP32 RX -> connects to FC TX (A9 for UART1)
constexpr int CRSF_BAUDRATE = 420000; // CRSF baud rate

// RC channel data
static inline uint16_t clampCRSF(int v){
    return (v < 172) ? 172 : (v > 1811) ? 1811 : (uint16_t)v;
}
static inline uint16_t mid() { return 992; }
static inline uint16_t thr_min() { return 172; }
static inline uint16_t aux_low() { return 172; }
static inline uint16_t aux_high() { return 1811; }

uint16_t ch[16] = {
  mid(),
  mid(),
  thr_min(),
  mid(),
  aux_low(),
  mid(), mid(), mid(), mid(), mid(), mid(), mid(), mid(), mid(), mid(), mid()
};

void setArm(bool armed) {
  ch[4] = armed ? aux_high() : aux_low();
}

uint8_t crc8(const uint8_t*data, size_t len){
  uint8_t c = 0x00;
  for(size_t i = 0; i < len; i++){
    c ^= data[i];
    for(int j = 0; j < 8; j++){
      c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0xD5) : (uint8_t)(c << 1);
    }
  }
  return c;
}

void send_rc_frame(){
  uint8_t f[26];
  f[0] = CRSF_ADDR_FC;        // Address
  f[1] = 24;                  // Length = TYPE(1) + PAYLOAD(22) + CRC(1)
  f[2] = CRSF_TYPE_RC;          // Type = RC Channel Data

  // Pack channel data into frame
  f[3] = ch[0] & 0xFF;
  f[4] = (ch[0] >> 8) | (uint8_t)((ch[1] & 0x07) << 3);
  f[5] = (ch[1] >> 5) | (uint8_t)((ch[2] & 0x3F) << 6);
  f[6] = ch[2] >> 2;
  f[7] = (ch[2] >> 10) | (uint8_t)((ch[3] & 0x01FF) << 1);
  f[8] = (ch[3] >> 7) | (uint8_t)((ch[4] & 0x0F) << 4);
  f[9]  = (ch[4] >> 4)  | (uint8_t)((ch[5] & 0x01) << 7);
  f[10] =  ch[5] >> 1;
  f[11] = (ch[5] >> 9)  | (uint8_t)((ch[6] & 0x3) << 2);
  f[12] = (ch[6] >> 6)  | (uint8_t)((ch[7] & 0x1F) << 5);
  f[13] =  ch[7] >> 3;

  f[14] =  ch[8] & 0xFF;
  f[15] = (ch[8] >> 8)  | (uint8_t)((ch[9] & 0x07) << 3);
  f[16] = (ch[9] >> 5)  | (uint8_t)((ch[10] & 0x3F) << 6);
  f[17] =  ch[10] >> 2;
  f[18] = (ch[10] >> 10) | (uint8_t)((ch[11] & 0x01FF) << 1);
  f[19] = (ch[11] >> 7)  | (uint8_t)((ch[12] & 0x0F) << 4);
  f[20] = (ch[12] >> 4)  | (uint8_t)((ch[13] & 0x01) << 7);
  f[21] =  ch[13] >> 1;
  f[22] = (ch[13] >> 9)  | (uint8_t)((ch[14] & 0x3) << 2);
  f[23] = (ch[14] >> 6)  | (uint8_t)((ch[15] & 0x1F) << 5);
  f[24] =  ch[15] >> 3;
  f[25] = crc8(&f[2], 23);
  CRSF.write(f, sizeof(f));
}

// Telemetry request frame
enum RXState{ S_ADDR, S_LEN, S_BODY};
RXState rx_state = S_ADDR;
uint8_t rxAddr = 0, rxLen = 0;
uint8_t rxBuf[64];
uint8_t rxIdx=0;

void hexdump(const uint8_t* d, int n){
  for(int i=0;i<n;++i){ Serial.printf("%02X ", d[i]); }
}

void parse_link_stats(const uint8_t* p, int n){
  Serial.print("[CRSF] LinkStats payload: ");
  hexdump(p, n); Serial.println();
}

void parse_gps(const uint8_t* p, int n){
  Serial.print("[CRSF] GPS payload: ");
  hexdump(p, n); Serial.println();
}

void parse_battery(const uint8_t* p, int n){
  Serial.print("[CRSF] Battery payload: ");
  hexdump(p, n); Serial.println();
}

void parse_attitude(const uint8_t* p, int n){
  Serial.print("[CRSF] Attitude payload: ");
  hexdump(p, n); Serial.println();
}

void parse_device_info(const uint8_t* p, int n){
  Serial.print("[CRSF] DeviceInfo payload: ");
  hexdump(p, n); Serial.println();
}

void handle_frame(uint8_t type, const uint8_t* payload, int plen){
  switch(type){
    case 0x16: // RC channels (usually TX->FC; seeing one here would be odd)
      Serial.println("[CRSF] RC frame observed on RX line");
      break;
    case 0x1C: // ink statistics
      parse_link_stats(payload, plen);
      break;
    case 0x02: //  GPS
      parse_gps(payload, plen);
      break;
    case 0x14: //  Battery or Device info in some stacks
    case 0x08: // (In others) Battery sensor
      parse_battery(payload, plen);
      break;
    case 0x1D: // Attitude
      parse_attitude(payload, plen);
      break;
    case 0x10: // (Often) Device info
      parse_device_info(payload, plen);
      break;
    default:
      Serial.printf("[CRSF] Type 0x%02X payload (%d B): ", type, plen);
      hexdump(payload, plen); Serial.println();
      break;
  }
}

void poll_rx(){
  while(CRSF.available()){
    uint8_t b = CRSF.read();
    switch(rx_state){
      case S_ADDR:
        rxAddr = b; rx_state = S_LEN; break;
      case S_LEN:
        rxLen = b;
        if(rxLen==0 || rxLen > sizeof(rxBuf)) { rx_state = S_ADDR; break; }
        rxIdx = 0; rx_state = S_BODY; break;
      case S_BODY:
        rxBuf[rxIdx++] = b;
        if(rxIdx == rxLen){
          uint8_t type = rxBuf[0];
          uint8_t crc  = rxBuf[rxLen-1];
          uint8_t calc = crc8(rxBuf, rxLen-1);
          if (calc == crc){
            handle_frame(type, &rxBuf[1], rxLen-2);
          } else {
            Serial.printf("[CRSF] CRC FAIL type=0x%02X len=%u calc=%02X got=%02X\n", type, rxLen, calc, crc);
          }
          rx_state = S_ADDR;
        }
        break;
    }
  }
}

inline void led_set(uint8_t r,uint8_t g,uint8_t b){
  led.setPixelColor(0, led.Color(r,g,b)); led.show();
}

void setup(){
  Serial.begin(115200);
  delay(200);
  led.begin(); led.setBrightness(40); led_set(255,255,255);
  CRSF.begin(CRSF_BAUDRATE, SERIAL_8N1, FC_RX_PIN, FC_TX_PIN);
  setArm(false);
}

void loop(){
  // Send RC at ~50 Hz (only CH5 varies; others constant)
  static uint32_t tSend=0;
  uint32_t now = millis();
  if (now - tSend >= 20){
    tSend = now;
    send_rc_frame();
  }
  setArm(armed);
  // if armed true blink green, else red
  if (armed){
    led_set(0,255,0);
  } else {
    led_set(255,0,0);
  }
  poll_rx();
}