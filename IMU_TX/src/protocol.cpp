// Minimal protocol implementation for Hand IMU TX
// Only implements sending functions needed for IMU transmitter

#include "protocol.h"
#include <esp_now.h>
#include <WiFi.h>

// Protocol state
static bool initialized = false;
static bool isTxMode = true;
static uint8_t sequenceNumber = 0;
static ProtocolStats stats = {0};
static uint8_t peerMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Forward declarations
static void onDataSent(const uint8_t* mac, esp_now_send_status_t status);

bool CustomProtocol_Init(bool isTx, const uint8_t* targetMac) {
  isTxMode = isTx;
  
  // Copy peer MAC
  if (targetMac) {
    memcpy(peerMac, targetMac, 6);
  }
  
  // Set WiFi mode
  WiFi.mode(WIFI_STA);
  
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    return false;
  }
  
  // Register send callback
  esp_now_register_send_cb(onDataSent);
  
  // Add peer (broadcast)
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    return false;
  }
  
  initialized = true;
  return true;
}

void CustomProtocol_Update() {
  // Check for link timeout
  unsigned long now = millis();
  if (stats.packetsSent > 0 && (now - stats.lastPacketMs > 2000)) {
    stats.linkActive = false;
  }
}

bool CustomProtocol_SendPacket(CustomPacketType type, const uint8_t* payload, uint16_t payloadLen) {
  if (!initialized) return false;
  
  CustomPacket packet;
  packet.magic = CUSTOM_PROTO_MAGIC;
  packet.version = CUSTOM_PROTO_VERSION;
  packet.packetType = type;
  packet.sequence = sequenceNumber++;
  packet.payloadLength = payloadLen;
  
  if (payloadLen > 0 && payloadLen <= CUSTOM_PROTO_MAX_PAYLOAD) {
    memcpy(packet.payload, payload, payloadLen);
  }
  
  // Calculate CRC over header + payload
  size_t crcLen = 6 + payloadLen;  // magic to payload
  packet.crc16 = CustomProtocol_CRC16((uint8_t*)&packet, crcLen);
  
  // Send via ESP-NOW
  size_t totalLen = 6 + payloadLen + 2;  // header + payload + crc
  esp_err_t result = esp_now_send(peerMac, (uint8_t*)&packet, totalLen);
  
  if (result == ESP_OK) {
    stats.packetsSent++;
    stats.lastPacketMs = millis();
    return true;
  } else {
    stats.sendFailures++;
    return false;
  }
}

bool CustomProtocol_SendImuData(float qw, float qx, float qy, float qz, 
                                 float throttle, uint8_t flags) {
  ImuDataPayload payload;
  payload.qw = qw;
  payload.qx = qx;
  payload.qy = qy;
  payload.qz = qz;
  payload.throttle = throttle;
  payload.timestamp = millis();
  payload.flags = flags;
  
  return CustomProtocol_SendPacket(PKT_IMU_DATA, (uint8_t*)&payload, sizeof(payload));
}

bool CustomProtocol_SendHeartbeat() {
  uint8_t dummy = 0;
  return CustomProtocol_SendPacket(PKT_HEARTBEAT, &dummy, 1);
}

bool CustomProtocol_IsLinkActive() {
  return stats.linkActive;
}

void CustomProtocol_GetStats(ProtocolStats* outStats) {
  if (outStats) {
    memcpy(outStats, &stats, sizeof(ProtocolStats));
  }
}

uint16_t CustomProtocol_CRC16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc = crc >> 1;
      }
    }
  }
  return crc;
}

// ESP-NOW callback
static void onDataSent(const uint8_t* mac, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    stats.linkActive = true;
  }
}

