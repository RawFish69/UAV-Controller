// Custom Wireless Protocol - Compatible with TX_RX
// This is a SUBSET - only what's needed for hand IMU transmitter

#ifndef CUSTOM_PROTOCOL_H
#define CUSTOM_PROTOCOL_H

#include <Arduino.h>

#define CUSTOM_PROTO_MAGIC      0xA5
#define CUSTOM_PROTO_VERSION    0x01
#define CUSTOM_PROTO_MAX_PAYLOAD 128

enum CustomPacketType {
  PKT_RC_COMMAND    = 0x10,  // RC channel commands
  PKT_DIRECT_CMD    = 0x11,  // Direct control commands
  PKT_IMU_DATA      = 0x12,  // IMU orientation data (NEW)
  PKT_TELEMETRY_REQ = 0x20,  // Telemetry request
  PKT_TELEMETRY_DATA= 0x21,  // Telemetry data
  PKT_CONFIG        = 0x30,  // Configuration
  PKT_ACK           = 0xF0,  // Acknowledgment
  PKT_NACK          = 0xF1,  // Negative acknowledgment
  PKT_HEARTBEAT     = 0xFF   // Heartbeat
};

typedef struct __attribute__((packed)) {
  uint8_t magic;
  uint8_t version;
  uint8_t packetType;
  uint8_t sequence;
  uint16_t payloadLength;
  uint8_t payload[CUSTOM_PROTO_MAX_PAYLOAD];
  uint16_t crc16;
} CustomPacket;

// IMU orientation payload
typedef struct __attribute__((packed)) {
  float qw, qx, qy, qz;  // Quaternion (w, x, y, z)
  float throttle;        // 0.0 to 1.0
  uint32_t timestamp;
  uint8_t flags;         // Bit 0: calibrated, Bit 1: button pressed
} ImuDataPayload;

typedef struct {
  uint32_t packetsReceived;
  uint32_t packetsSent;
  uint32_t crcErrors;
  uint32_t timeoutErrors;
  uint32_t lastPacketMs;
  bool linkActive;
  uint32_t sendFailures;
  float packetLossPercent;
} ProtocolStats;

bool CustomProtocol_Init(bool isTx = true, const uint8_t* peerMac = nullptr);
void CustomProtocol_Update();
bool CustomProtocol_SendPacket(CustomPacketType type, const uint8_t* payload, uint16_t payloadLen);
bool CustomProtocol_SendImuData(float qw, float qx, float qy, float qz, float throttle, uint8_t flags);
bool CustomProtocol_SendHeartbeat();
bool CustomProtocol_IsLinkActive();
void CustomProtocol_GetStats(ProtocolStats* stats);
uint16_t CustomProtocol_CRC16(const uint8_t* data, size_t len);

#endif // CUSTOM_PROTOCOL_H

