#include "sunmodbus.h"
#include "esphome/core/log.h"

namespace esphome {
namespace sunmodbus {

static const char *const TAG = "sunmodbus_block10";

static uint16_t crc16_modbus(const uint8_t *buf, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t pos = 0; pos < len; pos++) {
    crc ^= buf[pos];
    for (int i = 0; i < 8; i++) {
      if (crc & 1)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }
  return crc;
}

void SunModbus::setup() {
  ESP_LOGI(TAG, "Setup: slave=%u start_address=%u count=10", this->slave_id_, this->start_address_);
}

void SunModbus::update() {
  uint8_t resp[5 + 20 + 2] = {0};  // header + 10 regs + crc

  if (!this->read_block10_(this->slave_id_, this->start_address_, resp, sizeof(resp))) {
    ESP_LOGW(TAG, "Block read failed");
    return;
  }

  // Data starter ved resp[3], bytecount=20 ved resp[2]
  if (resp[2] != 20) {
    ESP_LOGW(TAG, "Unexpected bytecount %u (expected 20)", resp[2]);
    return;
  }

  for (int i = 0; i < 10; i++) {
    auto *s = this->sensors_[i];
    if (s == nullptr) continue;

    uint8_t hi = resp[3 + i * 2];
    uint8_t lo = resp[3 + i * 2 + 1];
    uint16_t raw = ((uint16_t) hi << 8) | (uint16_t) lo;

    float value = static_cast<int16_t>(raw);  // tolkes som signed; kan justeres per sensor senere

    s->publish_state(value);
  }
}

bool SunModbus::read_block10_(uint8_t slave, uint16_t start, uint8_t *buffer, uint16_t len) {
  if (len < 27)  // 1 addr + 1 func + 1 bytecount + 20 data + 2 crc
    return false;

  uint8_t req[8];
  req[0] = slave;
  req[1] = 0x03;  // holding registers
  req[2] = (start >> 8) & 0xFF;
  req[3] = start & 0xFF;
  req[4] = 0x00;
  req[5] = 0x0A;  // 10 registers
  uint16_t crc = crc16_modbus(req, 6);
  req[6] = crc & 0xFF;
  req[7] = (crc >> 8) & 0xFF;

  this->flush();
  this->write_array(req, 8);

  uint32_t timeout_ms = 150;
  uint32_t start_ms = millis();
  uint16_t idx = 0;
  const uint16_t expected = 27;

  while ((millis() - start_ms) < timeout_ms && idx < expected) {
    int avail = this->available();
    if (avail > 0) {
      int to_read = std::min<int>(avail, expected - idx);
      idx += this->read_array(buffer + idx, to_read);
    } else {
      delay(0);
    }
  }

  if (idx < expected) {
    ESP_LOGW(TAG, "Short response: %u bytes (expected %u)", idx, expected);
    return false;
  }

  uint16_t resp_crc = (uint16_t) buffer[expected - 2] | ((uint16_t) buffer[expected - 1] << 8);
  if (crc16_modbus(buffer, expected - 2) != resp_crc) {
    ESP_LOGW(TAG, "CRC mismatch");
    return false;
  }

  if (buffer[1] & 0x80) {
    ESP_LOGW(TAG, "Modbus exception %u", buffer[2]);
    return false;
  }

  return true;
}

}  // namespace sunmodbus
}  // namespace esphome
