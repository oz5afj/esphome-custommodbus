#include "sunmodbus.h"
#include "esphome/core/log.h"

namespace esphome {
namespace sunmodbus {

static const char *const TAG = "sunmodbus_safe_test";

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
  ESP_LOGI(TAG, "SAFE setup start");
  ESP_LOGI(TAG, "Slave=%u start=%u", this->slave_id_, this->start_address_);
  ESP_LOGI(TAG, "SAFE setup done");
}

void SunModbus::update() {
  ESP_LOGI(TAG, "SAFE ENTER update()");

  // Læs KUN 5 registre fra start_address_
  uint8_t resp[5 + 10 + 2] = {0};  // 5 regs = 10 data bytes
  if (!this->read_block_(this->slave_id_, this->start_address_, 5, resp, sizeof(resp))) {
    ESP_LOGW(TAG, "SAFE block read failed");
    return;
  }

  ESP_LOGI(TAG, "SAFE block OK, bytecount=%u", resp[2]);

  // Publish kun første register (reg0)
  if (this->sensors_[0] != nullptr) {
    uint8_t hi = resp[3];
    uint8_t lo = resp[4];
    int16_t raw = (hi << 8) | lo;
    ESP_LOGI(TAG, "SAFE reg0 raw=%d", raw);
    this->sensors_[0]->publish_state(raw);
  } else {
    ESP_LOGW(TAG, "SAFE reg0 sensor is null");
  }

  ESP_LOGI(TAG, "SAFE EXIT update()");
}

bool SunModbus::read_block_(uint8_t slave, uint16_t start, uint8_t count, uint8_t *buffer, uint16_t len) {
  ESP_LOGI(TAG, "SAFE ENTER read_block_ start=%u count=%u", start, count);

  uint16_t expected = 5 + count * 2 + 2;  // addr+func+bytecount + data + crc
  if (len < expected) {
    ESP_LOGW(TAG, "SAFE buffer too small: len=%u expected=%u", len, expected);
    return false;
  }

  uint8_t req[8];
  req[0] = slave;
  req[1] = 0x03;
  req[2] = start >> 8;
  req[3] = start & 0xFF;
  req[4] = 0x00;
  req[5] = count;
  uint16_t crc = crc16_modbus(req, 6);
  req[6] = crc & 0xFF;
  req[7] = crc >> 8;

  ESP_LOGI(TAG, "SAFE sending request");
  this->flush();
  this->write_array(req, 8);

  uint32_t timeout = millis() + 200;
  uint16_t idx = 0;

  ESP_LOGI(TAG, "SAFE waiting for response, expected=%u", expected);
  while (millis() < timeout && idx < expected) {
    int avail = this->available();
    if (avail > 0) {
      int to_read = std::min<int>(avail, expected - idx);
      int r = this->read_array(buffer + idx, to_read);
      idx += r;
      ESP_LOGI(TAG, "SAFE read %d bytes (total %u/%u)", r, idx, expected);
    } else {
      delay(0);
    }
  }

  if (idx < expected) {
    ESP_LOGW(TAG, "SAFE short response: %u/%u", idx, expected);
    return false;
  }

  ESP_LOGI(TAG, "SAFE response complete, checking CRC");
  uint16_t resp_crc = buffer[expected - 2] | (buffer[expected - 1] << 8);
  uint16_t calc_crc = crc16_modbus(buffer, expected - 2);
  if (calc_crc != resp_crc) {
    ESP_LOGW(TAG, "SAFE CRC mismatch calc=0x%04X resp=0x%04X", calc_crc, resp_crc);
    return false;
  }

  ESP_LOGI(TAG, "SAFE CRC OK");
  ESP_LOGI(TAG, "SAFE EXIT read_block_");
  return true;
}

}  // namespace sunmodbus
}  // namespace esphome
