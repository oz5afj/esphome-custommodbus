#include "sunmodbus.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace sunmodbus {

static const char *const TAG = "sunmodbus";

static uint16_t crc16_modbus(const uint8_t *buf, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];
    for (int i = 0; i < 8; i++) {
      if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
      else crc = (crc >> 1);
    }
  }
  return crc;
}

void SunModbus::setup() {
  ESP_LOGI(TAG, "SunModbus setup: slave_id=%u start_address=%u offset=%u scale=%f type=%d interval=%u",
           this->slave_id_, this->start_address_, this->offset_, this->scale_, this->type_, this->update_interval_);
}

void SunModbus::update() {
  uint32_t t0 = millis();
  ESP_LOGD(TAG, "update() start");

  if (this->uart_ == nullptr || this->sensor_ == nullptr) {
    ESP_LOGW(TAG, "UART or sensor not set");
    ESP_LOGD(TAG, "update() end, took %u ms", millis() - t0);
    return;
  }

  uint16_t raw = 0;
  if (!this->read_single_register_(this->slave_id_, this->start_address_, raw)) {
    ESP_LOGW(TAG, "Failed to read holding register");
    ESP_LOGD(TAG, "update() end, took %u ms", millis() - t0);
    return;
  }

  float value = 0.0f;

  if (this->type_ == TYPE_UINT16) {
    value = static_cast<uint16_t>(raw);
  } else {
    int16_t s = static_cast<int16_t>(raw);
    value = static_cast<float>(s);
  }

  value = (value + this->offset_) * this->scale_;
  this->sensor_->publish_state(value);

  ESP_LOGD(TAG, "update() end, took %u ms", millis() - t0);
}

bool SunModbus::read_single_register_(uint8_t slave, uint16_t address, uint16_t &out_value) {
  uint32_t t0 = millis();
  ESP_LOGD(TAG, "read_single_register_ slave=%u address=%u", slave, address);

  uint8_t req[8];
  req[0] = slave;
  req[1] = 0x03;  // Read Holding Registers
  req[2] = (address >> 8) & 0xFF;
  req[3] = address & 0xFF;
  req[4] = 0x00;
  req[5] = 0x01;  // count = 1
  uint16_t crc = crc16_modbus(req, 6);
  req[6] = crc & 0xFF;
  req[7] = (crc >> 8) & 0xFF;

  this->flush();
  this->write_array(req, 8);

  uint8_t resp[8] = {0};
  uint32_t timeout_ms = 100;
  uint32_t start_ms = millis();
  uint16_t idx = 0;

  const uint16_t expected_len = 7;  // addr + func + bytecount + 2 data + 2 crc

  while ((millis() - start_ms) < timeout_ms && idx < expected_len) {
    int avail = this->available();
    if (avail > 0) {
      int to_read = std::min<int>(avail, expected_len - idx);
      int r = this->read_array(resp + idx, to_read);
      if (r > 0) idx += r;
    } else {
      delay(0);
    }
  }

  if (idx < expected_len) {
    ESP_LOGW(TAG, "Response too short: %u bytes (expected %u)", idx, expected_len);
    return false;
  }

  uint16_t resp_crc = (uint16_t)resp[expected_len - 2] | ((uint16_t)resp[expected_len - 1] << 8);
  if (crc16_modbus(resp, expected_len - 2) != resp_crc) {
    ESP_LOGW(TAG, "CRC mismatch");
    return false;
  }

  if (resp[1] & 0x80) {
    ESP_LOGW(TAG, "Modbus exception code: %u", resp[2]);
    return false;
  }

  if (resp[2] != 2) {
    ESP_LOGW(TAG, "Unexpected bytecount %u (expected 2)", resp[2]);
    return false;
  }

  out_value = ((uint16_t)resp[3] << 8) | (uint16_t)resp[4];

  ESP_LOGD(TAG, "read_single_register_ ok, value=%u, took %u ms", out_value, millis() - t0);
  return true;
}

}  // namespace sunmodbus
}  // namespace esphome
