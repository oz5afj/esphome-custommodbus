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
  ESP_LOGI(TAG, "SunModbus setup: slave_id=%u start_address=%u count=%u offset=%u scale=%f type=%d interval=%u",
           this->slave_id_, this->start_address_, this->count_, this->offset_, this->scale_, this->type_, this->update_interval_);
}

void SunModbus::update() {
  uint32_t t0 = millis();
  ESP_LOGD(TAG, "update() start");

  if (this->uart_ == nullptr || this->sensor_ == nullptr) {
    ESP_LOGW(TAG, "UART or sensor not set");
    ESP_LOGD(TAG, "update() end, took %u ms", millis() - t0);
    return;
  }

  uint8_t resp[256] = {0};

  if (!this->read_holding_registers_(this->slave_id_, this->start_address_, this->count_, resp, sizeof(resp))) {
    ESP_LOGW(TAG, "Failed to read holding registers");
    ESP_LOGD(TAG, "update() end, took %u ms", millis() - t0);
    return;
  }

  if (this->count_ < 1) {
    ESP_LOGW(TAG, "No registers read");
    ESP_LOGD(TAG, "update() end, took %u ms", millis() - t0);
    return;
  }

  uint16_t raw = (resp[0] << 8) | resp[1];
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

bool SunModbus::read_holding_registers_(uint8_t slave, uint16_t start, uint16_t count, uint8_t *buffer, uint16_t len) {
  uint32_t t0 = millis();
  ESP_LOGD(TAG, "read_holding_registers_ start slave=%u start=%u count=%u", slave, start, count);

  if (count == 0 || count > 125) {
    ESP_LOGW(TAG, "Invalid count %u", count);
    return false;
  }
  uint16_t expected_data_bytes = count * 2;
  uint16_t expected_response_len = 5 + expected_data_bytes; // addr + func + bytecount + data + crc(2)

  if (len < expected_data_bytes) {
    ESP_LOGW(TAG, "Buffer too small: need %u bytes", expected_data_bytes);
    return false;
  }

  uint8_t req[8];
  req[0] = slave;
  req[1] = 0x03; // Read Holding Registers
  req[2] = (start >> 8) & 0xFF;
  req[3] = start & 0xFF;
  req[4] = (count >> 8) & 0xFF;
  req[5] = count & 0xFF;
  uint16_t crc = crc16_modbus(req, 6);
  req[6] = crc & 0xFF;
  req[7] = (crc >> 8) & 0xFF;

  // Flush RX buffer before sending
  this->flush();

  // Send request
  this->write_array(req, 8);

  // Non-blocking-ish read loop with short timeout and small yields
  uint32_t timeout_ms = 120; // short timeout to avoid blocking main loop
  uint32_t start_ms = millis();
  uint16_t idx = 0;
  while ((millis() - start_ms) < timeout_ms && idx < expected_response_len) {
    int avail = this->available();
    if (avail > 0) {
      int to_read = std::min<int>(avail, expected_response_len - idx);
      int r = this->read_array(buffer + idx, to_read);
      if (r > 0) idx += r;
    } else {
      delay(0); // yield
    }
  }

  if (idx < 5) {
    ESP_LOGW(TAG, "Response too short: %u bytes (timeout %u ms)", idx, timeout_ms);
    ESP_LOGD(TAG, "read_holding_registers_ end, took %u ms, bytes=%u", millis() - t0, idx);
    return false;
  }

  // Validate CRC (last two bytes)
  uint16_t resp_crc = (uint16_t)buffer[idx - 2] | ((uint16_t)buffer[idx - 1] << 8);
  if (crc16_modbus(buffer, idx - 2) != resp_crc) {
    ESP_LOGW(TAG, "CRC mismatch");
    ESP_LOGD(TAG, "read_holding_registers_ end, took %u ms, bytes=%u", millis() - t0, idx);
    return false;
  }

  // Check for exception
  if (buffer[1] & 0x80) {
    ESP_LOGW(TAG, "Modbus exception code: %u", buffer[2]);
    ESP_LOGD(TAG, "read_holding_registers_ end, took %u ms, bytes=%u", millis() - t0, idx);
    return false;
  }

  uint8_t bytecount = buffer[2];
  if (bytecount == 0) {
    ESP_LOGW(TAG, "Zero bytecount in response");
    ESP_LOGD(TAG, "read_holding_registers_ end, took %u ms, bytes=%u", millis() - t0, idx);
    return false;
  }

  uint16_t copy_len = std::min<uint16_t>(bytecount, expected_data_bytes);
  for (uint16_t i = 0; i < copy_len; ++i) {
    buffer[i] = buffer[3 + i];
  }

  ESP_LOGD(TAG, "read_holding_registers_ end, took %u ms, bytes=%u", millis() - t0, idx);
  return true;
}

}  // namespace sunmodbus
}  // namespace esphome
