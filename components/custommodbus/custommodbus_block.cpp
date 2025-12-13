#include "custommodbus_block.h"
#include "esphome/core/log.h"

namespace custommodbus {

static const char *const TAG = "custommodbus_block";

void CustomModbusBlock::loop() {
  const uint32_t now = millis();
  if (now - last_poll_ < poll_interval_)
    return;
  last_poll_ = now;

  if (sensors_.empty() || count_ == 0) {
    ESP_LOGW(TAG, "No sensors or count=0, skipping poll");
    return;
  }

  std::vector<uint16_t> regs;
  if (!this->read_block_(regs)) {
    ESP_LOGW(TAG, "Block read failed");
    return;
  }

  // regs.size() should be == count_
  for (auto &cfg : sensors_) {
    if (cfg.offset >= regs.size()) {
      ESP_LOGW(TAG, "Offset %u out of range (regs=%u)", cfg.offset, (unsigned) regs.size());
      continue;
    }

    uint16_t raw = regs[cfg.offset];
    float value = 0.0f;

    switch (cfg.type) {
      case TYPE_UINT16:
        value = static_cast<float>(raw);
        break;
      case TYPE_INT16:
        value = static_cast<int16_t>(raw);
        break;
      default:
        ESP_LOGW(TAG, "Unknown data type");
        continue;
    }

    value *= cfg.scale;

    if (cfg.sensor != nullptr) {
      cfg.sensor->publish_state(value);
    }
  }
}

bool CustomModbusBlock::read_block_(std::vector<uint16_t> &regs) {
  // Build request frame: [slave][func=3][start_hi][start_lo][count_hi][count_lo][crc_lo][crc_hi]
  uint8_t frame[8];
  frame[0] = slave_id_;
  frame[1] = 0x03;
  frame[2] = (start_address_ >> 8) & 0xFF;
  frame[3] = start_address_ & 0xFF;
  frame[4] = 0x00;
  frame[5] = count_;
  uint16_t crc = crc16_(frame, 6);
  frame[6] = crc & 0xFF;
  frame[7] = (crc >> 8) & 0xFF;

  ESP_LOGV(TAG, "TX: %02X %02X %02X %02X %02X %02X %02X %02X",
           frame[0], frame[1], frame[2], frame[3],
           frame[4], frame[5], frame[6], frame[7]);

  // flush RX buffer
  while (this->available() > 0) {
    this->read();
  }

  this->write_array(frame, sizeof(frame));
  this->flush();

  // Expected response: [slave][func=3][byte_count][data...][crc_lo][crc_hi]
  const uint8_t expected_bytes = 3 + count_ * 2 + 2;
  uint8_t resp[256];
  if (expected_bytes > sizeof(resp)) {
    ESP_LOGW(TAG, "Expected bytes %u exceed buffer", expected_bytes);
    return false;
  }

  uint32_t start = millis();
  uint8_t idx = 0;
  while (millis() - start < 200) {  // 200 ms timeout
    if (this->available()) {
      resp[idx++] = this->read();
      if (idx >= expected_bytes) break;
    }
  }

  if (idx < expected_bytes) {
    ESP_LOGW(TAG, "No or incomplete response (%u/%u bytes)", idx, expected_bytes);
    return false;
  }

  ESP_LOGV(TAG, "RX: %s", esphome::format_hex_pretty(resp, idx).c_str());

  // CRC check
  uint16_t resp_crc = (static_cast<uint16_t>(resp[idx - 1]) << 8) | resp[idx - 2];
  uint16_t calc_crc = crc16_(resp, idx - 2);
  if (resp_crc != calc_crc) {
    ESP_LOGW(TAG, "CRC mismatch resp=%04X calc=%04X", resp_crc, calc_crc);
    return false;
  }

  if (resp[0] != slave_id_ || resp[1] != 0x03) {
    ESP_LOGW(TAG, "Invalid response header: slave=%u func=%u", resp[0], resp[1]);
    return false;
  }

  uint8_t byte_count = resp[2];
  if (byte_count != count_ * 2) {
    ESP_LOGW(TAG, "Unexpected byte count %u (expected %u)", byte_count, count_ * 2);
    return false;
  }

  regs.clear();
  regs.reserve(count_);
  for (uint8_t i = 0; i < count_; i++) {
    uint8_t hi = resp[3 + i * 2];
    uint8_t lo = resp[4 + i * 2];
    uint16_t val = (static_cast<uint16_t>(hi) << 8) | lo;
    regs.push_back(val);
  }

  return true;
}

uint16_t CustomModbusBlock::crc16_(uint8_t *buf, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t pos = 0; pos < len; pos++) {
    crc ^= buf[pos];
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

}  // namespace custommodbus
