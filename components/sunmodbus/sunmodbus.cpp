#include "sunmodbus.h"
#include "esphome/core/log.h"

namespace esphome {
namespace sunmodbus {

static const char *const TAG = "sunmodbus_split10_dbg";

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
  ESP_LOGI(TAG, "Setup start");
  ESP_LOGI(TAG, "Slave=%u start=%u", this->slave_id_, this->start_address_);
  ESP_LOGI(TAG, "Setup done");
}

void SunModbus::update() {
  ESP_LOGI(TAG, "ENTER update()");

  uint8_t resp1[5 + 10 + 2] = {0};
  uint8_t resp2[5 + 10 + 2] = {0};

  ESP_LOGI(TAG, "Reading block1 (598–602)");
  if (!this->read_block_(this->slave_id_, this->start_address_, 5, resp1, sizeof(resp1))) {
    ESP_LOGW(TAG, "Block1 failed");
    return;
  }
  ESP_LOGI(TAG, "Block1 OK");

  ESP_LOGI(TAG, "Reading block2 (603–607)");
  if (!this->read_block_(this->slave_id_, this->start_address_ + 5, 5, resp2, sizeof(resp2))) {
    ESP_LOGW(TAG, "Block2 failed");
    return;
  }
  ESP_LOGI(TAG, "Block2 OK");

  ESP_LOGI(TAG, "Publishing block1");
  for (int i = 0; i < 5; i++) {
    auto *s = this->sensors_[i];
    if (!s) continue;

    uint8_t hi = resp1[3 + i * 2];
    uint8_t lo = resp1[3 + i * 2 + 1];
    int16_t raw = (hi << 8) | lo;

    ESP_LOGI(TAG, "reg%d = %d", i, raw);
    s->publish_state(raw);
  }

  ESP_LOGI(TAG, "Publishing block2");
  for (int i = 0; i < 5; i++) {
    auto *s = this->sensors_[i + 5];
    if (!s) continue;

    uint8_t hi = resp2[3 + i * 2];
    uint8_t lo = resp2[3 + i * 2 + 1];
    int16_t raw = (hi << 8) | lo;

    ESP_LOGI(TAG, "reg%d = %d", i + 5, raw);
    s->publish_state(raw);
  }

  ESP_LOGI(TAG, "EXIT update()");
}

bool SunModbus::read_block_(uint8_t slave, uint16_t start, uint8_t count, uint8_t *buffer, uint16_t len) {
  ESP_LOGI(TAG, "ENTER read_block_ start=%u count=%u", start, count);

  uint16_t expected = 5 + count * 2 + 2;
  ESP_LOGI(TAG, "Expected response length: %u", expected);

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

  ESP_LOGI(TAG, "Sending request");
  this->flush();
  this->write_array(req, 8);

  uint32_t timeout = millis() + 150;
  uint16_t idx = 0;

  ESP_LOGI(TAG, "Waiting for response...");
  while (millis() < timeout && idx < expected) {
    int avail = this->available();
    if (avail > 0) {
      int to_read = std::min<int>(avail, expected - idx);
      int r = this->read_array(buffer + idx, to_read);
      idx += r;
      ESP_LOGI(TAG, "Read %d bytes (total %u/%u)", r, idx, expected);
    } else {
      delay(0);
    }
  }

  if (idx < expected) {
    ESP_LOGW(TAG, "Short response: %u/%u", idx, expected);
    return false;
  }

  ESP_LOGI(TAG, "Response complete, checking CRC");
  uint16_t resp_crc = buffer[expected - 2] | (buffer[expected - 1] << 8);
  if (crc16_modbus(buffer, expected - 2) != resp_crc) {
    ESP_LOGW(TAG, "CRC mismatch");
    return false;
  }

  ESP_LOGI(TAG, "CRC OK");
  ESP_LOGI(TAG, "EXIT read_block_");
  return true;
}

}  // namespace sunmodbus
}  // namespace esphome
