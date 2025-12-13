#include "custommodbus.h"
#include "esphome/core/log.h"

namespace custommodbus {

static const char *const TAG = "custommodbus";

void CustomModbus::setup() {
  ESP_LOGI(TAG, "CustomModbus setup");
}

void CustomModbus::loop() {
  static uint32_t last = 0;
  uint32_t now = esphome::millis();

  if (now - last < 1000)
    return;
  last = now;

  if (this->sensor_ == nullptr) {
    ESP_LOGW(TAG, "No sensor attached, skipping Modbus request");
    return;
  }

  uint8_t frame[8] = {1, 3, 0, 0x10, 0, 1};
  uint16_t crc = crc16(frame, 6);
  frame[6] = crc & 0xFF;
  frame[7] = crc >> 8;

  // UARTDevice-metoder skal kaldes via this->
  this->write_array(frame, 8);
  this->flush();
  esphome::delay(20);

  uint8_t resp[7];
  if (this->available() >= 7) {
    this->read_array(resp, 7);

    if (resp[0] == 1 && resp[1] == 3 && resp[2] == 2) {
      uint16_t raw = (uint16_t(resp[3]) << 8) | uint16_t(resp[4]);
      ESP_LOGD(TAG, "Received raw value: %u", raw);
      this->sensor_->publish_state(raw);
    } else {
      ESP_LOGW(TAG, "Unexpected Modbus response");
    }
  } else {
    ESP_LOGW(TAG, "Not enough data available on UART");
  }
}

uint16_t CustomModbus::crc16(uint8_t *buf, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t pos = 0; pos < len; pos++) {
    crc ^= buf[pos];
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 1) {
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
