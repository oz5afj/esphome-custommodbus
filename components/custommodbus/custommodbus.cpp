#include "custommodbus.h"
#include "esphome/core/log.h"

namespace custommodbus {

static const char *const TAG = "custommodbus";

void CustomModbus::loop() {
  static uint32_t last = 0;
  uint32_t now = esphome::millis();
  if (now - last < 1000) return;
  last = now;

  if (sensor_ == nullptr) {
    ESP_LOGW(TAG, "No sensor attached");
    return;
  }

  uint8_t frame[8];
  frame[0] = slave_id_;
  frame[1] = 3;  // Read Holding Registers
  frame[2] = (register_ >> 8) & 0xFF;
  frame[3] = register_ & 0xFF;
  frame[4] = (count_ >> 8) & 0xFF;
  frame[5] = count_ & 0xFF;

  uint16_t crc = crc16(frame, 6);
  frame[6] = crc & 0xFF;
  frame[7] = crc >> 8;

  this->write_array(frame, 8);
  this->flush();
  esphome::delay(20);

  if (this->available() < 7) {
    ESP_LOGW(TAG, "No response");
    return;
  }

  uint8_t resp[7];
  this->read_array(resp, 7);

  if (resp[0] != slave_id_ || resp[1] != 3) {
    ESP_LOGW(TAG, "Invalid response");
    return;
  }

  uint16_t raw = (resp[3] << 8) | resp[4];
  float value = 0;

  switch (data_type_) {
    case TYPE_UINT16:
      value = raw;
      break;

    case TYPE_INT16:
      value = (int16_t) raw;
      break;
  }

  value *= scale_;
  sensor_->publish_state(value);
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
