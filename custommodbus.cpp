#include "custommodbus.h"

namespace custommodbus {

CustomModbus::CustomModbus(esphome::UARTComponent *parent)
    : UARTDevice(parent) {}

void CustomModbus::setup() {
  // Ingen speciel init
}

void CustomModbus::loop() {
  static uint32_t last = 0;
  if (millis() - last < 1000) return;
  last = millis();

  uint8_t frame[8] = {1, 3, 0, 0x10, 0, 1};
  uint16_t crc = crc16(frame, 6);
  frame[6] = crc & 0xFF;
  frame[7] = crc >> 8;

  write_array(frame, 8);
  flush();
  delay(20);

  uint8_t resp[7];
  if (available() >= 7) {
    read_array(resp, 7);

    if (resp[0] == 1 && resp[1] == 3 && resp[2] == 2) {
      uint16_t raw = (resp[3] << 8) | resp[4];
      sensor_->publish_state(raw);
    }
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
