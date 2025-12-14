#include "custommodbus.h"
#include "esphome/core/log.h"

namespace custommodbus {

static const char *const TAG = "custommodbus";

void CustomModbus::add_read_sensor(uint16_t reg, uint8_t count, DataType type,
                                   float scale, esphome::sensor::Sensor *s) {
  ReadItem item{};
  item.reg = reg;
  item.count = count;
  item.type = type;
  item.scale = scale;
  item.sensor = s;
  item.binary_sensor = nullptr;
  item.text_sensor = nullptr;
  item.bitmask = 0;
  reads_.push_back(item);
}

void CustomModbus::add_binary_sensor(uint16_t reg, uint16_t mask,
                                     esphome::binary_sensor::BinarySensor *bs) {
  ReadItem item{};
  item.reg = reg;
  item.count = 1;
  item.type = TYPE_UINT16;
  item.scale = 1.0f;
  item.sensor = nullptr;
  item.binary_sensor = bs;
  item.text_sensor = nullptr;
  item.bitmask = mask;
  reads_.push_back(item);
}

void CustomModbus::add_text_sensor(uint16_t reg, esphome::text_sensor::TextSensor *ts) {
  ReadItem item{};
  item.reg = reg;
  item.count = 1;
  item.type = TYPE_UINT16;
  item.scale = 1.0f;
  item.sensor = nullptr;
  item.binary_sensor = nullptr;
  item.text_sensor = ts;
  item.bitmask = 0;
  reads_.push_back(item);
}

void CustomModbus::write_single(uint16_t reg, uint16_t value) {
  WriteItem w{};
  w.reg = reg;
  w.value = value;
  w.use_mask = false;
  writes_.push_back(w);
}

void CustomModbus::write_bitmask(uint16_t reg, uint16_t mask, bool state) {
  WriteItem w{};
  w.reg = reg;
  w.mask = mask;
  w.value = state ? mask : 0;
  w.use_mask = true;
  writes_.push_back(w);
}

void CustomModbus::loop() {
  static uint32_t last = 0;
  uint32_t now = esphome::millis();
  if (now - last < 200) return;
  last = now;

  process_writes();
  process_reads();
}

void CustomModbus::process_reads() {
  for (auto &r : reads_) {
    uint8_t resp[16];
    uint8_t len = 0;

    if (!read_registers(r.reg, r.count, resp, len))
      continue;

    uint16_t raw16 = (resp[3] << 8) | resp[4];
    uint32_t raw32 = 0;

    if (r.count == 2) {
      raw32 = (resp[3] << 24) | (resp[4] << 16) | (resp[5] << 8) | resp[6];
    }

    if (r.sensor) {
      float value = 0;

      switch (r.type) {
        case TYPE_UINT16: value = raw16; break;
        case TYPE_INT16: value = (int16_t) raw16; break;
        case TYPE_UINT32: value = raw32; break;
        case TYPE_UINT32_R: value = __builtin_bswap32(raw32); break;
      }

      value *= r.scale;
      r.sensor->publish_state(value);
    }

    if (r.binary_sensor) {
      bool state = raw16 & r.bitmask;
      r.binary_sensor->publish_state(state);
    }

    if (r.text_sensor) {
      char buf[16];
      sprintf(buf, "%04X", raw16);
      r.text_sensor->publish_state(buf);
    }
  }
}

void CustomModbus::process_writes() {
  if (writes_.empty()) return;

  WriteItem w = writes_.front();
  writes_.erase(writes_.begin());

  uint8_t frame[8];
  frame[0] = slave_id_;
  frame[1] = 6;  // Write Single Register
  frame[2] = (w.reg >> 8) & 0xFF;
  frame[3] = w.reg & 0xFF;

  uint16_t val = w.value;
  frame[4] = (val >> 8) & 0xFF;
  frame[5] = val & 0xFF;

  uint16_t crc = crc16(frame, 6);
  frame[6] = crc & 0xFF;
  frame[7] = crc >> 8;

  this->write_array(frame, 8);
  this->flush();
}

bool CustomModbus::read_registers(uint16_t reg, uint8_t count,
                                  uint8_t *resp, uint8_t &resp_len) {
  uint8_t frame[8];
  frame[0] = slave_id_;
  frame[1] = 3;
  frame[2] = (reg >> 8) & 0xFF;
  frame[3] = reg & 0xFF;
  frame[4] = 0;
  frame[5] = count;

  uint16_t crc = crc16(frame, 6);
  frame[6] = crc & 0xFF;
  frame[7] = crc >> 8;

  this->write_array(frame, 8);
  this->flush();
  esphome::delay(20);

  int needed = 5 + count * 2;
  if (this->available() < needed) {
    ESP_LOGW(TAG, "No response");
    return false;
  }

  this->read_array(resp, needed);
  resp_len = needed;

  return true;
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
