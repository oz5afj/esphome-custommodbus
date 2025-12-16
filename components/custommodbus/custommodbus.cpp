#include "custommodbus.h"

// Platform headers til fuld definition i .cpp
#include "esphome/core/log.h"

namespace esphome {
namespace custommodbus {

static const char *const TAG = "custommodbus";

//
// SETUP
//
void CustomModbus::setup() {
  ESP_LOGCONFIG(TAG, "Setting up CustomModbus with slave ID %u", this->slave_id_);
}

//
// LOOP
//
void CustomModbus::loop() {
  static uint32_t last = 0;
  const uint32_t now = millis();

  if (now - last < 200)
    return;

  last = now;

  this->process_writes();
  this->process_reads();
}

//
// REGISTRERING AF READS
//
void CustomModbus::add_read_sensor(uint16_t reg, uint8_t count, DataType type,
                                   float scale, sensor::Sensor *s) {
  ReadItem item{};
  item.reg = reg;
  item.count = count;
  item.type = type;
  item.scale = scale;
  item.sensor = s;
  item.binary_sensor = nullptr;
  item.text_sensor = nullptr;
  item.bitmask = 0;
  this->reads_.push_back(item);
}

void CustomModbus::add_binary_sensor(uint16_t reg, uint16_t mask,
                                     binary_sensor::BinarySensor *bs) {
  ReadItem item{};
  item.reg = reg;
  item.count = 1;
  item.type = TYPE_UINT16;
  item.scale = 1.0f;
  item.sensor = nullptr;
  item.binary_sensor = bs;
  item.text_sensor = nullptr;
  item.bitmask = mask;
  this->reads_.push_back(item);
}

void CustomModbus::add_text_sensor(uint16_t reg, text_sensor::TextSensor *ts) {
  ReadItem item{};
  item.reg = reg;
  item.count = 1;
  item.type = TYPE_UINT16;
  item.scale = 1.0f;
  item.sensor = nullptr;
  item.binary_sensor = nullptr;
  item.text_sensor = ts;
  item.bitmask = 0;
  this->reads_.push_back(item);
}

//
// QUEUE WRITES
//
void CustomModbus::write_single(uint16_t reg, uint16_t value) {
  WriteItem w{};
  w.reg = reg;
  w.value = value;
  w.mask = 0;
  w.use_mask = false;
  this->writes_.push_back(w);
}

void CustomModbus::write_bitmask(uint16_t reg, uint16_t mask, bool state) {
  WriteItem w{};
  w.reg = reg;
  w.mask = mask;
  w.value = state ? mask : 0;
  w.use_mask = true;
  this->writes_.push_back(w);
}

//
// PROCESS READS
//
void CustomModbus::process_reads() {
  static size_t index = 0;

  if (this->reads_.empty())
    return;

  if (index >= this->reads_.size())
    index = 0;

  auto &r = this->reads_[index];
  index++;

  uint8_t resp[64]{0};
  uint8_t resp_len = 0;

  if (!this->read_registers(r.reg, r.count, resp, resp_len))
    return;

  const uint16_t raw16 = (resp[3] << 8) | resp[4];

  uint32_t raw32 = 0;
  if (r.count == 2) {
    raw32 = (static_cast<uint32_t>(resp[3]) << 24) |
            (static_cast<uint32_t>(resp[4]) << 16) |
            (static_cast<uint32_t>(resp[5]) << 8) |
            static_cast<uint32_t>(resp[6]);
  }

  if (r.sensor != nullptr) {
    float value = 0.0f;

    switch (r.type) {
      case TYPE_UINT16:
        value = static_cast<float>(raw16);
        break;
      case TYPE_INT16:
        value = static_cast<float>(static_cast<int16_t>(raw16));
        break;
      case TYPE_UINT32:
        value = static_cast<float>(raw32);
        break;
      case TYPE_UINT32_R:
        value = static_cast<float>(__builtin_bswap32(raw32));
        break;
    }

    value *= r.scale;
    r.sensor->publish_state(value);
  }

  if (r.binary_sensor != nullptr) {
    const bool state = (raw16 & r.bitmask) != 0;
    r.binary_sensor->publish_state(state);
  }

  if (r.text_sensor != nullptr) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%04X", raw16);
    r.text_sensor->publish_state(buf);
  }
}

//
// PROCESS WRITES
//
void CustomModbus::process_writes() {
  if (this->writes_.empty())
    return;

  const WriteItem w = this->writes_.front();
  this->writes_.erase(this->writes_.begin());

  uint8_t frame[8];
  frame[0] = this->slave_id_;
  frame[1] = 6;  // Write Single Register
  frame[2] = (w.reg >> 8) & 0xFF;
  frame[3] = w.reg & 0xFF;

  uint16_t val = w.value;

  frame[4] = (val >> 8) & 0xFF;
  frame[5] = val & 0xFF;

  uint16_t crc = this->crc16(frame, 6);
  frame[6] = crc & 0xFF;
  frame[7] = crc >> 8;

  // Brug UARTDevice's write_array/flush som er tilgængelige via UARTDevice base
  this->write_array(frame, 8);
  this->flush();
}

//
// READ REGISTERS
//
bool CustomModbus::read_registers(uint16_t reg, uint8_t count, uint8_t *resp, uint8_t &resp_len) {
  uint8_t frame[8];
  frame[0] = this->slave_id_;
  frame[1] = 3;  // Read Holding Registers
  frame[2] = (reg >> 8) & 0xFF;
  frame[3] = reg & 0xFF;
  frame[4] = 0;
  frame[5] = count;

  uint16_t crc = this->crc16(frame, 6);
  frame[6] = crc & 0xFF;
  frame[7] = crc >> 8;

  this->write_array(frame, 8);
  this->flush();

  const uint32_t start = millis();
  const uint8_t expected = static_cast<uint8_t>(5 + count * 2);

  while (millis() - start < 200) {
    int avail = this->available();

    if (avail >= expected) {
      this->read_array(resp, expected);
      resp_len = expected;
      return true;
    }

    if (avail > 0 && avail < expected) {
      ESP_LOGW(TAG, "Partial Modbus response: %d/%d bytes", avail, expected);
    }

    delay(1);
  }

  ESP_LOGW(TAG, "Timeout waiting for Modbus response reg=0x%04X", reg);
  return false;
}

//
// CRC16
//
uint16_t CustomModbus::crc16(uint8_t *buf, uint8_t len) {
  uint16_t crc = 0xFFFF;

  for (uint8_t pos = 0; pos < len; pos++) {
    crc ^= buf[pos];

    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 1)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }

  return crc;
}

}  // namespace custommodbus
}  // namespace esphome
