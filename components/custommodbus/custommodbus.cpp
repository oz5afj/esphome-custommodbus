#include "custommodbus.h"

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/log.h"
#include <algorithm>
#include <cstring>
#include <cstdio>

namespace esphome {
namespace custommodbus {

static const char *const TAG = "custommodbus";

// Uncomment to enable manual DE/RE control and set DE_PIN accordingly
// #define MANUAL_DE
#ifdef MANUAL_DE
#ifndef DE_PIN
#define DE_PIN 4
#endif
#endif

//
// SETUP
//
void CustomModbus::setup() {
  ESP_LOGCONFIG(TAG, "Setting up CustomModbus with slave ID %u", this->slave_id_);
  ESP_LOGI(TAG, "uart_parent_ pointer: %p", this->uart_parent_);

  // Ensure the three GPIOs required by your hardware are set HIGH
  // (user requested pins 16,17,19 must be HIGH for the board to function)
  pinMode(16, OUTPUT);
  digitalWrite(16, HIGH);
  pinMode(17, OUTPUT);
  digitalWrite(17, HIGH);
  pinMode(19, OUTPUT);
  digitalWrite(19, HIGH);

#ifdef MANUAL_DE
  pinMode(DE_PIN, OUTPUT);
  digitalWrite(DE_PIN, LOW);  // default RX
#endif

  // Initialize read state variables (assumes these are declared in header as members)
  this->read_state_ = IDLE;
  this->read_start_ms_ = 0;
  this->read_expected_ = 0;
  this->read_got_ = 0;
  this->read_index_ = 0;
  this->read_reg_ = 0;
  this->read_count_ = 0;
  this->read_timeout_ms_ = 2000; // 2s timeout for slow inverters
  memset(this->read_buf_, 0, sizeof(this->read_buf_));
}

//
// LOOP
//
void CustomModbus::loop() {
  static uint32_t last = 0;
  const uint32_t now = millis();

  // Poll interval (ms). Øg hvis bus er langsom eller for at reducere trafik.
  const uint32_t interval = 2000;

  // Always process read state (non-blocking)
  this->handle_read_state();

  if (now - last < interval)
    return;

  last = now;

  this->process_writes();

  // Start next read in rotation if idle
  if (this->read_state_ == IDLE && !this->reads_.empty()) {
    if (this->read_index_ >= this->reads_.size())
      this->read_index_ = 0;
    auto &r = this->reads_[this->read_index_++];
    this->start_read(r.reg, r.count);
  }
}

//
// REGISTRATION OF READS
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

void CustomModbus::add_read_sensor(uint16_t reg, uint8_t count, uint8_t type_as_int, float scale, sensor::Sensor *s) {
  DataType dtype = static_cast<DataType>(type_as_int);
  this->add_read_sensor(reg, count, dtype, scale, s);
}

void CustomModbus::add_binary_sensor(uint16_t reg, uint16_t mask, esphome::binary_sensor::BinarySensor *bs) {
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

  if (this->uart_parent_ == nullptr) {
    ESP_LOGW(TAG, "UART parent not set; skipping write_single reg=0x%04X", w.reg);
    return;
  }

  // Flush RX buffer before TX to avoid old bytes/echo
  while (this->uart_parent_->available() > 0) {
    uint8_t tmp;
    this->uart_parent_->read_array(&tmp, 1);
  }

  // Log TX frame (diagnostic)
  ESP_LOGVV(TAG, "TX frame (write):");
  for (int i = 0; i < 8; ++i) ESP_LOGVV(TAG, " %02X", frame[i]);

#ifdef MANUAL_DE
  digitalWrite(DE_PIN, HIGH);
#endif
  this->uart_parent_->write_array(frame, 8);
  this->uart_parent_->flush();
  delay(10); // ensure TX finished
#ifdef MANUAL_DE
  digitalWrite(DE_PIN, LOW);
#endif
}

//
// SYNCHRONOUS READ REGISTERS (fallback)
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

  if (this->uart_parent_ == nullptr) {
    ESP_LOGW(TAG, "UART parent not set; cannot read registers reg=0x%04X", reg);
    return false;
  }

  // Flush RX buffer before sending
  while (this->uart_parent_->available() > 0) {
    uint8_t tmp;
    this->uart_parent_->read_array(&tmp, 1);
  }

  // Log TX frame (diagnostic)
  ESP_LOGVV(TAG, "TX frame (read):");
  for (int i = 0; i < 8; ++i) ESP_LOGVV(TAG, " %02X", frame[i]);

#ifdef MANUAL_DE
  digitalWrite(DE_PIN, HIGH);
#endif
  this->uart_parent_->write_array(frame, 8);
  this->uart_parent_->flush();
  delay(10);
#ifdef MANUAL_DE
  digitalWrite(DE_PIN, LOW);
#endif

  const uint32_t start = millis();
  const uint8_t expected = static_cast<uint8_t>(5 + count * 2);
  uint8_t buf[64];
  uint8_t got = 0;

  while (millis() - start < this->read_timeout_ms_) {
    int avail = this->uart_parent_->available();
    if (avail > 0) {
      int to_read = std::min(static_cast<int>(sizeof(buf) - got), avail);
      if (to_read > 0) {
        this->uart_parent_->read_array(buf + got, to_read);
        got += to_read;

        ESP_LOGVV(TAG, "RX raw (%d):", got);
        for (uint8_t i = 0; i < got; ++i) ESP_LOGVV(TAG, " %02X", buf[i]);
      }
    }

    if (got >= expected) {
      // Validate slave id and function
      if (buf[0] != this->slave_id_ || buf[1] != 3) {
        ESP_LOGW(TAG, "Ignoring frame wrong slave/function %02X %02X", buf[0], buf[1]);
        return false;
      }
      // Validate CRC (Modbus CRC: low byte first)
      uint16_t recv_crc = (static_cast<uint16_t>(buf[expected - 1]) << 8) | buf[expected - 2];
      uint16_t calc_crc = this->crc16(buf, expected - 2);
      if (recv_crc != calc_crc) {
        ESP_LOGW(TAG, "Ignoring frame CRC mismatch got %04X calc %04X", recv_crc, calc_crc);
        return false;
      }

      memcpy(resp, buf, expected);
      resp_len = expected;
      return true;
    }

    if (got > 0 && got < expected) {
      ESP_LOGW(TAG, "Partial Modbus response: %d/%d bytes", got, expected);
    }

    delay(1);
  }

  ESP_LOGW(TAG, "Timeout waiting for Modbus response reg=0x%04X", reg);
  return false;
}

//
// ASYNC READ FLOW: start_read + handle_read_state (medlemsfunktioner)
//
void CustomModbus::start_read(uint16_t reg, uint8_t count) {
  if (this->uart_parent_ == nullptr) return;

  uint8_t frame[8];
  frame[0] = this->slave_id_;
  frame[1] = 3;
  frame[2] = (reg >> 8) & 0xFF;
  frame[3] = reg & 0xFF;
  frame[4] = 0;
  frame[5] = count;
  uint16_t crc = this->crc16(frame, 6);
  frame[6] = crc & 0xFF;
  frame[7] = crc >> 8;

  // Flush RX before TX
  while (this->uart_parent_->available() > 0) {
    uint8_t tmp; this->uart_parent_->read_array(&tmp, 1);
  }

  // Log TX
  ESP_LOGVV(TAG, "TX frame (start_read):");
  for (int i = 0; i < 8; ++i) ESP_LOGVV(TAG, " %02X", frame[i]);

#ifdef MANUAL_DE
  digitalWrite(DE_PIN, HIGH);
#endif
  this->uart_parent_->write_array(frame, 8);
  this->uart_parent_->flush();
  delay(10);
#ifdef MANUAL_DE
  digitalWrite(DE_PIN, LOW);
#endif

  // Save metadata for async read
  this->read_state_ = WAITING;
  this->read_start_ms_ = millis();
  this->read_expected_ = static_cast<uint8_t>(5 + count * 2);
  this->read_got_ = 0;
  this->read_reg_ = reg;
  this->read_count_ = count;
  memset(this->read_buf_, 0, sizeof(this->read_buf_));
}

void CustomModbus::handle_read_state() {
  if (this->read_state_ == IDLE) return;
  if (this->uart_parent_ == nullptr) { this->read_state_ = IDLE; return; }

  uint32_t now = millis();

  // Accumulate available bytes
  int avail = this->uart_parent_->available();
  if (avail > 0) {
    int to_read = std::min(static_cast<int>(sizeof(this->read_buf_) - this->read_got_), avail);
    if (to_read > 0) {
      this->uart_parent_->read_array(this->read_buf_ + this->read_got_, to_read);
      this->read_got_ += to_read;
    }
  }

  // If we have any bytes, try to find a valid Modbus start (slave + function)
  if (this->read_got_ > 1) {
    int start = -1;
    for (int i = 0; i + 1 < this->read_got_; ++i) {
      if (this->read_buf_[i] == this->slave_id_ && this->read_buf_[i + 1] == 0x03) {
        start = i;
        break;
      }
    }

    if (start < 0) {
      // No valid start found yet. Prevent buffer overflow by keeping last bytes.
      if (this->read_got_ > 48) {
        // keep last 16 bytes
        memmove(this->read_buf_, this->read_buf_ + this->read_got_ - 16, 16);
        this->read_got_ = 16;
      }
      return;
    }

    // If start > 0, shift buffer so frame starts at index 0
    if (start > 0) {
      memmove(this->read_buf_, this->read_buf_ + start, this->read_got_ - start);
      this->read_got_ -= start;
    }

    // Now check if we have expected bytes
    if (this->read_got_ >= this->read_expected_) {
      uint8_t *buf = this->read_buf_;
      uint8_t expected = this->read_expected_;

      // Validate slave id + function (should be true by search, but double-check)
      if (buf[0] != this->slave_id_ || buf[1] != 3) {
        ESP_LOGW(TAG, "Ignoring frame wrong slave/function %02X %02X", buf[0], buf[1]);
        this->read_state_ = IDLE;
        return;
      }

      // Validate CRC (Modbus: low byte first)
      uint16_t recv_crc = (static_cast<uint16_t>(buf[expected - 1]) << 8) | buf[expected - 2];
      uint16_t calc_crc = this->crc16(buf, expected - 2);
      if (recv_crc != calc_crc) {
        ESP_LOGW(TAG, "Ignoring frame CRC mismatch got %04X calc %04X", recv_crc, calc_crc);
        // drop the first byte and try again next loop (resync)
        memmove(this->read_buf_, this->read_buf_ + 1, --this->read_got_);
        return;
      }

      // Copy to local resp and publish to matching read item
      uint8_t resp_len = expected;
      uint8_t resp[64];
      memcpy(resp, buf, expected);

      // Log the successful raw frame at verbose level
      ESP_LOGVV(TAG, "RX raw (%d):", resp_len);
      for (uint8_t i = 0; i < resp_len; ++i) ESP_LOGVV(TAG, " %02X", resp[i]);

      // Find read item with matching reg and count (we started this->read_reg_/read_count_)
      for (auto &r : this->reads_) {
        if (r.reg == this->read_reg_ && r.count == this->read_count_) {
          const uint16_t raw16 = (static_cast<uint16_t>(resp[3]) << 8) | resp[4];
          uint32_t raw32 = 0;
          if (r.count == 2 && resp_len >= 7) {
            raw32 = (static_cast<uint32_t>(resp[3]) << 24) |
                    (static_cast<uint32_t>(resp[4]) << 16) |
                    (static_cast<uint32_t>(resp[5]) << 8) |
                    static_cast<uint32_t>(resp[6]);
          }

          if (r.sensor != nullptr) {
            float value = 0.0f;
            switch (r.type) {
              case TYPE_UINT16: value = static_cast<float>(raw16); break;
              case TYPE_INT16: value = static_cast<float>(static_cast<int16_t>(raw16)); break;
              case TYPE_UINT32: value = static_cast<float>(raw32); break;
              case TYPE_UINT32_R: value = static_cast<float>(__builtin_bswap32(raw32)); break;
              default: value = static_cast<float>(raw16); break;
            }
            value *= r.scale;
            r.sensor->publish_state(value);
          }
          break;
        }
      }

      // Reset state and clear buffer
      this->read_state_ = IDLE;
      this->read_got_ = 0;
      memset(this->read_buf_, 0, sizeof(this->read_buf_));
      return;
    }
  }

  // Timeout handling
  if (now - this->read_start_ms_ > this->read_timeout_ms_) {
    ESP_LOGW(TAG, "Timeout waiting for Modbus response reg=0x%04X", this->read_reg_);
    this->read_state_ = IDLE;
    this->read_got_ = 0;
    memset(this->read_buf_, 0, sizeof(this->read_buf_));
    return;
  }
}

//
// CRC16 (Modbus)
//
uint16_t CustomModbus::crc16(uint8_t *buf, uint8_t len) {
  uint16_t crc = 0xFFFF;

  for (uint8_t pos = 0; pos < len; pos++) {
    crc ^= static_cast<uint16_t>(buf[pos]);

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
