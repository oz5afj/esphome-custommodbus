#include "custommodbus.h"

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/log.h"
#include <algorithm>
#include <cstring>
#include <cstdio>
#include "Arduino.h"  // for pinMode/digitalWrite/millis/delay

namespace esphome {
namespace custommodbus {

static const char *const TAG = "custommodbus";

//
// NOTE about read state variables
// The header provided declares these variables at file scope (outside the class).
// The implementation below uses those globals (read_state_, read_start_ms_, etc.)
// so they must be visible from the header as you provided earlier.
//

//
// SETUP
//
void CustomModbus::setup() {
  ESP_LOGCONFIG(TAG, "Setting up CustomModbus with slave ID %u", this->slave_id_);
  ESP_LOGI(TAG, "uart_parent_ pointer: %p", this->uart_parent_);

  // Ensure the three GPIO pins are driven HIGH so the RS485 board works as you requested.
  // This mirrors the YAML outputs you use (pins 16, 17, 19 must be HIGH).
  // We set them as outputs and drive them HIGH here so the board will function even
  // if the YAML outputs are not yet initialized.
  pinMode(16, OUTPUT);
  digitalWrite(16, HIGH);

  pinMode(17, OUTPUT);
  digitalWrite(17, HIGH);

  pinMode(19, OUTPUT);
  digitalWrite(19, HIGH);

  // Initialize read state globals (they are declared in the header)
  read_state_ = IDLE;
  read_start_ms_ = 0;
  read_expected_ = 0;
  read_got_ = 0;
  read_index_ = 0;
  read_reg_ = 0;
  read_count_ = 0;
  memset(read_buf_, 0, sizeof(read_buf_));
}

//
// LOOP
//
void CustomModbus::loop() {
  static uint32_t last = 0;
  const uint32_t now = millis();

  // Poll interval (ms). Øg hvis bus er langsom eller for at reducere trafik.
  const uint32_t interval = 500;

  if (now - last < interval)
    return;

  last = now;

  // First, if there are pending writes, process them (non-blocking)
  this->process_writes();

  // If we are idle, start the next read (rotate through reads_)
  if (read_state_ == IDLE && !this->reads_.empty()) {
    if (read_index_ >= this->reads_.size())
      read_index_ = 0;

    auto &r = this->reads_[read_index_++];
    // start_read will send the Modbus request and set read_state_ to WAITING
    this->start_read(r.reg, r.count);
    // We return here; the actual response will be handled by handle_read_state()
    // which is called below.
  }

  // Always call handle_read_state to progress the state machine and process any incoming bytes.
  this->handle_read_state();

  // If a response has been received and validated (read_state_ == PROCESSING),
  // parse it and publish to sensors, then set state back to IDLE.
  if (read_state_ == PROCESSING) {
    // We expect the response to be in read_buf_ with length read_expected_
    if (read_expected_ < 5) {
      ESP_LOGW(TAG, "Received too short response %d bytes", read_expected_);
      read_state_ = IDLE;
      return;
    }

    // Example: for 1 register: data starts at read_buf_[3], read_buf_[3]<<8 | read_buf_[4]
    const uint16_t raw16 = (static_cast<uint16_t>(read_buf_[3]) << 8) | read_buf_[4];

    uint32_t raw32 = 0;
    if (read_count_ == 2 && read_expected_ >= 7) {
      raw32 = (static_cast<uint32_t>(read_buf_[3]) << 24) |
              (static_cast<uint32_t>(read_buf_[4]) << 16) |
              (static_cast<uint32_t>(read_buf_[5]) << 8) |
              static_cast<uint32_t>(read_buf_[6]);
    }

    // We need to find which ReadItem this response corresponds to.
    // Because we increment read_index_ after starting the read, the item we started
    // is at index (read_index_ - 1) modulo reads_.size()
    if (!this->reads_.empty()) {
      size_t idx = (read_index_ == 0) ? (this->reads_.size() - 1) : (read_index_ - 1);
      auto &r = this->reads_[idx];

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
          default:
            value = static_cast<float>(raw16);
            break;
        }

        value *= r.scale;
        r.sensor->publish_state(value);
      }

      // Binary/text publishing is disabled at runtime in your header; keep it commented out.
#if 0
      if (r.binary_sensor != nullptr) {
        const bool state = (raw16 & r.bitmask) != 0;
        r.binary_sensor->publish_state(state);
      }

      if (r.text_sensor != nullptr) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%04X", raw16);
        r.text_sensor->publish_state(buf);
      }
#endif
    }

    // Done processing this response
    read_state_ = IDLE;
    read_got_ = 0;
    read_expected_ = 0;
    memset(read_buf_, 0, sizeof(read_buf_));
  }
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

// Wrapper overload: accepterer tal fra auto-genereret main.cpp og caster til DataType
void CustomModbus::add_read_sensor(uint16_t reg, uint8_t count, uint8_t type_as_int, float scale, sensor::Sensor *s) {
  DataType dtype = static_cast<DataType>(type_as_int);
  this->add_read_sensor(reg, count, dtype, scale, s);
}

// Binary/text registration midlertidigt deaktiveret i runtime; deklarationer beholdes for kompatibilitet
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

  // Tøm RX buffer før TX for at undgå gamle bytes/echo
  while (this->uart_parent_->available() > 0) {
    uint8_t tmp;
    this->uart_parent_->read_array(&tmp, 1);
  }

  // Log TX frame (diagnostik)
  ESP_LOGVV(TAG, "TX frame (write):");
  for (int i = 0; i < 8; ++i) ESP_LOGVV(TAG, " %02X", frame[i]);

  this->uart_parent_->write_array(frame, 8);
  this->uart_parent_->flush();

  // Kort pause for at sikre at TX er sendt og slave kan begynde at svare
  delay(2);
}

//
// START READ (non-blocking) - initiates a Modbus read and sets state to WAITING
//
void CustomModbus::start_read(uint16_t reg, uint8_t count) {
  if (this->uart_parent_ == nullptr) {
    ESP_LOGW(TAG, "UART parent not set; cannot start read reg=0x%04X", reg);
    return;
  }

  // Build Modbus RTU request: [slave][func=3][addr_hi][addr_lo][count_hi=0][count_lo][crc_lo][crc_hi]
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

  // Clear RX buffer before sending
  while (this->uart_parent_->available() > 0) {
    uint8_t tmp;
    this->uart_parent_->read_array(&tmp, 1);
  }

  // Log TX frame (diagnostik)
  ESP_LOGVV(TAG, "TX frame (start_read):");
  for (int i = 0; i < 8; ++i) ESP_LOGVV(TAG, " %02X", frame[i]);

  this->uart_parent_->write_array(frame, 8);
  this->uart_parent_->flush();

  // Set up read state machine globals (declared in header)
  read_state_ = WAITING;
  read_start_ms_ = millis();
  read_expected_ = static_cast<uint8_t>(5 + count * 2);  // slave + func + bytecount + data + crc_lo + crc_hi
  read_got_ = 0;
  read_reg_ = reg;
  read_count_ = count;
  memset(read_buf_, 0, sizeof(read_buf_));

  // Short pause to let transceiver switch and slave start responding
  delay(2);
}

//
// HANDLE READ STATE (non-blocking) - called frequently from loop()
// Reads incoming bytes into read_buf_ and validates when enough bytes arrived.
//
void CustomModbus::handle_read_state() {
  if (read_state_ == IDLE)
    return;

  if (this->uart_parent_ == nullptr) {
    ESP_LOGW(TAG, "UART parent not set; cannot handle read state");
    read_state_ = IDLE;
    return;
  }

  // Read available bytes into read_buf_
  int avail = this->uart_parent_->available();
  if (avail > 0) {
    int to_read = std::min(static_cast<int>(sizeof(read_buf_) - read_got_), avail);
    if (to_read > 0) {
      this->uart_parent_->read_array(read_buf_ + read_got_, to_read);
      read_got_ += to_read;

      // Log raw bytes for diagnostics
      ESP_LOGVV(TAG, "RX raw (%d):", read_got_);
      for (uint8_t i = 0; i < read_got_; ++i) ESP_LOGVV(TAG, " %02X", read_buf_[i]);
    }
  }

  // If we have enough bytes, validate frame
  if (read_got_ >= read_expected_) {
    // Validate slave id and function
    if (read_buf_[0] != this->slave_id_ || read_buf_[1] != 3) {
      ESP_LOGW(TAG, "Ignoring frame wrong slave/function %02X %02X", read_buf_[0], read_buf_[1]);
      // Reset and go back to IDLE so next read can be started
      read_state_ = IDLE;
      read_got_ = 0;
      return;
    }

    // Validate CRC (Modbus CRC: low byte first in stream)
    uint16_t recv_crc = (static_cast<uint16_t>(read_buf_[read_expected_ - 1]) << 8) | read_buf_[read_expected_ - 2];
    uint16_t calc_crc = this->crc16(read_buf_, read_expected_ - 2);
    if (recv_crc != calc_crc) {
      ESP_LOGW(TAG, "Ignoring frame CRC mismatch got %04X calc %04X", recv_crc, calc_crc);
      read_state_ = IDLE;
      read_got_ = 0;
      return;
    }

    // Valid frame received: mark for processing by loop/process_reads
    read_state_ = PROCESSING;
    // Note: actual parsing/publishing is done in loop() after handle_read_state returns
    return;
  }

  // Timeout handling
  if (millis() - read_start_ms_ > read_timeout_ms_) {
    ESP_LOGW(TAG, "Timeout waiting for Modbus response reg=0x%04X", read_reg_);
    read_state_ = IDLE;
    read_got_ = 0;
    read_expected_ = 0;
    memset(read_buf_, 0, sizeof(read_buf_));
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
