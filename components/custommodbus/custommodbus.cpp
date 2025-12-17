#include "custommodbus.h"

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/log.h"
#include "Arduino.h"
#include <algorithm>
#include <cstring>
#include <cstdio>

namespace esphome {
namespace custommodbus {

static const char *const TAG = "custommodbus";

//
// NOTE about read-state storage
// The header you provided contains a block of read-state variables. To avoid
// ODR / multiple-definition issues and to keep the implementation self-contained,
// we keep the runtime state here as static variables inside the namespace.
// This matches the behaviour expected by the .cpp implementation below.
//

enum ReadState { IDLE, WAITING, PROCESSING };
static ReadState read_state_ = IDLE;
static uint32_t read_start_ms_ = 0;
static uint32_t read_timeout_ms_ = 1000;  // 1s timeout
static uint8_t read_expected_ = 0;
static uint8_t read_buf_[64];
static uint8_t read_got_ = 0;
static uint16_t read_reg_ = 0;
static uint8_t read_count_ = 0;
static size_t read_index_ = 0;  // rotation index for reads_

//
// SETUP
//
void CustomModbus::setup() {
  ESP_LOGCONFIG(TAG, "Setting up CustomModbus with slave ID %u", this->slave_id_);
  ESP_LOGI(TAG, "uart_parent_ pointer: %p", this->uart_parent_);

  // Ensure the three GPIO pins are driven HIGH as requested by the user.
  // These pins are configured in YAML as binary outputs in the user's config,
  // but we set them high here as a safety measure so the RS485 board works.
  // Use Arduino API for direct pin control.
  pinMode(16, OUTPUT);
  digitalWrite(16, HIGH);

  pinMode(17, OUTPUT);
  digitalWrite(17, HIGH);

  pinMode(19, OUTPUT);
  digitalWrite(19, HIGH);

  // Initialize read-state
  read_state_ = IDLE;
  read_start_ms_ = 0;
  read_expected_ = 0;
  read_got_ = 0;
  read_index_ = 0;
}

//
// LOOP
//
void CustomModbus::loop() {
  // Keep loop light; rotate reads and handle state machine frequently.
  // We still throttle overall polling to avoid saturating the bus.
  static uint32_t last = 0;
  const uint32_t now = millis();

  // Poll interval (ms). Øg hvis bus er langsom eller for at reducere trafik.
  const uint32_t interval = 500;

  if (now - last < interval)
    return;

  last = now;

  // If there are pending writes, process one write first (FIFO).
  this->process_writes();

  // If no active read in progress, start next read from rotation
  if (read_state_ == IDLE && !this->reads_.empty()) {
    if (read_index_ >= this->reads_.size())
      read_index_ = 0;
    auto &r = this->reads_[read_index_++];
    this->start_read(r.reg, r.count);
  }

  // Drive the read-state machine to completion or timeout
  this->handle_read_state();

  // Note: process_reads() kept for backward compatibility; it can be a no-op
  // or used by older code paths. Our state machine handles reads now.
  // this->process_reads();
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
// START A READ (state machine entry)
//
void CustomModbus::start_read(uint16_t reg, uint8_t count) {
  if (this->uart_parent_ == nullptr) {
    ESP_LOGW(TAG, "UART parent not set; cannot start read reg=0x%04X", reg);
    return;
  }

  // Build Modbus Read Holding Registers frame
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

  // Clear any old RX bytes
  while (this->uart_parent_->available() > 0) {
    uint8_t tmp;
    this->uart_parent_->read_array(&tmp, 1);
  }

  // Log TX frame (diagnostik)
  ESP_LOGVV(TAG, "TX frame (start_read):");
  for (int i = 0; i < 8; ++i) ESP_LOGVV(TAG, " %02X", frame[i]);

  this->uart_parent_->write_array(frame, 8);
  this->uart_parent_->flush();

  // Prepare state machine
  read_state_ = WAITING;
  read_start_ms_ = millis();
  read_expected_ = static_cast<uint8_t>(5 + count * 2);  // slave + func + bytecount + data + crc_lo + crc_hi
  read_got_ = 0;
  read_reg_ = reg;
  read_count_ = count;

  // Small delay to let slave start responding (tunable)
  delay(2);
}

//
// HANDLE READ STATE MACHINE
//
void CustomModbus::handle_read_state() {
  if (read_state_ == IDLE)
    return;

  if (this->uart_parent_ == nullptr) {
    ESP_LOGW(TAG, "UART parent not set; aborting read state");
    read_state_ = IDLE;
    return;
  }

  // Read available bytes into buffer
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

  // If we've got enough bytes, validate and process
  if (read_got_ >= read_expected_) {
    // Validate slave id and function
    if (read_buf_[0] != this->slave_id_ || read_buf_[1] != 3) {
      ESP_LOGW(TAG, "Ignoring frame wrong slave/function %02X %02X", read_buf_[0], read_buf_[1]);
      // Reset state and discard buffer
      read_state_ = IDLE;
      read_got_ = 0;
      return;
    }

    // Validate CRC (Modbus CRC: low byte first in frame)
    uint16_t recv_crc = (static_cast<uint16_t>(read_buf_[read_expected_ - 1]) << 8) | read_buf_[read_expected_ - 2];
    uint16_t calc_crc = this->crc16(read_buf_, read_expected_ - 2);
    if (recv_crc != calc_crc) {
      ESP_LOGW(TAG, "Ignoring frame CRC mismatch got %04X calc %04X", recv_crc, calc_crc);
      read_state_ = IDLE;
      read_got_ = 0;
      return;
    }

    // We have a valid response; extract data and publish to sensors
    // For 1 register: data starts at read_buf_[3] (big-endian)
    const uint16_t raw16 = (static_cast<uint16_t>(read_buf_[3]) << 8) | read_buf_[4];

    uint32_t raw32 = 0;
    if (read_count_ == 2 && read_got_ >= 7) {
      raw32 = (static_cast<uint32_t>(read_buf_[3]) << 24) |
              (static_cast<uint32_t>(read_buf_[4]) << 16) |
              (static_cast<uint32_t>(read_buf_[5]) << 8) |
              static_cast<uint32_t>(read_buf_[6]);
    }

    // Find the ReadItem that matches the reg/count we requested.
    // There may be multiple registrations for the same register; publish to all matching.
    for (auto &r : this->reads_) {
      if (r.reg == read_reg_ && r.count == read_count_) {
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

        // Binary/text publishing currently disabled at runtime (kept for compatibility)
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
    }

    // Done processing
    read_state_ = IDLE;
    read_got_ = 0;
    return;
  }

  // Timeout handling
  if (millis() - read_start_ms_ > read_timeout_ms_) {
    ESP_LOGW(TAG, "Timeout waiting for Modbus response reg=0x%04X", read_reg_);
    read_state_ = IDLE;
    read_got_ = 0;
    return;
  }
}

//
// Legacy process_reads kept for compatibility (no-op since we use state machine)
//
void CustomModbus::process_reads() {
  // This implementation uses start_read() + handle_read_state() state machine.
  // Keep this function empty to avoid duplicate reads. If you prefer the
  // older blocking read_registers() approach, you can re-enable it here.
  (void)0;
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
