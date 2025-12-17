#include "custommodbus.h"
#include "esphome/core/log.h"

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace custommodbus {

static const char *TAG = "custommodbus";

// --- Helper: CRC16 (Modbus) ---
uint16_t CustomModbus::crc16(uint8_t *buf, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t pos = 0; pos < len; pos++) {
    crc ^= static_cast<uint16_t>(buf[pos]);
    for (int i = 0; i < 8; i++) {
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

// --- Public API: add/read/write registrations ---
void CustomModbus::add_read_sensor(uint16_t reg, uint8_t count, DataType type, float scale, esphome::sensor::Sensor *s) {
  ReadItem it;
  it.reg = reg;
  it.count = count;
  it.type = type;
  it.scale = scale;
  it.sensor = s;
  it.binary_sensor = nullptr;
  it.text_sensor = nullptr;
  it.bitmask = 0;
  this->reads_.push_back(it);
}

void CustomModbus::add_read_sensor(uint16_t reg, uint8_t count, uint8_t type_as_int, float scale, esphome::sensor::Sensor *s) {
  DataType t = static_cast<DataType>(type_as_int);
  this->add_read_sensor(reg, count, t, scale, s);
}

void CustomModbus::add_binary_sensor(uint16_t reg, uint16_t mask, esphome::binary_sensor::BinarySensor *bs) {
  ReadItem it;
  it.reg = reg;
  it.count = 1;
  it.type = TYPE_UINT16;
  it.scale = 1.0f;
  it.sensor = nullptr;
  it.binary_sensor = bs;
  it.text_sensor = nullptr;
  it.bitmask = mask;
  this->reads_.push_back(it);
}

void CustomModbus::add_text_sensor(uint16_t reg, esphome::text_sensor::TextSensor *ts) {
  ReadItem it;
  it.reg = reg;
  it.count = 1;
  it.type = TYPE_UINT16;
  it.scale = 1.0f;
  it.sensor = nullptr;
  it.binary_sensor = nullptr;
  it.text_sensor = ts;
  it.bitmask = 0;
  this->reads_.push_back(it);
}

void CustomModbus::write_single(uint16_t reg, uint16_t value) {
  WriteItem w;
  w.reg = reg;
  w.value = value;
  w.use_mask = false;
  w.mask = 0;
  this->writes_.push_back(w);
}

void CustomModbus::write_bitmask(uint16_t reg, uint16_t mask, bool state) {
  WriteItem w;
  w.reg = reg;
  w.mask = mask;
  w.use_mask = true;
  w.value = state ? mask : 0;
  this->writes_.push_back(w);
}

// --- Lifecycle ---
void CustomModbus::setup() {
  // Ensure required GPIOs are driven HIGH so board works (user requested)
  pinMode(16, OUTPUT);
  digitalWrite(16, HIGH);
  pinMode(17, OUTPUT);
  digitalWrite(17, HIGH);
  pinMode(19, OUTPUT);
  digitalWrite(19, HIGH);

  // Initialize state machine
  read_state_ = IDLE;
  read_start_ms_ = 0;
  read_timeout_ms_ = 1000;
  read_expected_ = 0;
  read_got_ = 0;
  read_reg_ = 0;
  read_count_ = 0;
  read_index_ = 0;

  // Nothing else to do here; uart_parent_ is set by set_uart_parent from ESPHome
  ESP_LOGI(TAG, "CustomModbus setup complete (slave id=%u)", this->slave_id_);
}

void CustomModbus::loop() {
  // Process incoming bytes / state machine
  this->process_reads();

  // Process any pending writes (non-blocking)
  this->process_writes();
}

// --- Internal: send a Modbus read request (function 0x03) ---
bool CustomModbus::read_registers(uint16_t reg, uint8_t count, uint8_t *resp, uint8_t &resp_len) {
  // Build request: [slave][func=3][addr_hi][addr_lo][count_hi][count_lo][crc_lo][crc_hi]
  uint8_t frame[8];
  frame[0] = this->slave_id_;
  frame[1] = 0x03;
  frame[2] = static_cast<uint8_t>((reg >> 8) & 0xFF);
  frame[3] = static_cast<uint8_t>(reg & 0xFF);
  frame[4] = static_cast<uint8_t>((count >> 8) & 0xFF);
  frame[5] = static_cast<uint8_t>(count & 0xFF);
  uint16_t crc = this->crc16(frame, 6);
  frame[6] = static_cast<uint8_t>(crc & 0xFF);
  frame[7] = static_cast<uint8_t>((crc >> 8) & 0xFF);

  if (!this->uart_parent_) {
    ESP_LOGW(TAG, "UART parent not set, cannot send Modbus request");
    return false;
  }

  // Transmit
  this->uart_parent_->write_array(frame, 8);
  this->uart_parent_->flush();

  // We don't block here; caller will handle waiting/reading via state machine
  (void)resp;
  (void)resp_len;
  return true;
}

// --- Internal: process reads state machine ---
void CustomModbus::process_reads() {
  if (!this->uart_parent_) return;

  // If idle, start a new read if we have configured reads
  if (read_state_ == IDLE) {
    if (this->reads_.empty()) return;

    // rotate through reads_ to spread queries
    if (read_index_ >= this->reads_.size()) read_index_ = 0;
    const ReadItem &r = this->reads_[read_index_];

    // send request for this read
    uint16_t reg = r.reg;
    uint8_t count = r.count;
    // prepare state
    read_state_ = WAITING;
    read_start_ms_ = millis();
    read_expected_ = static_cast<uint8_t>(5 + count * 2); // slave + func + bytecount + data + crc_lo + crc_hi
    read_got_ = 0;
    read_reg_ = reg;
    read_count_ = count;

    // send request
    uint8_t frame[8];
    frame[0] = this->slave_id_;
    frame[1] = 0x03;
    frame[2] = static_cast<uint8_t>((reg >> 8) & 0xFF);
    frame[3] = static_cast<uint8_t>(reg & 0xFF);
    frame[4] = 0x00;
    frame[5] = count;
    uint16_t crc = this->crc16(frame, 6);
    frame[6] = static_cast<uint8_t>(crc & 0xFF);
    frame[7] = static_cast<uint8_t>((crc >> 8) & 0xFF);

    this->uart_parent_->write_array(frame, 8);
    this->uart_parent_->flush();

    // advance rotation index for next time
    read_index_++;
    if (read_index_ >= this->reads_.size()) read_index_ = 0;

    return;
  }

  // WAITING: read available bytes
  if (read_state_ == WAITING) {
    size_t avail = this->uart_parent_->available();
    if (avail > 0) {
      // read up to buffer capacity
      int to_read = std::min(static_cast<int>(sizeof(read_buf_) - read_got_), static_cast<int>(avail));
      if (to_read > 0) {
        this->uart_parent_->read_array(this->read_buf_ + read_got_, to_read);
        read_got_ += to_read;
      }
    }

    // If we've got expected bytes, validate and process
    if (read_got_ >= read_expected_) {
      // Basic checks: slave id and function
      if (read_buf_[0] != this->slave_id_ || read_buf_[1] != 0x03) {
        ESP_LOGW(TAG, "Ignoring frame wrong slave/function %02X %02X", read_buf_[0], read_buf_[1]);
        // reset and continue
        read_state_ = IDLE;
        read_got_ = 0;
        return;
      }

      // CRC check
      uint16_t recv_crc = (static_cast<uint16_t>(read_buf_[read_expected_ - 1]) << 8) | read_buf_[read_expected_ - 2];
      uint16_t calc_crc = this->crc16(read_buf_, read_expected_ - 2);
      if (recv_crc != calc_crc) {
        ESP_LOGW(TAG, "CRC mismatch (recv=0x%04X calc=0x%04X) reg=0x%04X", recv_crc, calc_crc, read_reg_);
        read_state_ = IDLE;
        read_got_ = 0;
        return;
      }

      // Extract byte count and data
      uint8_t bytecount = read_buf_[2];
      const uint8_t *data = &read_buf_[3];

      // Find matching read item (reg & count)
      bool matched = false;
      for (auto &it : this->reads_) {
        if (it.reg == read_reg_ && it.count == read_count_) {
          matched = true;
          // handle binary/text sensors first
          if (it.binary_sensor) {
            // read first register as uint16
            uint16_t val = (static_cast<uint16_t>(data[0]) << 8) | data[1];
            bool state = (val & it.bitmask) != 0;
            it.binary_sensor->publish_state(state);
          } else if (it.text_sensor) {
            // publish raw hex or decimal string of first register
            uint16_t val = (static_cast<uint16_t>(data[0]) << 8) | data[1];
            char buf[16];
            snprintf(buf, sizeof(buf), "%u", val);
            it.text_sensor->publish_state(std::string(buf));
          } else if (it.sensor) {
            // numeric sensor: support types
            if (it.type == TYPE_UINT16) {
              uint16_t v = (static_cast<uint16_t>(data[0]) << 8) | data[1];
              it.sensor->publish_state(static_cast<float>(v) * it.scale);
            } else if (it.type == TYPE_INT16) {
              int16_t v = (static_cast<int16_t>(data[0]) << 8) | data[1];
              it.sensor->publish_state(static_cast<float>(v) * it.scale);
            } else if (it.type == TYPE_UINT32) {
              // big-endian: reg0 high word, reg1 low word
              if (bytecount >= 4) {
                uint32_t hi = (static_cast<uint32_t>(data[0]) << 8) | data[1];
                uint32_t lo = (static_cast<uint32_t>(data[2]) << 8) | data[3];
                uint32_t v = (hi << 16) | lo;
                it.sensor->publish_state(static_cast<float>(v) * it.scale);
              }
            } else if (it.type == TYPE_UINT32_R) {
              // reversed 32-bit: reg0 low word, reg1 high word
              if (bytecount >= 4) {
                uint32_t lo = (static_cast<uint32_t>(data[0]) << 8) | data[1];
                uint32_t hi = (static_cast<uint32_t>(data[2]) << 8) | data[3];
                uint32_t v = (lo << 16) | hi;
                it.sensor->publish_state(static_cast<float>(v) * it.scale);
              }
            }
          }
          break;
        }
      }

      if (!matched) {
        ESP_LOGW(TAG, "Received response for reg=0x%04X count=%u but no matching read item", read_reg_, read_count_);
      }

      // Reset state for next read
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
}

// --- Internal: process pending writes (simple implementation) ---
void CustomModbus::process_writes() {
  if (!this->uart_parent_) return;
  if (this->writes_.empty()) return;

  // Pop and send one write per loop to avoid blocking
  WriteItem w = this->writes_.front();
  this->writes_.erase(this->writes_.begin());

  // Build Modbus write single register (0x06) or write with mask (0x16/0x05 not standardized)
  // We'll implement write single (0x06) for simple writes and masked writes as read-modify-write if needed.
  if (!w.use_mask) {
    uint8_t frame[8];
    frame[0] = this->slave_id_;
    frame[1] = 0x06; // write single register
    frame[2] = static_cast<uint8_t>((w.reg >> 8) & 0xFF);
    frame[3] = static_cast<uint8_t>(w.reg & 0xFF);
    frame[4] = static_cast<uint8_t>((w.value >> 8) & 0xFF);
    frame[5] = static_cast<uint8_t>(w.value & 0xFF);
    uint16_t crc = this->crc16(frame, 6);
    frame[6] = static_cast<uint8_t>(crc & 0xFF);
    frame[7] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    this->uart_parent_->write_array(frame, 8);
    this->uart_parent_->flush();
    // We do not wait for response here; response will be handled by process_reads if it arrives.
  } else {
    // Masked write: read current register, apply mask, write back
    // Synchronous read/write would be more robust; here we queue a read and then a write.
    // For simplicity, perform a read now and then write single with masked value (blocking read with timeout).
    // NOTE: keep this short to avoid blocking main loop too long.
    uint8_t resp[64];
    uint8_t resp_len = 0;
    // Try to read register directly (blocking-ish with small timeout)
    // Build and send read request
    uint8_t req[8];
    req[0] = this->slave_id_;
    req[1] = 0x03;
    req[2] = static_cast<uint8_t>((w.reg >> 8) & 0xFF);
    req[3] = static_cast<uint8_t>(w.reg & 0xFF);
    req[4] = 0x00;
    req[5] = 0x01;
    uint16_t crc = this->crc16(req, 6);
    req[6] = static_cast<uint8_t>(crc & 0xFF);
    req[7] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    this->uart_parent_->write_array(req, 8);
    this->uart_parent_->flush();

    // small wait loop for response (non-blocking but with short timeout)
    uint32_t start = millis();
    uint8_t got = 0;
    while (millis() - start < 200) {
      size_t avail = this->uart_parent_->available();
      if (avail) {
        int to_read = std::min(static_cast<int>(sizeof(resp) - got), static_cast<int>(avail));
        this->uart_parent_->read_array(resp + got, to_read);
        got += to_read;
        // minimal validation
        if (got >= 7) break;
      }
      delay(2);
    }
    if (got >= 7) {
      // parse current value
      if (resp[0] == this->slave_id_ && resp[1] == 0x03) {
        uint16_t cur = (static_cast<uint16_t>(resp[3]) << 8) | resp[4];
        uint16_t newv = (cur & ~w.mask) | (w.value & w.mask);
        // send write single
        uint8_t frame[8];
        frame[0] = this->slave_id_;
        frame[1] = 0x06;
        frame[2] = static_cast<uint8_t>((w.reg >> 8) & 0xFF);
        frame[3] = static_cast<uint8_t>(w.reg & 0xFF);
        frame[4] = static_cast<uint8_t>((newv >> 8) & 0xFF);
        frame[5] = static_cast<uint8_t>(newv & 0xFF);
        uint16_t crc2 = this->crc16(frame, 6);
        frame[6] = static_cast<uint8_t>(crc2 & 0xFF);
        frame[7] = static_cast<uint8_t>((crc2 >> 8) & 0xFF);
        this->uart_parent_->write_array(frame, 8);
        this->uart_parent_->flush();
      } else {
        ESP_LOGW(TAG, "Masked write: unexpected response when reading reg=0x%04X", w.reg);
      }
    } else {
      ESP_LOGW(TAG, "Masked write: timeout reading reg=0x%04X", w.reg);
    }
  }
}

}  // namespace custommodbus
}  // namespace esphome

