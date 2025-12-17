#include "custommodbus.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace custommodbus {

static const char *TAG = "custommodbus";

uint16_t CustomModbus::modbus_crc16(const uint8_t *buf, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];
    for (int i = 0; i < 8; i++) {
      if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
      else crc = crc >> 1;
    }
  }
  return crc;
}

void CustomModbus::setup() {
  ESP_LOGI(TAG, "CustomModbus setup, slave=%d groups=%d", this->slave_id_, (int)this->groups_.size());
  this->read_state_ = IDLE;
  this->read_group_idx_ = -1;
}

void CustomModbus::add_group(const std::string &name, uint32_t interval_ms) {
  Group g;
  g.name = name;
  g.interval_ms = interval_ms;
  g.next_due_ms = millis() + 10;
  g.index = 0;
  g.force_now = false;
  this->groups_.push_back(g);
  ESP_LOGD(TAG, "Added group %s interval=%u", name.c_str(), interval_ms);
}

void CustomModbus::add_read_to_group(const std::string &group_name, uint16_t reg, uint8_t count,
                                     esphome::Sensor *sensor, float smoothing_alpha) {
  for (auto &g : this->groups_) {
    if (g.name == group_name) {
      ReadDef rd;
      rd.reg = reg;
      rd.count = count;
      rd.sensor = sensor;
      rd.smoothing_alpha = smoothing_alpha;
      rd.last_val = NAN;
      rd.last_ts = 0;
      rd.consecutive_failures = 0;
      g.reads.push_back(rd);
      ESP_LOGD(TAG, "Group %s add read reg=%u count=%u alpha=%f", group_name.c_str(), reg, count, smoothing_alpha);
      return;
    }
  }
  ESP_LOGW(TAG, "Group %s not found for read reg=%u", group_name.c_str(), reg);
}

void CustomModbus::start_read(uint16_t reg, uint8_t count) {
  if (this->uart_parent_ == nullptr) {
    ESP_LOGW(TAG, "UART not configured");
    return;
  }
  uint8_t tx[8];
  tx[0] = this->slave_id_;
  tx[1] = 0x03; // Read Holding Registers
  tx[2] = (reg >> 8) & 0xFF;
  tx[3] = reg & 0xFF;
  tx[4] = 0x00;
  tx[5] = count;
  uint16_t crc = modbus_crc16(tx, 6);
  tx[6] = crc & 0xFF;
  tx[7] = (crc >> 8) & 0xFF;

  // clear rx buffer
  while (this->uart_parent_->available()) {
    uint8_t tmp;
    this->uart_parent_->read_array(&tmp, 1);
  }

  this->uart_parent_->write_array(tx, 8);
  this->uart_parent_->flush();
  delay(5); // DE->RE delay; juster hvis nødvendig

  this->read_reg_ = reg;
  this->read_count_ = count;
  this->read_expected_ = 0;
  this->read_got_ = 0;
  this->read_state_ = WAITING;
  this->read_start_ms_ = millis();
  this->last_byte_ms_ = millis();
  ESP_LOGVV(TAG, "Sent read reg=%u count=%u", reg, count);
}

void CustomModbus::handle_read_state() {
  if (this->read_state_ == IDLE) return;
  if (this->uart_parent_ == nullptr) { this->read_state_ = IDLE; return; }

  int avail = this->uart_parent_->available();
  if (avail > 0) {
    int to_read = std::min((int)sizeof(this->read_buf_) - this->read_got_, avail);
    if (to_read > 0) {
      int got = this->uart_parent_->read_array(this->read_buf_ + this->read_got_, to_read);
      if (got > 0) {
        this->read_got_ += got;
        this->last_byte_ms_ = millis();
      }
    }
  } else {
    if (millis() - this->read_start_ms_ > this->read_timeout_ms_) {
      ESP_LOGW(TAG, "Timeout waiting for response reg=0x%04X", this->read_reg_);
      this->read_state_ = IDLE;
      this->read_got_ = 0;
      return;
    }
  }

  if (this->read_got_ >= 3 && this->read_expected_ == 0) {
    uint8_t bytecount = this->read_buf_[2];
    this->read_expected_ = 3 + bytecount + 2;
  }

  if (this->read_expected_ > 0 && this->read_got_ >= this->read_expected_) {
    // validate header
    if (this->read_buf_[0] != this->slave_id_ || this->read_buf_[1] != 0x03) {
      ESP_LOGW(TAG, "Unexpected response slave/func mismatch %02X %02X", this->read_buf_[0], this->read_buf_[1]);
      this->read_state_ = IDLE;
      this->read_got_ = 0;
      this->read_expected_ = 0;
      return;
    }

    // CRC (low byte first)
    uint16_t recv_crc = (uint16_t)this->read_buf_[this->read_expected_ - 2] |
                        ((uint16_t)this->read_buf_[this->read_expected_ - 1] << 8);
    uint16_t calc_crc = modbus_crc16(this->read_buf_, this->read_expected_ - 2);
    if (recv_crc != calc_crc) {
      ESP_LOGW(TAG, "CRC mismatch recv=%04X calc=%04X", recv_crc, calc_crc);
      this->read_state_ = IDLE;
      this->read_got_ = 0;
      this->read_expected_ = 0;
      return;
    }

    uint8_t bytecount = this->read_buf_[2];
    if (bytecount < 2) {
      ESP_LOGW(TAG, "Bytecount too small");
      this->read_state_ = IDLE;
      this->read_got_ = 0;
      this->read_expected_ = 0;
      return;
    }

    // parse first register (udvid efter behov)
    uint16_t val = (this->read_buf_[3] << 8) | this->read_buf_[4];

    // publish to the read target
    if (this->read_group_idx_ >= 0 && this->read_group_idx_ < (int)this->groups_.size()) {
      Group &g = this->groups_[this->read_group_idx_];
      int idx = (int)this->read_index_in_group_;
      if (idx >= 0 && idx < (int)g.reads.size()) {
        ReadDef &rd = g.reads[idx];
        bool ok = validate_and_publish(rd, val);
        if (!ok) {
          rd.consecutive_failures++;
          ESP_LOGW(TAG, "Validation failed for reg=%u (consec=%d)", rd.reg, rd.consecutive_failures);
          if (rd.consecutive_failures >= 5) {
            rd.sensor->publish_state_unavailable();
          }
        } else {
          rd.consecutive_failures = 0;
        }
      }
    }

    // reset
    this->read_state_ = IDLE;
    this->read_got_ = 0;
    this->read_expected_ = 0;
    memset(this->read_buf_, 0, sizeof(this->read_buf_));
  }
}

bool CustomModbus::validate_and_publish(ReadDef &rd, uint16_t raw_val) {
  uint32_t now = millis();
  float value = static_cast<float>(raw_val);

  // Basic sanity check: finite
  if (!std::isfinite(value)) {
    ESP_LOGW(TAG, "Non-finite value for reg %u", rd.reg);
    return false;
  }

  // smoothing
  float out = value;
  if (!std::isnan(rd.last_val) && rd.smoothing_alpha > 0.0f) {
    out = rd.smoothing_alpha * value + (1.0f - rd.smoothing_alpha) * rd.last_val;
  }

  rd.sensor->publish_state(out);
  rd.last_val = out;
  rd.last_ts = now;
  return true;
}

void CustomModbus::process_reads() {
  if (this->read_state_ != IDLE) return;
  uint32_t now = millis();
  for (size_t gi = 0; gi < this->groups_.size(); ++gi) {
    Group &g = this->groups_[gi];
    if (g.force_now || now >= g.next_due_ms) {
      if (g.reads.empty()) continue;
      size_t idx = g.index;
      // set read pointers
      this->read_group_idx_ = (int)gi;
      this->read_index_in_group_ = (int)idx;
      ReadDef &rd = g.reads[idx];
      // start read
      this->start_read(rd.reg, rd.count);
      // schedule next due spread across group interval
      uint32_t per_item = g.interval_ms / std::max<uint32_t>(1, (uint32_t)g.reads.size());
      g.next_due_ms = now + per_item;
      g.index = (g.index + 1) % g.reads.size();
      g.force_now = false;
      break; // kun én read ad gangen
    }
  }
}

void CustomModbus::loop() {
  this->handle_read_state();
  static uint32_t last_ms = 0;
  uint32_t now = millis();
  if (now - last_ms > 50) {
    last_ms = now;
    this->process_reads();
  }
}

// write_single_register: send 0x06 frame og vent på echo
bool CustomModbus::write_single_register(uint16_t reg, uint16_t value) {
  if (this->uart_parent_ == nullptr) return false;
  uint8_t tx[8];
  tx[0] = this->slave_id_;
  tx[1] = 0x06;
  tx[2] = (reg >> 8) & 0xFF;
  tx[3] = reg & 0xFF;
  tx[4] = (value >> 8) & 0xFF;
  tx[5] = value & 0xFF;
  uint16_t crc = modbus_crc16(tx, 6);
  tx[6] = crc & 0xFF;
  tx[7] = (crc >> 8) & 0xFF;

  // clear rx
  while (this->uart_parent_->available()) {
    uint8_t tmp;
    this->uart_parent_->read_array(&tmp, 1);
  }

  this->uart_parent_->write_array(tx, 8);
  this->uart_parent_->flush();
  delay(5);

  // read response (echo)
  uint8_t resp[16];
  size_t got = 0;
  uint32_t start = millis();
  while (millis() - start < 500 && got < 8) {
    int avail = this->uart_parent_->available();
    if (avail > 0) {
      int r = this->uart_parent_->read_array(resp + got, std::min((int)sizeof(resp) - (int)got, avail));
      if (r > 0) got += r;
    } else {
      delay(1);
    }
  }
  if (got < 8) {
    ESP_LOGW(TAG, "Write no response");
    return false;
  }
  uint16_t recv_crc = (uint16_t)resp[6] | ((uint16_t)resp[7] << 8);
  uint16_t calc_crc = modbus_crc16(resp, 6);
  if (recv_crc != calc_crc) {
    ESP_LOGW(TAG, "Write CRC mismatch");
    return false;
  }
  ESP_LOGD(TAG, "Write reg=%u val=%u OK", reg, value);
  return true;
}

void CustomModbus::flush_group_now(const std::string &group_name) {
  for (auto &g : this->groups_) {
    if (g.name == group_name) {
      g.force_now = true;
      g.next_due_ms = millis();
      ESP_LOGI(TAG, "Flush requested for group %s", group_name.c_str());
      return;
    }
  }
  ESP_LOGW(TAG, "Flush requested for unknown group %s", group_name.c_str());
}

}  // namespace custommodbus
}  // namespace esphome
