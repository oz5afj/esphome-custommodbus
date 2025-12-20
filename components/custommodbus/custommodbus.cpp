#include "custommodbus.h"
#include "esphome/core/log.h"
#include <cmath>
#include <cstring>

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

// preiss - START: per-sensor publish med decimals og threshold
void CustomModbus::publish_sensor_filtered(esphome::sensor::Sensor *sensor, float value, int decimals, float threshold) {
  if (!sensor) return;

  // Hurtig faktor uden dyr pow() kald
  static const float pow10[] = {1.0f, 10.0f, 100.0f, 1000.0f, 10000.0f, 100000.0f, 1000000.0f};
  float factor = 1.0f;
  if (decimals >= 0 && decimals <= 6) {
    factor = pow10[decimals];
  } else {
    factor = std::pow(10.0f, static_cast<float>(decimals)); // fallback hvis usædvanligt decimals
  }

  float rounded = std::round(value * factor) / factor;

  // Find sidste publicerede værdi for denne sensor
  auto it = this->last_published_values_.find(sensor);
  if (it == this->last_published_values_.end()) {
    sensor->publish_state(rounded);
    this->last_published_values_[sensor] = rounded;
    ESP_LOGD(TAG, "Published initial sensor value %.*f", decimals, rounded);
    return;
  }

  float last = it->second;
  if (std::fabs(rounded - last) >= threshold) {
    sensor->publish_state(rounded);
    this->last_published_values_[sensor] = rounded;
    ESP_LOGD(TAG, "Published sensor value changed %.*f -> %.*f", decimals, last, rounded);
  } else {
    ESP_LOGD(TAG, "Skipped publish (delta %.6f < threshold %.6f)", std::fabs(rounded - last), threshold);
  }
}
// preiss - END

// --- Public API: add/read/write registrations ---
void CustomModbus::add_read_sensor(uint16_t reg, uint8_t count, DataType type, float scale, esphome::sensor::Sensor *s, int decimals, float delta_threshold) {
  ReadItem it;
  it.reg = reg;
  it.count = count;
  it.type = type;
  it.scale = scale;
  it.sensor = s;
  it.binary_sensor = nullptr;
  it.text_sensor = nullptr;
  it.bitmask = 0;
  it.decimals = decimals;
  it.delta_threshold = delta_threshold;
  this->reads_.push_back(it);
}

void CustomModbus::add_read_sensor(uint16_t reg, uint8_t count, uint8_t type_as_int, float scale, esphome::sensor::Sensor *s, int decimals, float delta_threshold) {
  DataType t = static_cast<DataType>(type_as_int);
  this->add_read_sensor(reg, count, t, scale, s, decimals, delta_threshold);
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
  if (!should_write(reg, value)) {
    return;
  }

  WriteItem w;
  w.reg = reg;
  w.value = value;
  w.use_mask = false;
  w.mask = 0;
  this->writes_.push_back(w);

  record_write(reg, value);
}

void CustomModbus::write_bitmask(uint16_t reg, uint16_t mask, bool state) {
  uint16_t new_value = state ? mask : 0;

  if (!should_write(reg, new_value)) {
    return;
  }

  WriteItem w;
  w.reg = reg;
  w.mask = mask;
  w.use_mask = true;
  w.value = new_value;
  this->writes_.push_back(w);

  record_write(reg, new_value);
}

// --- Grouped reads: build blocks from reads_ ---
void CustomModbus::build_read_blocks() {
  this->blocks_.clear();
  if (reads_.empty()) return;

  // Sort by register
  std::sort(reads_.begin(), reads_.end(), [](const ReadItem &a, const ReadItem &b) {
    return a.reg < b.reg;
  });

  ReadBlock current;
  bool first = true;

  for (auto &it : this->reads_) {
    if (first) {
      current.start_reg = it.reg;
      current.count = it.count;
      current.items.clear();
      current.items.push_back(&it);
      first = false;
      continue;
    }

    uint16_t expected_next = current.start_reg + current.count;

    if (it.reg == expected_next) {
      // extend block
      current.count += it.count;
      current.items.push_back(&it);
    } else {
      // save block
      blocks_.push_back(current);

      // start new block
      current.start_reg = it.reg;
      current.count = it.count;
      current.items.clear();
      current.items.push_back(&it);
    }
  }

  // save last block
  if (!first) {
    blocks_.push_back(current);
  }

  ESP_LOGI(TAG, "Grouped %u read items into %u Modbus blocks",
           (unsigned)reads_.size(), (unsigned)blocks_.size());
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

  ESP_LOGI(TAG, "CustomModbus setup complete (slave id=%u)", this->slave_id_);

  // Hvis grouped-reads er slået til, bygg blokke
  if (this->use_grouped_reads_) {
    this->build_read_blocks();
  }
}

void CustomModbus::loop() {
  // Throttle hele loopet: sørg for mindst X ms mellem iterationer der starter nye reads/writes
  static const uint32_t LOOP_THROTTLE_MS = 2000; // 2 sekunder mellem "aktive" iterationer
  static uint32_t last_loop_ms = 0;
  uint32_t now = millis();

  if (now - last_loop_ms < LOOP_THROTTLE_MS) {
    return;
  }
  last_loop_ms = now;

  this->process_reads();
  this->process_writes();
}


// --- Internal: send a Modbus read request (function 0x03) ---
bool CustomModbus::read_registers(uint16_t reg, uint8_t count, uint8_t *resp, uint8_t &resp_len) {
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

  this->uart_parent_->write_array(frame, 8);
  this->uart_parent_->flush();

  (void)resp;
  (void)resp_len;
  return true;
}

// --- Internal: process reads state machine ---
void CustomModbus::process_reads() {
  if (!this->uart_parent_) return;

  // MODE 1: enkelt-read
  if (!this->use_grouped_reads_) {

    if (read_state_ == IDLE) {
      if (this->reads_.empty()) return;

      if (read_index_ >= this->reads_.size()) read_index_ = 0;
      const ReadItem &r = this->reads_[read_index_];

      uint16_t reg = r.reg;
      uint8_t count = r.count;
      read_state_ = WAITING;
      read_start_ms_ = millis();
      read_expected_ = static_cast<uint8_t>(5 + count * 2);
      read_got_ = 0;
      read_reg_ = reg;
      read_count_ = count;

      memset(read_buf_, 0, sizeof(read_buf_));

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

      read_index_++;
      if (read_index_ >= this->reads_.size()) read_index_ = 0;

      return;
    }

    if (read_state_ == WAITING) {
      size_t avail = this->uart_parent_->available();
      if (avail > 0) {
        int to_read = std::min(static_cast<int>(sizeof(read_buf_) - read_got_), static_cast<int>(avail));
        if (to_read > 0) {
          this->uart_parent_->read_array(this->read_buf_ + read_got_, to_read);
          read_got_ += to_read;
        }
      }

      if (read_got_ >= read_expected_) {
        if (read_buf_[0] != this->slave_id_ || read_buf_[1] != 0x03) {
          ESP_LOGW(TAG, "Ignoring frame wrong slave/function %02X %02X", read_buf_[0], read_buf_[1]);
          read_state_ = IDLE;
          read_got_ = 0;
          return;
        }

        uint16_t recv_crc = (static_cast<uint16_t>(read_buf_[read_expected_ - 1]) << 8) | read_buf_[read_expected_ - 2];
        uint16_t calc_crc = this->crc16(read_buf_, read_expected_ - 2);
        if (recv_crc != calc_crc) {
          ESP_LOGW(TAG, "CRC mismatch (recv=0x%04X calc=0x%04X) reg=0x%04X", recv_crc, calc_crc, read_reg_);
          read_state_ = IDLE;
          read_got_ = 0;
          return;
        }

        uint8_t bytecount = read_buf_[2];
        const uint8_t *data = &read_buf_[3];

        bool matched = false;
        for (auto &it : this->reads_) {
          if (it.reg == read_reg_ && it.count == read_count_) {
            matched = true;
            if (it.binary_sensor) {
              uint16_t val = (static_cast<uint16_t>(data[0]) << 8) | data[1];
              bool state = (val & it.bitmask) != 0;
              it.binary_sensor->publish_state(state);
            } else if (it.text_sensor) {
              uint16_t val = (static_cast<uint16_t>(data[0]) << 8) | data[1];
              char buf[16];
              snprintf(buf, sizeof(buf), "%u", val);
              it.text_sensor->publish_state(std::string(buf));
            } else if (it.sensor) {
              // numeric sensor: support types (brug per-sensor decimals og delta_threshold)
              if (it.type == TYPE_UINT16) {
                uint16_t v = (static_cast<uint16_t>(data[0]) << 8) | data[1];
                float value = static_cast<float>(v) * it.scale;
                this->publish_sensor_filtered(it.sensor, value, it.decimals, it.delta_threshold);
              } else if (it.type == TYPE_INT16) {
                int16_t v = (static_cast<int16_t>(data[0]) << 8) | data[1];
                float value = static_cast<float>(v) * it.scale;
                this->publish_sensor_filtered(it.sensor, value, it.decimals, it.delta_threshold);
              } else if (it.type == TYPE_UINT32) {
                if (bytecount >= 4) {
                  // Debug: kommenteret tung snprintf/ESP_LOGD for at reducere CPU
                  // {
                  //   char dbg[128];
                  //   snprintf(dbg, sizeof(dbg), "DBG single reg=0x%04X bytes=%02X %02X %02X %02X",
                  //            it.reg, data[0], data[1], data[2], data[3]);
                  //   ESP_LOGD(TAG, "%s", dbg);
                  // }

                  // High-word first (U_DWORD)
                  uint32_t hi = (static_cast<uint32_t>(data[0]) << 8) | data[1];
                  uint32_t lo = (static_cast<uint32_t>(data[2]) << 8) | data[3];
                  uint32_t v = (hi << 16) | lo;
                  // For TYPE_UINT32 we keep using it.scale (if configured)
                  this->publish_sensor_filtered(it.sensor, static_cast<float>(v) * it.scale, it.decimals, it.delta_threshold);
                }
              } else if (it.type == TYPE_UINT32_R) {
                if (bytecount >= 4) {
                  // Debug: kommenteret tung snprintf/ESP_LOGD for at reducere CPU
                  // {
                  //   char dbg[128];
                  //   snprintf(dbg, sizeof(dbg), "DBG single reg=0x%04X bytes=%02X %02X %02X %02X",
                  //            it.reg, data[0], data[1], data[2], data[3]);
                  //   ESP_LOGD(TAG, "%s", dbg);
                  // }

                  // CORRECTED: Low-word first variant for this device (U_DWORD_R)
                  // low 16-bit word is in data[2..3], high 16-bit word in data[0..1]
                  uint32_t lo = (static_cast<uint32_t>(data[2]) << 8) | data[3];
                  uint32_t hi = (static_cast<uint32_t>(data[0]) << 8) | data[1];
                  uint32_t v = (lo << 16) | hi;
                  // Publish raw 32-bit value; let YAML multiply/filter handle scaling if configured.
                  this->publish_sensor_filtered(it.sensor, static_cast<float>(v) * 1.0f, it.decimals, it.delta_threshold);
                }
              }
            }
            break;
          }
        }

        if (!matched) {
          ESP_LOGW(TAG, "Received response for reg=0x%04X count=%u but no matching read item", read_reg_, read_count_);
        }

        read_state_ = IDLE;
        read_got_ = 0;
        return;
      }

      if (millis() - read_start_ms_ > read_timeout_ms_) {
        ESP_LOGW(TAG, "Timeout waiting for Modbus response reg=0x%04X", read_reg_);
        read_state_ = IDLE;
        read_got_ = 0;
        return;
      }
    }

    return;
  }

  // MODE 2: grouped-reads
  if (read_state_ == IDLE) {
    if (this->blocks_.empty()) return;

    if (read_index_ >= this->blocks_.size()) read_index_ = 0;
    const ReadBlock &b = this->blocks_[read_index_];

    uint16_t reg = b.start_reg;
    uint8_t count = b.count;

    read_state_ = WAITING;
    read_start_ms_ = millis();
    read_expected_ = static_cast<uint8_t>(5 + count * 2);
    read_got_ = 0;
    read_reg_ = reg;
    read_count_ = count;

    memset(read_buf_, 0, sizeof(read_buf_));

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

    read_index_++;
    if (read_index_ >= this->blocks_.size()) read_index_ = 0;

    return;
  }

  if (read_state_ == WAITING) {
    size_t avail = this->uart_parent_->available();
    if (avail > 0) {
      int to_read = std::min(static_cast<int>(sizeof(read_buf_) - read_got_), static_cast<int>(avail));
      if (to_read > 0) {
        this->uart_parent_->read_array(this->read_buf_ + read_got_, to_read);
        read_got_ += to_read;
      }
    }

    if (read_got_ >= read_expected_) {
      if (read_buf_[0] != this->slave_id_ || read_buf_[1] != 0x03) {
        ESP_LOGW(TAG, "Ignoring frame wrong slave/function %02X %02X", read_buf_[0], read_buf_[1]);
        read_state_ = IDLE;
        read_got_ = 0;
        return;
      }

      uint16_t recv_crc = (static_cast<uint16_t>(read_buf_[read_expected_ - 1]) << 8) | read_buf_[read_expected_ - 2];
      uint16_t calc_crc = this->crc16(read_buf_, read_expected_ - 2);
      if (recv_crc != calc_crc) {
        ESP_LOGW(TAG, "CRC mismatch (recv=0x%04X calc=0x%04X) reg=0x%04X", recv_crc, calc_crc, read_reg_);
        read_state_ = IDLE;
        read_got_ = 0;
        return;
      }

      uint8_t bytecount = read_buf_[2];
      const uint8_t *data = &read_buf_[3];

      bool matched = false;

      for (auto &b : this->blocks_) {
        if (b.start_reg == read_reg_ && b.count == read_count_) {

          for (auto *it : b.items) {
            // yield kort for at undgå lange blokeringer i meget store blokke
            static int yield_counter = 0;
            if ((yield_counter++ & 0x1F) == 0) delay(0);

            uint16_t offset = (it->reg - b.start_reg) * 2;
            if (offset + 1 >= bytecount) {
              ESP_LOGW(TAG, "Offset out of range for reg=0x%04X", it->reg);
              continue;
            }
            const uint8_t *ptr = data + offset;

            if (it->binary_sensor) {
              uint16_t v = (static_cast<uint16_t>(ptr[0]) << 8) | ptr[1];
              bool state = (v & it->bitmask) != 0;
              it->binary_sensor->publish_state(state);
            } else if (it->text_sensor) {
              uint16_t v = (static_cast<uint16_t>(ptr[0]) << 8) | ptr[1];
              char buf[16];
              snprintf(buf, sizeof(buf), "%u", v);
              it->text_sensor->publish_state(std::string(buf));
            } else if (it->sensor) {
              // grouped numeric handling with per-sensor decimals/threshold
              if (it->type == TYPE_UINT16) {
                uint16_t v = (static_cast<uint16_t>(ptr[0]) << 8) | ptr[1];
                float value = static_cast<float>(v) * it->scale;
                this->publish_sensor_filtered(it->sensor, value, it->decimals, it->delta_threshold);
              } else if (it->type == TYPE_INT16) {
                int16_t v = (static_cast<int16_t>(ptr[0]) << 8) | ptr[1];
                float value = static_cast<float>(v) * it->scale;
                this->publish_sensor_filtered(it->sensor, value, it->decimals, it->delta_threshold);
              } else if (it->type == TYPE_UINT32) {
                if (offset + 3 >= bytecount) {
                  ESP_LOGW(TAG, "UINT32 out of range for reg=0x%04X", it->reg);
                } else {
                  // Debug: kommenteret tung snprintf/ESP_LOGD for at reducere CPU
                  // {
                  //   char dbg[128];
                  //   snprintf(dbg, sizeof(dbg), "DBG grouped reg=0x%04X offset=%u bytes=%02X %02X %02X %02X",
                  //            it->reg, offset/2, ptr[0], ptr[1], ptr[2], ptr[3]);
                  //   ESP_LOGD(TAG, "%s", dbg);
                  // }

                  // High-word first (U_DWORD)
                  uint32_t hi = (static_cast<uint32_t>(ptr[0]) << 8) | ptr[1];
                  uint32_t lo = (static_cast<uint32_t>(ptr[2]) << 8) | ptr[3];
                  uint32_t v = (hi << 16) | lo;
                  this->publish_sensor_filtered(it->sensor, static_cast<float>(v) * it->scale, it->decimals, it->delta_threshold);
                }
              } else if (it->type == TYPE_UINT32_R) {
                if (offset + 3 >= bytecount) {
                  ESP_LOGW(TAG, "UINT32_R out of range for reg=0x%04X", it->reg);
                } else {
                  // Debug: kommenteret tung snprintf/ESP_LOGD for at reducere CPU
                  // {
                  //   char dbg[128];
                  //   snprintf(dbg, sizeof(dbg), "DBG grouped reg=0x%04X offset=%u bytes=%02X %02X %02X %02X",
                  //            it->reg, offset/2, ptr[0], ptr[1], ptr[2], ptr[3]);
                  //   ESP_LOGD(TAG, "%s", dbg);
                  // }

                  // CORRECTED: Low-word first variant for this device (U_DWORD_R)
                  // low 16-bit word is in ptr[2..3], high 16-bit word in ptr[0..1]
                  uint32_t lo = (static_cast<uint32_t>(ptr[2]) << 8) | ptr[3];
                  uint32_t hi = (static_cast<uint32_t>(ptr[0]) << 8) | ptr[1];
                  uint32_t v = (lo << 16) | hi;
                  // Publish raw 32-bit value; let YAML multiply/filter handle scaling if configured.
                  this->publish_sensor_filtered(it->sensor, static_cast<float>(v) * 1.0f, it->decimals, it->delta_threshold);
                }
              }
            }
          }

          matched = true;
          break;
        }
      }

      if (!matched) {
        ESP_LOGW(TAG, "Received response for reg=0x%04X count=%u but no matching block", read_reg_, read_count_);
      }

      read_state_ = IDLE;
      read_got_ = 0;
      return;
    }

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

  WriteItem w = this->writes_.front();
  this->writes_.erase(this->writes_.begin());

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
  } else {
    uint8_t resp[64];

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

    uint32_t start = millis();
    uint8_t got = 0;
    // Forkortet timeout fra 200 ms til 50 ms for at undgå lange blokeringer
    while (millis() - start < 50) {
      size_t avail = this->uart_parent_->available();
      if (avail) {
        int to_read = std::min(static_cast<int>(sizeof(resp) - got), static_cast<int>(avail));
        this->uart_parent_->read_array(resp + got, to_read);
        got += to_read;
        if (got >= 7) break;
      }
      delay(2);
    }
    if (got >= 7) {
      if (resp[0] == this->slave_id_ && resp[1] == 0x03) {
        uint16_t cur = (static_cast<uint16_t>(resp[3]) << 8) | resp[4];
        uint16_t newv = (cur & ~w.mask) | (w.value & w.mask);

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

bool CustomModbus::should_write(uint16_t reg, uint16_t value) {
  uint32_t now = millis();

  if (last_written_value_.count(reg)) {
    uint16_t last_value = last_written_value_[reg];
    if (last_value == value) {
      ESP_LOGI(TAG, "Skip write reg=0x%04X (unchanged value %u)", reg, value);
      return false;
    }
  }

  if (last_write_time_.count(reg)) {
    uint32_t last_time = last_write_time_[reg];
    if (now - last_time < WRITE_COOLDOWN_MS) {
      ESP_LOGI(TAG, "Skip write reg=0x%04X (cooldown active)", reg);
      return false;
    }
  }

  return true;
}

void CustomModbus::record_write(uint16_t reg, uint16_t value) {
  last_written_value_[reg] = value;
  last_write_time_[reg] = millis();
}

}  // namespace custommodbus
}  // namespace esphome
