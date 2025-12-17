#include "custommodbus.h"

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/log.h"
#include <algorithm>
#include <cstring>
#include <cstdio>
#include <driver/gpio.h>

namespace esphome {
namespace custommodbus {

static const char *const TAG = "custommodbus";

//
// SETUP
//
void CustomModbus::setup() {
  ESP_LOGCONFIG(TAG, "Setting up CustomModbus with slave ID %u", this->slave_id_);

  // Sørg for at RS485-transceiverens DE/RE-linjer (hvis hardware kræver det)
  // ikke styres manuelt her. Mange boards har auto-DE, så vi lader det være.
  // Dog kræver dit board at GPIO16/17/19 er sat HIGH for at fungere korrekt.
  // Vi sætter dem derfor høje her for at sikre kompatibilitet med dit setup.
  // (Hvis disse pins allerede bruges af andre komponenter i YAML, fjern disse kald.)
  gpio_set_level(static_cast<gpio_num_t>(16), 1);
  gpio_set_level(static_cast<gpio_num_t>(17), 1);
  gpio_set_level(static_cast<gpio_num_t>(19), 1);

  // Log uart_parent pointer for diagnostic
  ESP_LOGI(TAG, "uart_parent_ pointer: %p", this->uart_parent_);

  // Init read state machine
  this->read_state_ = IDLE;
  this->read_start_ms_ = 0;
  this->read_expected_ = 0;
  this->read_got_ = 0;
  this->read_index_ = 0;
  this->read_reg_ = 0;
  this->read_count_ = 0;
}

//
// LOOP
//
void CustomModbus::loop() {
  static uint32_t last = 0;
  const uint32_t now = millis();

  // Poll interval (ms). Øg hvis bus er langsom eller for at reducere trafik.
  const uint32_t interval = 200;  // kortere interval for mere responsiv state-machine

  if (now - last < interval)
    return;

  last = now;

  // Håndter eventuelle udgående skriver først
  this->process_writes();

  // Hvis vi ikke allerede er i gang med en læsning, start en ny rotation
  if (this->read_state_ == IDLE && !this->reads_.empty()) {
    if (this->read_index_ >= this->reads_.size())
      this->read_index_ = 0;

    auto &r = this->reads_[this->read_index_++];
    this->start_read(r.reg, r.count);
  }

  // Håndter den asynkrone læsning (ikke-blokerende)
  this->handle_read_state();
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
// START READ (initierer asynkron læsning)
//
void CustomModbus::start_read(uint16_t reg, uint8_t count) {
  if (this->uart_parent_ == nullptr) {
    ESP_LOGW(TAG, "UART parent not set; cannot start read reg=0x%04X", reg);
    return;
  }

  // Byg Modbus RTU read frame (function 3)
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

  // Tøm RX buffer før vi sender
  while (this->uart_parent_->available() > 0) {
    uint8_t tmp;
    this->uart_parent_->read_array(&tmp, 1);
  }

  // Send frame
  ESP_LOGVV(TAG, "TX frame (start_read):");
  for (int i = 0; i < 8; ++i) ESP_LOGVV(TAG, " %02X", frame[i]);

  this->uart_parent_->write_array(frame, 8);
  this->uart_parent_->flush();

  // Forbered state-machine til at modtage svar
  this->read_state_ = WAITING;
  this->read_start_ms_ = millis();
  this->read_expected_ = static_cast<uint8_t>(5 + count * 2);  // slave + func + bytecount + data + crc_lo + crc_hi
  this->read_got_ = 0;
  this->read_reg_ = reg;
  this->read_count_ = count;
}

//
// HANDLE READ STATE (asynkron modtagelse og validering)
//
void CustomModbus::handle_read_state() {
  if (this->uart_parent_ == nullptr)
    return;

  // Hvis vi ikke venter på noget, intet at gøre
  if (this->read_state_ == IDLE)
    return;

  // Læs tilgængelige bytes ind i buffer
  int avail = this->uart_parent_->available();
  if (avail > 0) {
    int to_read = std::min(static_cast<int>(sizeof(this->read_buf_) - this->read_got_), avail);
    if (to_read > 0) {
      this->uart_parent_->read_array(this->read_buf_ + this->read_got_, to_read);
      this->read_got_ += to_read;

      // Log rå bytes i hex (diagnostik)
      ESP_LOGVV(TAG, "RX raw (%d):", this->read_got_);
      for (uint8_t i = 0; i < this->read_got_; ++i) ESP_LOGVV(TAG, " %02X", this->read_buf_[i]);
    }
  }

  // Hvis vi har modtaget nok bytes, valider og processer
  if (this->read_got_ >= this->read_expected_) {
    // Valider slave id og function
    if (this->read_buf_[0] != this->slave_id_ || this->read_buf_[1] != 3) {
      ESP_LOGW(TAG, "Ignoring frame wrong slave/function %02X %02X", this->read_buf_[0], this->read_buf_[1]);
      // Ryd op og gå til IDLE
      this->read_state_ = IDLE;
      this->read_got_ = 0;
      return;
    }

    // Valider CRC (Modbus CRC: lav byte først)
    uint16_t recv_crc = (static_cast<uint16_t>(this->read_buf_[this->read_expected_ - 1]) << 8) |
                        this->read_buf_[this->read_expected_ - 2];
    uint16_t calc_crc = this->crc16(this->read_buf_, this->read_expected_ - 2);
    if (recv_crc != calc_crc) {
      ESP_LOGW(TAG, "Ignoring frame CRC mismatch got %04X calc %04X", recv_crc, calc_crc);
      this->read_state_ = IDLE;
      this->read_got_ = 0;
      return;
    }

    // Kopier data til et lokalt svar-array for videre behandling
    uint8_t resp_len = this->read_expected_;
    uint8_t resp[64];
    memcpy(resp, this->read_buf_, resp_len);

    // Processér svaret: udpak værdier og publicér til sensorer
    // Forventet format: [slave][func][bytecount][data...][crc_lo][crc_hi]
    if (resp_len < 5) {
      ESP_LOGW(TAG, "Received too short response %d bytes", resp_len);
      this->read_state_ = IDLE;
      this->read_got_ = 0;
      return;
    }

    const uint16_t raw16 = (static_cast<uint16_t>(resp[3]) << 8) | resp[4];
    uint32_t raw32 = 0;
    if (this->read_count_ == 2 && resp_len >= 7) {
      raw32 = (static_cast<uint32_t>(resp[3]) << 24) |
              (static_cast<uint32_t>(resp[4]) << 16) |
              (static_cast<uint32_t>(resp[5]) << 8) |
              static_cast<uint32_t>(resp[6]);
    }

    // Find den ReadItem der svarer til denne læsning (rotation)
    // Vi bruger read_index_ - 1 (med wrap) fordi vi inkrementerede før start_read
    size_t idx = (this->read_index_ == 0) ? (this->reads_.size() - 1) : (this->read_index_ - 1);
    if (idx < this->reads_.size()) {
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

      // Binary/text publishing midlertidigt deaktiveret i runtime
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

    // Ryd op og gå til IDLE
    this->read_state_ = IDLE;
    this->read_got_ = 0;
    return;
  }

  // Timeout håndtering
  if (millis() - this->read_start_ms_ > this->read_timeout_ms_) {
    ESP_LOGW(TAG, "Timeout waiting for Modbus response reg=0x%04X", this->read_reg_);
    this->read_state_ = IDLE;
    this->read_got_ = 0;
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
