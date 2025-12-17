// custommodbus.cpp
#include "esphome.h"
#include "custommodbus.h"

// Inkluder konkrete sensorklasser for at undgå incomplete-type fejl
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"

#include <algorithm>
#include <vector>
#include <cstring>

namespace esphome {
namespace custommodbus {

static const char *TAG = "custommodbus";

// Helper: CRC16 (Modbus)
static uint16_t modbus_crc16(const uint8_t *buf, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];
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

// --- Setup / constructor ---
void CustomModbus::setup() {
  // Sørg for at GPIO16, 17 og 19 er sat som outputs og HIGH ved opstart
  // (Nødvendigt for dit hardware som du skrev)
  pinMode(16, OUTPUT);
  pinMode(17, OUTPUT);
  pinMode(19, OUTPUT);
  digitalWrite(16, HIGH);
  digitalWrite(17, HIGH);
  digitalWrite(19, HIGH);

  // Init interne variabler (sikrer kendt starttilstand)
  this->read_state_ = IDLE;
  this->read_index_ = -1;  // starter før første element så process_reads() vælger index 0
  this->read_got_ = 0;
  this->read_expected_ = 0;
  this->read_timeout_ms_ = 500;  // default timeout, kan overskrives fra header/yaml hvis ønsket

  ESP_LOGI(TAG, "CustomModbus setup complete, pins 16/17/19 set HIGH");
}

// --- start_read: header-signatur start_read(uint16_t reg, uint8_t count) ---
void CustomModbus::start_read(uint16_t reg, uint8_t count) {
  // Gem parametre i medlemvariabler
  this->read_reg_ = reg;
  this->read_count_ = count;
  this->read_expected_ = static_cast<int>(5 + count * 2); // slave + func + bytecount + data + crc_lo + crc_hi
  this->read_got_ = 0;
  this->read_state_ = WAITING;
  this->read_start_ms_ = millis();

  // Byg Modbus RTU request (slave id fra medlemvariabel slave_id_)
  uint8_t tx[8];
  tx[0] = this->slave_id_;
  tx[1] = 0x03; // Read Holding Registers
  tx[2] = static_cast<uint8_t>((reg >> 8) & 0xFF);
  tx[3] = static_cast<uint8_t>(reg & 0xFF);
  tx[4] = 0x00;
  tx[5] = count;
  uint16_t crc = modbus_crc16(tx, 6);
  tx[6] = crc & 0xFF;
  tx[7] = (crc >> 8) & 0xFF;

  // Send via UART. Vi antager auto-DE, så vi rører ikke DE-pin.
  if (this->uart_parent_ != nullptr) {
    this->uart_parent_->write_array(tx, 8);
    this->uart_parent_->flush();
    ESP_LOGVV(TAG, "TX frame (start_read):");
    // Log hex bytes i en linje for debugging
    {
      char tmp[64];
      int pos = 0;
      for (int i = 0; i < 8; i++) pos += snprintf(tmp + pos, sizeof(tmp) - pos, " %02X", tx[i]);
      ESP_LOGVV(TAG, "%s", tmp);
    }
  } else {
    ESP_LOGW(TAG, "UART parent not configured, cannot send Modbus request");
    this->read_state_ = IDLE;
  }
}

// --- handle_read_state: kaldet fra loop() ---
void CustomModbus::handle_read_state() {
  if (this->read_state_ == IDLE) return;

  if (this->uart_parent_ == nullptr) {
    this->read_state_ = IDLE;
    return;
  }

  int avail = this->uart_parent_->available();
  if (avail <= 0) {
    // Timeout check
    if (millis() - this->read_start_ms_ > this->read_timeout_ms_) {
      ESP_LOGW(TAG, "Timeout waiting for Modbus response reg=0x%04X", this->read_reg_);
      this->read_state_ = IDLE;
    }
    return;
  }

  // Begræns læsning til bufferstørrelse
  int to_read = std::min(static_cast<int>(sizeof(this->read_buf_) - this->read_got_), avail);
  if (to_read <= 0) return;

  int got = this->uart_parent_->read_array(this->read_buf_ + this->read_got_, to_read);
  if (got > 0) this->read_got_ += got;

  // Hvis vi har modtaget nok bytes, valider og processér
  if (this->read_got_ >= this->read_expected_) {
    // Tjek slave id og funktion
    if (this->read_buf_[0] != this->slave_id_ || this->read_buf_[1] != 0x03) {
      ESP_LOGW(TAG, "Unexpected Modbus response (slave/func mismatch) got slave=0x%02X func=0x%02X expected slave=0x%02X func=0x03",
               this->read_buf_[0], this->read_buf_[1], this->slave_id_);
      this->read_state_ = IDLE;
      return;
    }

    // CRC check (Modbus: CRC low byte first, then high byte)
    uint16_t recv_crc = (static_cast<uint16_t>(this->read_buf_[this->read_expected_ - 1]) << 8) |
                        static_cast<uint16_t>(this->read_buf_[this->read_expected_ - 2]);
    uint16_t calc_crc = modbus_crc16(this->read_buf_, this->read_expected_ - 2);
    if (recv_crc != calc_crc) {
      ESP_LOGW(TAG, "CRC mismatch: recv=0x%04X calc=0x%04X", recv_crc, calc_crc);
      this->read_state_ = IDLE;
      return;
    }

    // Extract data bytes (bytecount = read_buf_[2])
    uint8_t bytecount = this->read_buf_[2];
    if (bytecount + 5 > static_cast<uint8_t>(this->read_expected_)) {
      ESP_LOGW(TAG, "Bytecount mismatch in Modbus response: bytecount=%u expected=%d", bytecount, this->read_expected_);
      this->read_state_ = IDLE;
      return;
    }

    // For hver registerpar (2 bytes) konverter til uint16_t
    std::vector<uint16_t> regs;
    regs.reserve(bytecount / 2);
    for (int i = 0; i < bytecount; i += 2) {
      uint16_t val = (static_cast<uint16_t>(this->read_buf_[3 + i]) << 8) |
                     static_cast<uint16_t>(this->read_buf_[3 + i + 1]);
      regs.push_back(val);
    }

    // Gem eller processér værdierne i reads_ (eksempelvis publish til sensorer)
    if (this->read_index_ >= 0 && this->read_index_ < static_cast<int>(this->reads_.size())) {
      auto &it = this->reads_[this->read_index_];
      // Hvis binary_sensor er sat, publicér boolean (non-zero => true)
      if (it.binary_sensor != nullptr && !regs.empty()) {
        bool state = (regs[0] != 0);
        it.binary_sensor->publish_state(state);
      }
      // Hvis text_sensor er sat, publicér tekst (første register som tal)
      if (it.text_sensor != nullptr && !regs.empty()) {
        char buf[32];
        snprintf(buf, sizeof(buf), "%u", regs[0]);
        it.text_sensor->publish_state(std::string(buf));
      }
      // Hvis der er en sensor (float/number) kan du tilføje publish her
      if (it.sensor != nullptr && !regs.empty()) {
        // Eksempel: publish første register som integer/float
        it.sensor->publish_state(static_cast<float>(regs[0]));
      }
    } else {
      ESP_LOGVV(TAG, "Read index out of range: %d (reads_.size=%d)", this->read_index_, static_cast<int>(this->reads_.size()));
    }

    // Ryd buffer og afslut læsning
    this->read_got_ = 0;
    std::memset(this->read_buf_, 0, sizeof(this->read_buf_));
    this->read_state_ = IDLE;
  }
}

// --- process_reads: planlæg næste læsning ---
void CustomModbus::process_reads() {
  // Simpel rundtur gennem reads_ og start næste læsning
  if (this->read_state_ != IDLE) return;
  if (this->reads_.empty()) return;

  // Increment index og wrap
  this->read_index_++;
  if (this->read_index_ >= static_cast<int>(this->reads_.size())) this->read_index_ = 0;

  auto &r = this->reads_[this->read_index_];
  // Start læsning for dette register
  this->start_read(r.reg, r.count);
}

// --- loop: kald fra hovedloop ---
void CustomModbus::loop() {
  // Håndter indkommende bytes hvis vi venter
  this->handle_read_state();

  // Hvis IDLE, planlæg næste læsning med passende interval
  static uint32_t last_read_ms = 0;
  const uint32_t read_interval_ms = 1000; // juster efter behov
  if (this->read_state_ == IDLE && millis() - last_read_ms > read_interval_ms) {
    last_read_ms = millis();
    this->process_reads();
  }
}

}  // namespace custommodbus
}  // namespace esphome

