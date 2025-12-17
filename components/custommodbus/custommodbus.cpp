#include "custommodbus.h"
#include "esphome.h"

#include "esphome/components/uart/uart.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <vector>

namespace esphome {
namespace custommodbus {

static const char *TAG = "custommodbus";

// --- Hjælpefunktioner ---
static uint16_t modbus_crc16(const uint8_t *buf, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else
        crc >>= 1;
    }
  }
  return crc;
}

// --- CustomModbus implementation ---

void CustomModbus::setup() {
  ESP_LOGCONFIG(TAG, "Setting up CustomModbus");

  // Sørg for at GPIO16, 17 og 19 er sat HIGH så boardet fungerer med dit hardware
  // (Bruger Arduino-API direkte her; ESPHome wrapper kan også bruges hvis ønsket)
  pinMode(16, OUTPUT);
  digitalWrite(16, HIGH);

  pinMode(17, OUTPUT);
  digitalWrite(17, HIGH);

  pinMode(19, OUTPUT);
  digitalWrite(19, HIGH);

  // Hvis du ønsker manuel DE-kontrol, kan du initialisere DE-pin her (valgfrit)
  // pinMode(this->de_pin_, OUTPUT);
  // digitalWrite(this->de_pin_, LOW); // default receive
}

void CustomModbus::start_read(uint16_t reg, uint8_t count) {
  // Denne signatur matcher headeren som build-loggen viste
  // Forbered læse-kommando (funktion 0x03)
  if (!this->uart_parent_) {
    ESP_LOGW(TAG, "UART parent not set, cannot start read");
    return;
  }

  // Gem ønskede læseregistre i kø (reads_ er en vector af ReadItem defineret i header)
  ReadItem item;
  item.reg = reg;
  item.count = count;
  this->reads_.push_back(item);
}

void CustomModbus::handle_read_state() {
  // Denne funktion håndterer den interne state-maskine for læsning
  // Forventede medlemvariabler (skal være deklareret i header):
  // read_state_, read_start_ms_, read_expected_, read_got_, read_reg_, read_count_, read_buf_, read_index_, read_timeout_ms_

  // Hvis der ikke er en aktiv læseoperation, returner
  if (this->read_state_ == IDLE) {
    return;
  }

  // Hvis vi venter på svar, tjek UART buffer
  if (this->read_state_ == WAITING) {
    size_t avail = this->uart_parent_->available();
    if (avail > 0) {
      int to_read = std::min(static_cast<int>(sizeof(this->read_buf_) - this->read_got_), static_cast<int>(avail));
      int got = this->uart_parent_->read_bytes(this->read_buf_ + this->read_got_, to_read);
      if (got > 0) {
        this->read_got_ += got;
      }
    }

    // Har vi modtaget nok bytes?
    if (this->read_got_ >= this->read_expected_) {
      // Tjek slave id og funktion
      if (this->read_buf_[0] != this->slave_id_ || this->read_buf_[1] != 0x03) {
        ESP_LOGW(TAG, "Unexpected Modbus response (slave/fn mismatch)");
        this->read_state_ = IDLE;
        return;
      }

      // CRC i modtaget frame (low byte først)
      uint16_t recv_crc = (static_cast<uint16_t>(this->read_buf_[this->read_expected_ - 1]) << 8) |
                          static_cast<uint16_t>(this->read_buf_[this->read_expected_ - 2]);
      uint16_t calc_crc = modbus_crc16(this->read_buf_, this->read_expected_ - 2);

      if (recv_crc != calc_crc) {
        ESP_LOGW(TAG, "CRC mismatch: recv=0x%04X calc=0x%04X", recv_crc, calc_crc);
        this->read_state_ = IDLE;
        return;
      }

      // Alt ok: processer data
      this->read_state_ = IDLE;
      this->read_index_ = 0;
      this->process_reads();  // processer køen (læser fra reads_ og opdaterer sensorer)
      return;
    }

    // Timeout check
    if (millis() - this->read_start_ms_ > this->read_timeout_ms_) {
      ESP_LOGW(TAG, "Timeout waiting for Modbus response reg=0x%04X", this->read_reg_);
      this->read_state_ = IDLE;
      return;
    }
  }
}

void CustomModbus::process_reads() {
  // Gennemgå reads_ køen og publicer tilknyttede sensorer
  // Forventet struktur: reads_ er en vector<ReadItem> hvor ReadItem indeholder
  // reg, count, optional pointerer til binary_sensor/text_sensor/other
  // Vi antager at ReadItem har felter: binary_sensor, text_sensor, sensor osv.

  for (auto &it : this->reads_) {
    // Eksempel: hvis der er en binary_sensor tilknyttet
    if (it.binary_sensor != nullptr) {
      // Bestem state ud fra modbus-data i read_buf_
      // Her antager vi at første dataord er MSB/LSB i read_buf_ efter bytecount
      bool state = false;
      if (this->read_expected_ >= 5) {
        // Eksempel: hvis data byte 0 ikke er 0 => true
        state = (this->read_buf_[3] != 0);
      }
      it.binary_sensor->publish_state(state);
    }

    // Tekst-sensor eksempel (konverter bytes til string)
    if (it.text_sensor != nullptr) {
      // Konverter data bytes til std::string (enkelt eksempel)
      std::string buf;
      // Bytecount er i read_buf_[2]
      uint8_t bytecount = this->read_buf_[2];
      for (uint8_t i = 0; i < bytecount && (3 + i) < this->read_expected_ - 2; i++) {
        char c = static_cast<char>(this->read_buf_[3 + i]);
        if (c == '\0')
          break;
        buf.push_back(c);
      }
      it.text_sensor->publish_state(std::string(buf));
    }

    // Andre sensortyper kan håndteres tilsvarende (float, int osv.)
    if (it.sensor != nullptr) {
      // Eksempel: læs et 16-bit register og publicer som float/skalering
      if (this->read_expected_ >= 5) {
        uint16_t val = (static_cast<uint16_t>(this->read_buf_[3]) << 8) | static_cast<uint16_t>(this->read_buf_[4]);
        float scaled = static_cast<float>(val) / it.scale;  // antag ReadItem har scale
        it.sensor->publish_state(scaled);
      }
    }
  }

  // Ryd køen efter behandling
  this->reads_.clear();
}

void CustomModbus::loop() {
  // Kald state handler
  this->handle_read_state();

  // Hvis vi ikke venter på svar og der er reads i køen, start næste læsning
  if (this->read_state_ == IDLE && !this->reads_.empty()) {
    // Tag første item
    ReadItem r = this->reads_.front();
    this->reads_.erase(this->reads_.begin());

    // Byg Modbus-forespørgsel (slave + fn + reg_hi + reg_lo + count_hi + count_lo + crc_lo + crc_hi)
    uint8_t frame[8];
    frame[0] = static_cast<uint8_t>(this->slave_id_);
    frame[1] = 0x03;  // read holding registers
    frame[2] = static_cast<uint8_t>((r.reg >> 8) & 0xFF);
    frame[3] = static_cast<uint8_t>(r.reg & 0xFF);
    frame[4] = static_cast<uint8_t>((r.count >> 8) & 0xFF);
    frame[5] = static_cast<uint8_t>(r.count & 0xFF);
    uint16_t crc = modbus_crc16(frame, 6);
    frame[6] = static_cast<uint8_t>(crc & 0xFF);       // CRC low
    frame[7] = static_cast<uint8_t>((crc >> 8) & 0xFF); // CRC high

    // Send forespørgsel via UART
    if (this->uart_parent_) {
      // Hvis du har en DE-pin og ønsker at styre den manuelt, kan du aktivere her:
      // digitalWrite(this->de_pin_, HIGH); // TX enable
      this->uart_parent_->write_array(frame, 8);
      this->uart_parent_->flush();
      // digitalWrite(this->de_pin_, LOW); // tilbage til RX (hvis manuel DE)
    }

    // Forbered læse-state
    this->read_state_ = WAITING;
    this->read_start_ms_ = millis();
    this->read_expected_ = static_cast<uint8_t>(5 + r.count * 2); // slave + fn + bytecount + data + crc_lo + crc_hi
    this->read_got_ = 0;
    this->read_reg_ = r.reg;
    this->read_count_ = r.count;
    // read_buf_ nulstilles ikke nødvendigvis, men vi sætter read_got_ til 0 så vi overskriver
  }
}

}  // namespace custommodbus
}  // namespace esphome
