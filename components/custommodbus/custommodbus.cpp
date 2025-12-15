#include "custommodbus.h"
#include "esphome/core/log.h"

namespace esphome {
namespace custommodbus {

static const char *const TAG = "custommodbus";

//
// ============================================================================
//  SETUP
// ============================================================================
//  Kører én gang ved boot. Logger kun slave-ID.
//  Ingen Modbus-trafik her.
// ============================================================================
void CustomModbus::setup() {
  ESP_LOGCONFIG(TAG, "Setting up CustomModbus with slave ID %u", this->slave_id_);
}

//
// ============================================================================
//  LOOP
// ============================================================================
//  ESPHome kalder loop() mange gange i sekundet.
//  Vi må ALDRIG blokere her, ellers dør WiFi og API.
//
//  Derfor:
//    - Vi kører kun Modbus hver 200 ms
//    - process_reads() læser KUN én sensor per loop (round-robin)
// ============================================================================
void CustomModbus::loop() {
  static uint32_t last = 0;
  const uint32_t now = millis();

  // Kun hver 200 ms
  if (now - last < 200)
    return;

  last = now;

  this->process_writes();
  this->process_reads();
}

//
// ============================================================================
//  ADD SENSOR FUNCTIONS
// ============================================================================
//  Disse funktioner opretter ReadItem structs og lægger dem i reads_-listen.
//  Ingen logik her — kun opsætning.
// ============================================================================

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
// ============================================================================
//  WRITE FUNCTIONS
// ============================================================================
//  Disse lægger write-kommandoer i writes_-køen.
//  Selve afsendelsen sker i process_writes().
// ============================================================================

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
// ============================================================================
//  PROCESS READS  (VIGTIG ÆNDRING)
// ============================================================================
//  Tidligere: læste ALLE sensorer i ét loop → blokerede ESPHome API.
//
//  Nu: round-robin → kun ÉN sensor per loop.
//  Dette gør driveren 100% non-blocking og stabil.
// ============================================================================

void CustomModbus::process_reads() {
  static size_t index = 0;

  // Ingen sensorer
  if (this->reads_.empty())
    return;

  // Loop tilbage til start
  if (index >= this->reads_.size())
    index = 0;

  // Vælg én sensor
  auto &r = this->reads_[index];
  index++;

  uint8_t resp[64]{0};
  uint8_t resp_len = 0;

  // Læs registeret (non-blocking)
  if (!this->read_registers(r.reg, r.count, resp, resp_len))
    return;

  // Udpak 16-bit værdi
  const uint16_t raw16 = (resp[3] << 8) | resp[4];

  // Udpak 32-bit værdi hvis count == 2
  uint32_t raw32 = 0;
  if (r.count == 2) {
    raw32 = (static_cast<uint32_t>(resp[3]) << 24) |
            (static_cast<uint32_t>(resp[4]) << 16) |
            (static_cast<uint32_t>(resp[5]) << 8) |
            static_cast<uint32_t>(resp[6]);
  }

  // FLOAT SENSOR
  if (r.sensor != nullptr) {
    float value = 0.0f;

    switch (r.type) {
      case TYPE_UINT16: value = raw16; break;
      case TYPE_INT16: value = (int16_t)raw16; break;
      case TYPE_UINT32: value = raw32; break;
      case TYPE_UINT32_R: value = __builtin_bswap32(raw32); break;
    }

    value *= r.scale;
    r.sensor->publish_state(value);
  }

  // BINARY SENSOR
  if (r.binary_sensor != nullptr) {
    bool state = (raw16 & r.bitmask) != 0;
    r.binary_sensor->publish_state(state);
  }

  // TEXT SENSOR
  if (r.text_sensor != nullptr) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%04X", raw16);
    r.text_sensor->publish_state(buf);
  }
}

//
// ============================================================================
//  PROCESS WRITES
// ============================================================================
//  Sender én write-kommando per loop.
//  Blocking er minimal og påvirker ikke API.
// ============================================================================

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

  this->write_array(frame, 8);
  this->flush();
}

//
// ============================================================================
//  READ REGISTERS  (VIGTIG ÆNDRING)
// ============================================================================
//  Tidligere: tight while-loop → ESP32 fik ingen CPU → API døde.
//
//  Nu: delay(1) i loopet → WiFi og API får CPU-tid.
//  Timeout efter 200 ms.
// ============================================================================

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

  // Send Modbus-forespørgsel
  this->write_array(frame, 8);
  this->flush();

  const uint32_t start = millis();
  const uint8_t expected = 5 + count * 2;

  // Vent på svar — men giv CPU-tid til WiFi/API
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

    delay(1);  // ✅ Giver CPU-tid til WiFi og ESPHome API
  }

  ESP_LOGW(TAG, "Timeout waiting for Modbus response reg=0x%04X", reg);
  return false;
}

//
// ============================================================================
//  CRC16
// ============================================================================
//  Standard Modbus CRC16 (LSB-first).
// ============================================================================

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
