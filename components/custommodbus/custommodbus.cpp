#include "custommodbus.h"

// Kun sensor header inkluderes her — binary/text er midlertidigt udeladt
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace custommodbus {

static const char *const TAG = "custommodbus";

namespace esphome {
namespace custommodbus {

static const char *const TAG = "custommodbus";

//
// ============================================================================
//  SETUP
// ============================================================================
//  Kører én gang efter boot. Her logger vi, at komponenten er sat op.
//  Ingen Modbus-trafik her.
// ============================================================================
void CustomModbus::setup() {
  ESP_LOGCONFIG(TAG, "Setting up CustomModbus with slave ID %u", this->slave_id_);
}

//
// ============================================================================
//  LOOP
// ============================================================================
//  ESPHome kalder loop() ofte. Vi må ALDRIG lave tungt eller blokkerende arbejde.
//  Vi gør derfor:
//    - Kun én iteration hver 200 ms
//    - process_writes() håndterer max én write ad gangen
//    - process_reads() håndterer KUN én read per loop (round-robin)
// ============================================================================
void CustomModbus::loop() {
  static uint32_t last = 0;
  const uint32_t now = millis();

  // Begræns polling til hver 200 ms
  if (now - last < 200)
    return;

  last = now;

  this->process_writes();
  this->process_reads();
}

//
// ============================================================================
//  REGISTRERING AF READS
// ============================================================================
//  Disse funktioner kaldes under konfiguration (fra Python/YAML) for at
//  oprette ReadItem og lægge dem i reads_-vektoren.
//  De laver IKKE Modbus-opkald selv.
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
//  QUEUE WRITES
// ============================================================================
//  Disse funktioner tilføjer bare WriteItems til køen.
//  Selve Modbus‑sende‑logikken ligger i process_writes().
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
//  PROCESS READS  (Round-robin, én sensor per loop)
// ============================================================================
//  Tidligere var her et for-loop over ALLE reads_ → kunne blokere længe.
//
//  Nu:
//    - static index holder styr på hvor vi er i listen
//    - hver loop() kaldes kun én ReadItem
//    - API og WiFi får masser af CPU-tid
// ============================================================================
void CustomModbus::process_reads() {
  static size_t index = 0;

  if (this->reads_.empty())
    return;

  // Wrap index hvis vi når slutningen
  if (index >= this->reads_.size())
    index = 0;

  // Vælg næste ReadItem i round-robin
  auto &r = this->reads_[index];
  index++;

  uint8_t resp[64]{0};
  uint8_t resp_len = 0;

  // Kald read_registers() (non-blocking i tid og CPU)
  if (!this->read_registers(r.reg, r.count, resp, resp_len))
    return;  // Timeout eller fejl, vi logger i read_registers()

  // Udpak 16-bit værdi (første register)
  const uint16_t raw16 = (resp[3] << 8) | resp[4];

  // Udpak 32-bit, hvis der er to registre
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
    }

    value *= r.scale;
    r.sensor->publish_state(value);
  }

  // BINARY SENSOR
  if (r.binary_sensor != nullptr) {
    const bool state = (raw16 & r.bitmask) != 0;
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
//  Sender én Modbus "Write Single Register" per loop.
//  Blocking her er minimal og påvirker ikke ESPHome API væsentligt.
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
  // På sigt kan du lave "read-modify-write" her, hvis use_mask = true

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
//  READ REGISTERS (Non-blocking, med delay(1))
// ============================================================================
//  Sender en Modbus "Read Holding Registers (0x03)" forespørgsel og
//  venter op til 200 ms på svar.
//
//  VIGTIGT:
//    - Vi bruger while(millis - start < 200) + delay(1)
//    - Ingen stram while(available() < expected)-loop
//    - Giver CPU-tid til WiFi og ESPHome API
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

  // Send forespørgsel
  this->write_array(frame, 8);
  this->flush();

  const uint32_t start = millis();
  const uint8_t expected = static_cast<uint8_t>(5 + count * 2);

  while (millis() - start < 200) {
    int avail = this->available();

    if (avail >= expected) {
      // Vi har nok bytes, læs hele svaret
      this->read_array(resp, expected);
      resp_len = expected;
      return true;
    }

    if (avail > 0 && avail < expected) {
      // Der er data, men ikke nok til et komplet svar
      ESP_LOGW(TAG, "Partial Modbus response: %d/%d bytes", avail, expected);
    }

    // Giv CPU-tid til andre tasks (WiFi, API, etc.)
    delay(1);
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


