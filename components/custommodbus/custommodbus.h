#pragma once

#include "esphome.h"


#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include <vector>
#include <algorithm>
#include <map>   // <-- NYT til EEPROM-safe write tracking

// Forward declarations for binary/text sensors to avoid requiring their headers here
namespace esphome {
namespace binary_sensor { class BinarySensor; }
namespace text_sensor { class TextSensor; }
}  // namespace esphome

namespace esphome {
namespace custommodbus {

enum DataType : uint8_t {
  TYPE_UINT16 = 0,
  TYPE_INT16 = 1,
  TYPE_UINT32 = 2,
  TYPE_UINT32_R = 3,
};

struct ReadItem {
  uint16_t reg;
  uint8_t count;
  DataType type;
  float scale;
  esphome::sensor::Sensor *sensor;
  esphome::binary_sensor::BinarySensor *binary_sensor;
  esphome::text_sensor::TextSensor *text_sensor;
  uint16_t bitmask;
};

struct WriteItem {
  uint16_t reg;
  uint16_t value;
  uint16_t mask;
  bool use_mask;
};

struct ReadBlock {
  uint16_t start_reg;
  uint8_t count;
  std::vector<ReadItem*> items;
};

// Bemærk: brug uart::-navnerummet for at få de korrekte typer
class CustomModbus : public Component, public uart::UARTDevice {
 public:
  CustomModbus() = default;

  // Binding fra Python: ESPHome genererer kaldet set_uart_parent
  void set_uart_parent(uart::UARTComponent *parent) { this->uart_parent_ = parent; }
  void set_slave_id(uint8_t id) { this->slave_id_ = id; }

  // Valgfri grouped-reads
  void set_use_grouped_reads(bool v) { this->use_grouped_reads_ = v; }

  // API som Python kalder
  void add_read_sensor(uint16_t reg, uint8_t count, DataType type, float scale, esphome::sensor::Sensor *s);
  void add_read_sensor(uint16_t reg, uint8_t count, uint8_t type_as_int, float scale, esphome::sensor::Sensor *s);

  void add_binary_sensor(uint16_t reg, uint16_t mask, esphome::binary_sensor::BinarySensor *bs);
  void add_text_sensor(uint16_t reg, esphome::text_sensor::TextSensor *ts);

  void write_single(uint16_t reg, uint16_t value);
  void write_bitmask(uint16_t reg, uint16_t mask, bool state);

  // Lifecycle
  void setup() override;
  void loop() override;

 protected:
  // loop begrænsning test 
  uint32_t last_publish_ms_{0};
  static constexpr uint32_t PUBLISH_INTERVAL_MS = 2000; // 2s
  float last_published_value_{NAN};
  float publish_deadband_ = 0.05f; // juster efter behov
  // loop begrænsing test slut
  void process_reads();
  void process_writes();
  bool read_registers(uint16_t reg, uint8_t count, uint8_t *resp, uint8_t &resp_len);
  uint16_t crc16(uint8_t *buf, uint8_t len);

  void build_read_blocks();

  // uart::UARTComponent pointer type (sættes af ESPHome via set_uart_parent)
  uart::UARTComponent *uart_parent_{nullptr};
  uint8_t slave_id_{1};

  // Læse- og skrivekøer
  std::vector<ReadItem> reads_;
  std::vector<WriteItem> writes_;
  std::vector<ReadBlock> blocks_;

  bool use_grouped_reads_{false};

  // --- EEPROM-safe write tracking (NYT) ---
  std::map<uint16_t, uint16_t> last_written_value_;
  std::map<uint16_t, uint32_t> last_write_time_;
  static const uint32_t WRITE_COOLDOWN_MS = 30000; // 30 seconds

  // --- Asynkrone read‑state medlemmer ---
  enum ReadState { IDLE = 0, WAITING = 1, PROCESSING = 2 };

  ReadState read_state_{IDLE};
  uint32_t read_start_ms_{0};
  uint32_t read_timeout_ms_{1000};
  uint8_t read_expected_{0};
  uint8_t read_buf_[64];
  uint8_t read_got_{0};
  uint16_t read_reg_{0};
  uint8_t read_count_{0};
  size_t read_index_{0};

  // (Beholdes for kompatibilitet – ikke brugt i den nye kode)
  void start_read(uint16_t reg, uint8_t count);
  void handle_read_state();

  // --- EEPROM-safe helpers (NYT) ---
  bool should_write(uint16_t reg, uint16_t value);
  void record_write(uint16_t reg, uint16_t value);

  // preiss - START: indsæt nye medlemmer og prototype for filtreret publicering
  // Map til at huske sidste publicerede værdi per sensor (for at undgå oversvømmelse)
  std::map<esphome::sensor::Sensor*, float> last_published_values_;
  // Tærskel for hvornår vi sender en ændring (1 i 3. decimal = 0.001)
  float publish_delta_threshold_ = 0.001f;
  // Hjælpefunktion: afrund og publicer kun hvis ændring > threshold
  void publish_sensor_filtered(esphome::sensor::Sensor *sensor, float value);
  // preiss - END




};

}  // namespace custommodbus
}  // namespace esphome





