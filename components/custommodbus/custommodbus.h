#pragma once

#include "esphome.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include <vector>

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

// Bemærk: brug uart::-navnerummet for at få de korrekte typer
class CustomModbus : public Component, public uart::UARTDevice {
 public:
  CustomModbus() = default;

  // Binding fra Python: ESPHome genererer kaldet set_uart_parent
  void set_uart_parent(uart::UARTComponent *parent) { this->uart_parent_ = parent; }
  void set_slave_id(uint8_t id) { this->slave_id_ = id; }

  // API som Python kalder
  void add_read_sensor(uint16_t reg, uint8_t count, DataType type, float scale, esphome::sensor::Sensor *s);
  // Overload for bagudkompatibilitet med auto-genereret main.cpp (sender et tal for data_type)
  void add_read_sensor(uint16_t reg, uint8_t count, uint8_t type_as_int, float scale, esphome::sensor::Sensor *s);

  void add_binary_sensor(uint16_t reg, uint16_t mask, esphome::binary_sensor::BinarySensor *bs);
  void add_text_sensor(uint16_t reg, esphome::text_sensor::TextSensor *ts);

  void write_single(uint16_t reg, uint16_t value);
  void write_bitmask(uint16_t reg, uint16_t mask, bool state);

  // Lifecycle
  void setup() override;
  void loop() override;

 protected:
  void process_reads();
  void process_writes();
  bool read_registers(uint16_t reg, uint8_t count, uint8_t *resp, uint8_t &resp_len);
  uint16_t crc16(uint8_t *buf, uint8_t len);

  // uart::UARTComponent pointer type (sættes af ESPHome via set_uart_parent)
  uart::UARTComponent *uart_parent_{nullptr};
  uint8_t slave_id_{1};

  // Læse- og skrivekøer
  std::vector<ReadItem> reads_;
  std::vector<WriteItem> writes_;

  // --- Asynkrone read‑state medlemmer (bruges af custommodbus.cpp) ---
  enum ReadState { IDLE = 0, WAITING = 1, PROCESSING = 2 };

  ReadState read_state_{IDLE};
  uint32_t read_start_ms_{0};
  uint32_t read_timeout_ms_{1000}; // default timeout i ms (kan justeres)
  uint8_t read_expected_{0};
  uint8_t read_buf_[64];
  uint8_t read_got_{0};
  uint16_t read_reg_{0};
  uint8_t read_count_{0};
  size_t read_index_{0};

  // Asynkrone read helper metoder (deklarationer)
  void start_read(uint16_t reg, uint8_t count);
  void handle_read_state();
};

}  // namespace custommodbus
}  // namespace esphome
