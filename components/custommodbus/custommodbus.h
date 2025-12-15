#pragma once
#include "esphome.h"

// Forward declarations for platform types to avoid direct includes
namespace esphome {
namespace sensor {
class Sensor;
}  // namespace sensor

namespace binary_sensor {
class BinarySensor;
}  // namespace binary_sensor

namespace text_sensor {
class TextSensor;
}  // namespace text_sensor
}  // namespace esphome

namespace esphome {
namespace custommodbus {

// DataType enum (skal matche hvad sensor.py sender)
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

class CustomModbus : public Component, public UARTDevice {
 public:
  CustomModbus() = default;

  // Binding fra Python
  void set_uart_parent(UARTComponent *parent) { this->uart_parent_ = parent; }
  void set_slave_id(uint8_t id) { this->slave_id_ = id; }

  // API som Python kalder
  void add_read_sensor(uint16_t reg, uint8_t count, DataType type, float scale, esphome::sensor::Sensor *s);
  void add_binary_sensor(uint16_t reg, uint16_t mask, esphome::binary_sensor::BinarySensor *bs);
  void add_text_sensor(uint16_t reg, esphome::text_sensor::TextSensor *ts);

  void write_single(uint16_t reg, uint16_t value);
  void write_bitmask(uint16_t reg, uint16_t mask, bool state);

  // Lifecycle
  void setup() override;
  void loop() override;

 protected:
  // Internals
  void process_reads();
  void process_writes();
  bool read_registers(uint16_t reg, uint8_t count, uint8_t *resp, uint8_t &resp_len);
  uint16_t crc16(uint8_t *buf, uint8_t len);

  UARTComponent *uart_parent_{nullptr};
  uint8_t slave_id_{1};

  std::vector<ReadItem> reads_;
  std::vector<WriteItem> writes_;
};
}  // namespace custommodbus
}  // namespace esphome
