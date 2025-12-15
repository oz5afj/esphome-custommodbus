#pragma once
#include "esphome.h"

// Brug forward-deklarationer i stedet for at inkludere alle platform-headers
namespace esphome {
namespace sensor {
class Sensor;
}  // namespace sensor
}  // namespace esphome

namespace esphome {
namespace custommodbus {

struct ReadSensorEntry {
  uint16_t reg;
  uint8_t count;
  uint8_t dtype;
  float scale;
  esphome::sensor::Sensor *sensor;
};

class CustomModbus : public Component, public UARTDevice {
 public:
  CustomModbus() = default;

  // Binding fra Python
  void set_uart_parent(UARTComponent *parent) { this->uart_parent_ = parent; }
  void set_slave_id(uint8_t id) { this->slave_id_ = id; }

  // API som sensor.py kalder
  void add_read_sensor(uint16_t reg, uint8_t count, uint8_t dtype, float scale, esphome::sensor::Sensor *sensor);

  // Lifecycle
  void setup() override;
  void loop() override;

 protected:
  // Hjælpemetoder
  void process_loop();
  bool send_read_request(uint16_t reg, uint8_t count);
  bool read_response(std::vector<uint8_t> &resp, uint32_t timeout_ms = 200);
  uint16_t crc16(const uint8_t *buf, size_t len);
  float decode_value(const std::vector<uint8_t> &data, uint8_t dtype, uint8_t count, float scale);

  UARTComponent *uart_parent_{nullptr};
  uint8_t slave_id_{1};

  std::vector<ReadSensorEntry> read_entries_;
  unsigned long last_request_ms_{0};
  size_t current_index_{0};
  unsigned long request_interval_ms_{1000};
};

}  // namespace custommodbus
}  // namespace esphome
