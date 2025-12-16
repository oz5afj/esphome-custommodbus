#pragma once

#include "esphome.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"

// NOTE: binary_sensor and text_sensor support temporarily removed to avoid missing-header build errors.

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
  // binary/text sensors removed for now
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

  void set_uart_parent(UARTComponent *parent) { this->uart_parent_ = parent; }
  void set_slave_id(uint8_t id) { this->slave_id_ = id; }

  // Kun float sensors for nu
  void add_read_sensor(uint16_t reg, uint8_t count, DataType type, float scale, esphome::sensor::Sensor *s);

  // Write API
  void write_single(uint16_t reg, uint16_t value);
  void write_bitmask(uint16_t reg, uint16_t mask, bool state);

  void setup() override;
  void loop() override;

 protected:
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
