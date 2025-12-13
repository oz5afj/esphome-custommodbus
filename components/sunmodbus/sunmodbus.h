#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace sunmodbus {

enum DataType {
  TYPE_UINT16 = 0,
  TYPE_INT16 = 1,
};

class SunModbus : public PollingComponent, public uart::UARTDevice {
 public:
  SunModbus() = default;

  void set_uart(uart::UARTComponent *uart) { this->uart_ = uart; }
  void set_sensor(sensor::Sensor *sensor) { this->sensor_ = sensor; }

  void set_slave_id(uint8_t slave_id) { this->slave_id_ = slave_id; }
  void set_start_address(uint16_t start_address) { this->start_address_ = start_address; }
  void set_count(uint16_t count) { this->count_ = count; }
  void set_offset(uint16_t offset) { this->offset_ = offset; }
  void set_scale(float scale) { this->scale_ = scale; }
  void set_type(DataType type) { this->type_ = type; }

  // ✅ KORREKT måde i 2025.11.x
  void set_update_interval(uint32_t update_interval) { this->update_interval_ = update_interval; }

  void setup() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  uart::UARTComponent *uart_{nullptr};
  sensor::Sensor *sensor_{nullptr};

  uint8_t slave_id_{1};
  uint16_t start_address_{0};
  uint16_t count_{1};
  uint16_t offset_{0};
  float scale_{1.0f};
  DataType type_{TYPE_UINT16};

  uint32_t update_interval_{1000};

  bool read_holding_registers_(uint8_t slave, uint16_t start, uint16_t count, uint8_t *buffer, uint16_t len);
};

}  // namespace sunmodbus
}  // namespace esphome
