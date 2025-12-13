#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace sunmodbus {

// samme enum som Python refererer til
enum class DataType {
  TYPE_UINT16 = 0,
  TYPE_INT16 = 1,
};

class SunModbus : public PollingComponent, public uart::UARTDevice {
 public:
  SunModbus() : PollingComponent(1000) {}  // default 1s, overskrives af set_update_interval hvis du vil

  void setup() override;
  void update() override;

  void set_uart(uart::UARTComponent *uart) { this->parent_ = uart; }
  void set_slave_id(uint8_t id) { this->slave_id_ = id; }
  void set_start_address(uint16_t addr) { this->start_address_ = addr; }
  void set_count(uint16_t count) { this->count_ = count; }
  void set_update_interval(uint32_t ms) { this->set_update_interval_ms(ms); }

  void set_offset(uint16_t offset) { this->offset_ = offset; }
  void set_type(DataType type) { this->type_ = type; }
  void set_scale(float scale) { this->scale_ = scale; }

 protected:
  bool read_registers(uint16_t start, uint16_t count, uint16_t *buffer);

  uart::UARTComponent *parent_{nullptr};
  sensor::Sensor *sensor_{nullptr};

  uint8_t slave_id_{1};
  uint16_t start_address_{0};
  uint16_t count_{1};
  uint16_t offset_{0};
  DataType type_{DataType::TYPE_UINT16};
  float scale_{1.0f};
};

}  // namespace sunmodbus
}  // namespace esphome
