#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace sunmodbus {

class SunModbus : public PollingComponent, public uart::UARTDevice {
 public:
  SunModbus() = default;

  void set_uart(uart::UARTComponent *uart) { this->uart_ = uart; }
  void set_slave_id(uint8_t slave_id) { this->slave_id_ = slave_id; }
  void set_start_address(uint16_t start_address) { this->start_address_ = start_address; }

  void set_update_interval(uint32_t update_interval) { this->update_interval_ = update_interval; }

  void set_sensor(uint8_t index, sensor::Sensor *sens) {
    if (index < 10) this->sensors_[index] = sens;
  }

  void setup() override;
  void update() override;

 protected:
  uart::UARTComponent *uart_{nullptr};
  uint8_t slave_id_{1};
  uint16_t start_address_{598};

  sensor::Sensor *sensors_[10]{nullptr};

  bool read_block_(uint8_t slave, uint16_t start, uint8_t count, uint8_t *resp, uint16_t len);
};

}  // namespace sunmodbus
}  // namespace esphome
