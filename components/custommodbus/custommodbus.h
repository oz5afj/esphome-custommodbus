#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor_component.h"
#include "esphome/components/switch/switch_component.h"
#include "esphome/components/number/number_component.h"
#include "esphome/components/text_sensor/text_sensor_component.h"
#include "esphome/components/select/select_component.h"

namespace esphome {
namespace custommodbus {

class CustomModbus : public Component, public uart::UARTDevice {
 public:
  void set_slave_id(uint8_t id) { this->slave_id_ = id; }
  uint8_t slave_id() const { return this->slave_id_; }

  // Sensor
  void add_sensor(sensor::Sensor *sens) { this->sensors_.push_back(sens); }

  // Switch
  void add_switch(switch_::Switch *sw) { this->switches_.push_back(sw); }

  // Number
  void add_number(number::Number *num) { this->numbers_.push_back(num); }

  // Text sensor
  void add_text_sensor(text_sensor::TextSensor *ts) { this->text_sensors_.push_back(ts); }

  // Select
  void add_select(select::Select *sel) { this->selects_.push_back(sel); }

  void setup() override;
  void loop() override;

 protected:
  uint8_t slave_id_;

  std::vector<sensor::Sensor *> sensors_;
  std::vector<switch_::Switch *> switches_;
  std::vector<number::Number *> numbers_;
  std::vector<text_sensor::TextSensor *> text_sensors_;
  std::vector<select::Select *> selects_;

  // Helper functions til Modbus læs/skriv
  void write_single(uint16_t reg, uint16_t value);
  void write_bitmask(uint16_t reg, uint16_t bitmask, bool state);
};

}  // namespace custommodbus
}  // namespace esphome
