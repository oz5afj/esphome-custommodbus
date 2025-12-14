#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/select/select.h"
#include "esphome/components/switch/switch.h"
#include "esphome/core/log.h"

namespace esphome {
namespace custommodbus {

// Datatyper til sensorer
enum DataType {
  TYPE_UINT16,
  TYPE_INT16,
  TYPE_UINT32,
  TYPE_UINT32_R
};

// Read-item til sensorer, binary, text
struct ReadItem {
  uint16_t reg;
  uint8_t count;
  DataType type;
  float scale;
  sensor::Sensor *sensor;
  binary_sensor::BinarySensor *binary_sensor;
  text_sensor::TextSensor *text_sensor;
  uint16_t bitmask;
};

// Write-item til single eller bitmask
struct WriteItem {
  uint16_t reg;
  uint16_t value;
  uint16_t mask;
  bool use_mask;
};

class CustomModbus : public Component, public uart::UARTDevice {
 public:
  void set_slave_id(uint8_t id) { this->slave_id_ = id; }
  uint8_t slave_id() const { return this->slave_id_; }

  // Tilføj læseobjekter
  void add_read_sensor(uint16_t reg, uint8_t count, DataType type,
                       float scale, sensor::Sensor *s);
  void add_binary_sensor(uint16_t reg, uint16_t mask, binary_sensor::BinarySensor *bs);
  void add_text_sensor(uint16_t reg, text_sensor::TextSensor *ts);

  // Tilføj skriveobjekter
  void write_single(uint16_t reg, uint16_t value);
  void write_bitmask(uint16_t reg, uint16_t mask, bool state);

  void setup() override;
  void loop() override;

 protected:
  uint8_t slave_id_{1};
  std::vector<ReadItem> reads_;
  std::vector<WriteItem> writes_;

  // Interne funktioner
  void process_reads();
  void process_writes();
  bool read_registers(uint16_t reg, uint8_t count, uint8_t *resp, uint8_t &resp_len);
  uint16_t crc16(uint8_t *buf, uint8_t len);
};

}  // namespace custommodbus
}  // namespace esphome

