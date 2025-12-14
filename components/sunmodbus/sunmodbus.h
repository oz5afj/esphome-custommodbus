#pragma once



#include "esphome/components/uart/uart.h"

#include "esphome/core/component.h"





namespace esphome {


namespace custommodbus {



enum DataType {

  TYPE_UINT16,

  TYPE_INT16,

};




class CustomModbus : public Component, public uart::UARTDevice {

 public:


  void set_slave_id(uint8_t id) { slave_id_ = id; }


  void set_start_address(uint16_t addr) { start_address_ = addr; }


  void set_count(uint16_t c) { count_ = c; }


  void set_update_interval(uint32_t ms) { update_interval_ = ms; }





  void add_sensor(sensor::Sensor *s, uint8_t offset, DataType type, float scale);




  void setup() override;

  void loop() override;












 protected:





  uint8_t slave_id_;

  uint16_t start_address_;

  uint16_t count_;




  uint32_t update_interval_;


  uint32_t last_read_{0};





  struct SensorEntry {


    sensor::Sensor *sensor;


    uint8_t offset;


    DataType type;


    float scale;


  };





  std::vector<SensorEntry> sensors_;

};




}  // namespace custommodbus

}  // namespace esphome
