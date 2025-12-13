#enum DataType {
  TYPE_UINT16 = 0,
  TYPE_INT16 = 1,
};

class CustomModbus : public esphome::Component, public esphome::uart::UARTDevice {
 public:
  void set_slave_id(uint8_t id) { slave_id_ = id; }
  void set_register(uint16_t reg) { register_ = reg; }
  void set_count(uint8_t c) { count_ = c; }

  // IMPORTANT FIX:
  void set_data_type(int t) { data_type_ = static_cast<DataType>(t); }

  void set_scale(float s) { scale_ = s; }
  void set_sensor(esphome::sensor::Sensor *s) { sensor_ = s; }
