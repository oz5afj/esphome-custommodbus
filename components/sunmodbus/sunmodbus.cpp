#include "custommodbus.h"


#include "esphome/core/log.h"



namespace esphome {


namespace custommodbus {




static const char *TAG = "custommodbus";




void CustomModbus::setup() {


  ESP_LOGI(TAG, "CustomModbus setup: slave=%d start=%d count=%d",


           slave_id_, start_address_, count_);

}




void CustomModbus::add_sensor(sensor::Sensor *s, uint8_t offset, DataType type, float scale) {


  sensors_.push_back({s, offset, type, scale});

}




void CustomModbus::loop() {


  const uint32_t now = millis();


  if (now - last_read_ < update_interval_) return;


  last_read_ = now;





  // Request block


  uint8_t frame[8];


  frame[0] = slave_id_;


  frame[1] = 0x03;


  frame[2] = start_address_ >> 8;


  frame[3] = start_address_ & 0xFF;


  frame[4] = count_ >> 8;


  frame[5] = count_ & 0xFF;





  // CRC


  uint16_t crc = 0xFFFF;


  for (int i = 0; i < 6; i++) {


    crc ^= frame[i];


    for (int j = 0; j < 8; j++)


      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);


  }


  frame[6] = crc & 0xFF;


  frame[7] = crc >> 8;





  this->write_array(frame, 8);





  delay(50);





  const int expected = 3 + count_ * 2;


  std::vector<uint8_t> resp(expected);


  int len = this->read_array(resp.data(), expected);




  if (len != expected) {


    ESP_LOGW(TAG, "Invalid response length: %d != %d", len, expected);



    return;

  }




  for (auto &e : sensors_) {


    int idx = 3 + e.offset * 2;


    uint16_t raw = (resp[idx] << 8) | resp[idx + 1];





    float value = 0;


    if (e.type == TYPE_UINT16)


      value = raw * e.scale;


    else


      value = int16_t(raw) * e.scale;




    e.sensor->publish_state(value);


  }



}




}  // namespace custommodbus

}  // namespace esphome
