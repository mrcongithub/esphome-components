// components/custom_watertank_sensor/custom_watertank_sensor.h
#pragma once
#include "esphome.h"

class CustomWatertankSensor : public Component, public UARTDevice {
 public:
  CustomWatertankSensor(UARTComponent *parent) : UARTDevice(parent) {}

  Sensor *distance_sensor = new Sensor();
  Sensor *measurements_sensor = new Sensor();
  float update_interval;

  void setup() override {
    last_update_ = millis();
  }

  void loop() override {
    static std::vector<uint8_t> buffer;

    while (available()) {
      buffer.push_back(read());
    }

    while (buffer.size() >= 4) {
      if (buffer[0] != 0xFF) {
        buffer.erase(buffer.begin());
        continue;
      }
      if (buffer.size() < 4) break;
      uint8_t sum = (buffer[0] + buffer[1] + buffer[2]) & 0xFF;
      if (sum == buffer[3]) {
        float dist = (buffer[1] << 8) + buffer[2];
        distances_.push_back(dist);
        buffer.erase(buffer.begin(), buffer.begin() + 4);
      } else {
        ESP_LOGW("CustomWatertankSensor", "Invalid checksum at index 0");
        buffer.erase(buffer.begin());
      }
    }

    unsigned long now = millis();
    if (now - last_update_ >= update_interval) {
      if (distances_.size() >= 5) {
        float sum_dist = 0;
        for (float d : distances_) {
          sum_dist += d;
        }
        float final_dist = sum_dist / distances_.size() / 10.0;
        distance_sensor->publish_state(round(final_dist * 10) / 10.0);
        measurements_sensor->publish_state(distances_.size());
        distances_.clear();
      }
      last_update_ = now;
    }
  }

  void set_update_interval(float interval) { update_interval = interval * 1000; }

 private:
  std::vector<float> distances_;
  unsigned long last_update_;
};
