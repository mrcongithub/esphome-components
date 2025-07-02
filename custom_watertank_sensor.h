// \config\esphome\custom_components\custom_watertank_sensor.h
#pragma once
#include "esphome.h"

class CustomWaterTankSensor : public Component, public UARTDevice {
 public:
  CustomWaterTankSensor(UARTComponent *parent) : UARTDevice(parent) {}

  // Link to the sensors for publishing data
  Sensor *distance_sensor = new Sensor();
  Sensor *measurements_sensor = new Sensor();

  void setup() override {
    // Initialize if needed
  }

  void loop() override {
    // Buffer to store incoming data
    static std::vector<uint8_t> buffer;

    // Read available UART data
    while (available()) {
      buffer.push_back(read());
    }

    // Process data when enough bytes are available
    while (buffer.size() >= 4) {
      if (buffer[0] != 0xFF) {
        // Skip bytes until we find the header
        buffer.erase(buffer.begin());
        continue;
      }

      // Ensure we have a complete packet
      if (buffer.size() < 4) break;

      // Calculate checksum
      uint8_t sum = (buffer[0] + buffer[1] + buffer[2]) & 0xFF;
      if (sum == buffer[3]) {
        // Calculate distance
        float dist = (buffer[1] << 8) + buffer[2];
        distances_.push_back(dist);
        buffer.erase(buffer.begin(), buffer.begin() + 4); // Remove processed packet
      } else {
        // Invalid checksum, skip the first byte
        buffer.erase(buffer.begin());
      }
    }

    // Publish average distance if enough measurements
    if (distances_.size() >= 5) {
      float sum_dist = 0;
      for (float d : distances_) {
        sum_dist += d;
      }
      float final_dist = sum_dist / distances_.size() / 10.0; // Convert to cm
      distance_sensor->publish_state(round(final_dist * 10) / 10.0);
      measurements_sensor->publish_state(distances_.size());
      distances_.clear(); // Reset for next batch
    }
  }

 private:
  std::vector<float> distances_; // Store valid distance measurements
};