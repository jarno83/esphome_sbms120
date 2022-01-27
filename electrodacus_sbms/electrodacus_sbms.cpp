#include "sbmsData.hpp"
#include "electrodacus_sbms.h"
#include "esphome/core/log.h"
#include <vector>

namespace esphome {
namespace electrodacus_sbms {

static const char *const TAG = "electrodacus_sbms";

static const uint8_t SBMS_FRAME_SIZE = 13;
static const uint8_t SBMS_TEMPERATURE_OFFSET = 40;
static const uint16_t SBMS_CURRENT_OFFSET = 30000;

static const uint8_t SBMS_REQUEST_BATTERY_LEVEL = 0x90;
static const uint8_t SBMS_REQUEST_MIN_MAX_VOLTAGE = 0x91;
static const uint8_t SBMS_REQUEST_MIN_MAX_TEMPERATURE = 0x92;
static const uint8_t SBMS_REQUEST_MOS = 0x93;
static const uint8_t SBMS_REQUEST_STATUS = 0x94;
static const uint8_t SBMS_REQUEST_TEMPERATURE = 0x96;

void ElectrodacusSbmsComponent::setup() {}

void ElectrodacusSbmsComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Electrodacus SBMS:");
  this->check_uart_settings(9600);
}

void ElectrodacusSbmsComponent::update() {
  this->request_data_(SBMS_REQUEST_BATTERY_LEVEL);
  this->request_data_(SBMS_REQUEST_MIN_MAX_VOLTAGE);
  this->request_data_(SBMS_REQUEST_MIN_MAX_TEMPERATURE);
  this->request_data_(SBMS_REQUEST_MOS);
  this->request_data_(SBMS_REQUEST_STATUS);
  this->request_data_(SBMS_REQUEST_TEMPERATURE);

  std::vector<uint8_t> get_battery_level_data;
  int available_data = this->available();
  if (available_data >= SBMS_FRAME_SIZE) {
    get_battery_level_data.resize(available_data);
    this->read_array(get_battery_level_data.data(), available_data);
    this->decode_data_(get_battery_level_data);
  }
}

float ElectrodacusSbmsComponent::get_setup_priority() const { return setup_priority::DATA; }

void ElectrodacusSbmsComponent::request_data_(uint8_t data_id) {
  uint8_t request_message[SBMS_FRAME_SIZE];

  request_message[0] = 0xA5;     // Start Flag
  request_message[1] = 0x80;     // Communication Module Address
  request_message[2] = data_id;  // Data ID
  request_message[3] = 0x08;     // Data Length (Fixed)
  request_message[4] = 0x00;     // Empty Data
  request_message[5] = 0x00;     //     |
  request_message[6] = 0x00;     //     |
  request_message[7] = 0x00;     //     |
  request_message[8] = 0x00;     //     |
  request_message[9] = 0x00;     //     |
  request_message[10] = 0x00;    //     |
  request_message[11] = 0x00;    // Empty Data
  request_message[12] = (uint8_t)(request_message[0] + request_message[1] + request_message[2] +
                                  request_message[3]);  // Checksum (Lower byte of the other bytes sum)

  this->write_array(request_message, sizeof(request_message));
  this->flush();
}

void ElectrodacusSbmsComponent::decode_data_(std::vector<uint8_t> data) {
  auto it = data.begin();

  while ((it = std::find(it, data.end(), 0xA5)) != data.end()) {
    if (data.end() - it >= SBMS_FRAME_SIZE && it[1] == 0x01) {
      uint8_t checksum;
      int sum = 0;
      for (int i = 0; i < 12; i++) {
        sum += it[i];
      }
      checksum = sum;

      if (checksum == it[12]) {
        switch (it[2]) {
          case SBMS_REQUEST_BATTERY_LEVEL:
            if (this->voltage_sensor_) {
              this->voltage_sensor_->publish_state((float) encode_uint16(it[4], it[5]) / 10);
            }
            if (this->current_sensor_) {
              this->current_sensor_->publish_state(((float) (encode_uint16(it[8], it[9]) - SBMS_CURRENT_OFFSET) / 10));
            }
            if (this->battery_level_sensor_) {
              this->battery_level_sensor_->publish_state((float) encode_uint16(it[10], it[11]) / 10);
            }
            break;

          case SBMS_REQUEST_MIN_MAX_VOLTAGE:
            if (this->max_cell_voltage_) {
              this->max_cell_voltage_->publish_state((float) encode_uint16(it[4], it[5]) / 1000);
            }
            if (this->max_cell_voltage_number_) {
              this->max_cell_voltage_number_->publish_state(it[6]);
            }
            if (this->min_cell_voltage_) {
              this->min_cell_voltage_->publish_state((float) encode_uint16(it[7], it[8]) / 1000);
            }
            if (this->min_cell_voltage_number_) {
              this->min_cell_voltage_number_->publish_state(it[9]);
            }
            break;

          case SBMS_REQUEST_MIN_MAX_TEMPERATURE:
            if (this->max_temperature_) {
              this->max_temperature_->publish_state(it[4] - SBMS_TEMPERATURE_OFFSET);
            }
            if (this->max_temperature_probe_number_) {
              this->max_temperature_probe_number_->publish_state(it[5]);
            }
            if (this->min_temperature_) {
              this->min_temperature_->publish_state(it[6] - SBMS_TEMPERATURE_OFFSET);
            }
            if (this->min_temperature_probe_number_) {
              this->min_temperature_probe_number_->publish_state(it[7]);
            }
            break;

          case SBMS_REQUEST_MOS:
            if (this->status_text_sensor_ != nullptr) {
              switch (it[4]) {
                case 0:
                  this->status_text_sensor_->publish_state("Stationary");
                  break;
                case 1:
                  this->status_text_sensor_->publish_state("Charging");
                  break;
                case 2:
                  this->status_text_sensor_->publish_state("Discharging");
                  break;
                default:
                  break;
              }
            }
            if (this->charging_mos_enabled_) {
              this->charging_mos_enabled_->publish_state(it[5]);
            }
            if (this->discharging_mos_enabled_) {
              this->discharging_mos_enabled_->publish_state(it[6]);
            }
            if (this->remaining_capacity_) {
              this->remaining_capacity_->publish_state((float) encode_uint32(it[8], it[9], it[10], it[11]) / 1000);
            }
            break;

          case SBMS_REQUEST_STATUS:
            if (this->cells_number_) {
              this->cells_number_->publish_state(it[4]);
            }
            break;

          case SBMS_REQUEST_TEMPERATURE:
            if (it[4] == 1) {
              if (this->temperature_1_sensor_) {
                this->temperature_1_sensor_->publish_state(it[5] - SBMS_TEMPERATURE_OFFSET);
              }
              if (this->temperature_2_sensor_) {
                this->temperature_2_sensor_->publish_state(it[6] - SBMS_TEMPERATURE_OFFSET);
              }
            }
            break;

          default:
            break;
        }
      }
      std::advance(it, SBMS_FRAME_SIZE);
    } else {
      std::advance(it, 1);
    }
  }
}

}  // namespace electrodacus_sbms
}  // namespace esphome
