#include "sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace sensor {

static const char *const TAG = "sensor";

std::string state_class_to_string(StateClass state_class) {
  switch (state_class) {
    case STATE_CLASS_MEASUREMENT:
      return "measurement";
    case STATE_CLASS_TOTAL_INCREASING:
      return "total_increasing";
    case STATE_CLASS_TOTAL:
      return "total";
    case STATE_CLASS_NONE:
    default:
      return "";
  }
}

Sensor::Sensor() : state(NAN), raw_state(NAN) {}

int8_t Sensor::get_accuracy_decimals() {
  if (this->accuracy_decimals_.has_value())
    return *this->accuracy_decimals_;
  return 0;
}
void Sensor::set_accuracy_decimals(int8_t accuracy_decimals) { this->accuracy_decimals_ = accuracy_decimals; }

void Sensor::set_state_class(StateClass state_class) { this->state_class_ = state_class; }
StateClass Sensor::get_state_class() {
  if (this->state_class_.has_value())
    return *this->state_class_;
  return StateClass::STATE_CLASS_NONE;
}

void Sensor::publish_state(float state) {
  this->raw_state = state;
  this->raw_callback_.call(state);

  ESP_LOGV(TAG, "'%s': Received new state %f", this->name_.c_str(), state);

  if (this->filter_list_ == nullptr) {
    this->internal_send_state_to_frontend(state);
  } else {
    this->filter_list_->input(state);
  }
}

void Sensor::add_on_state_callback(std::function<void(float)> &&callback) { this->callback_.add(std::move(callback)); }
void Sensor::add_on_raw_state_callback(std::function<void(float)> &&callback) {
  this->raw_callback_.add(std::move(callback));
}

void Sensor::add_filter(Filter *filter) {
  // inefficient, but only happens once on every sensor setup and nobody's going to have massive amounts of
  // filters
  ESP_LOGVV(TAG, "Sensor(%p)::add_filter(%p)", this, filter);
  if (this->filter_list_ == nullptr) {
    this->filter_list_ = filter;
  } else {
    Filter *last_filter = this->filter_list_;
    while (last_filter->next_ != nullptr)
      last_filter = last_filter->next_;
    last_filter->initialize(this, filter);
  }
  filter->initialize(this, nullptr);
}
void Sensor::add_filters(const std::vector<Filter *> &filters) {
  for (Filter *filter : filters) {
    this->add_filter(filter);
  }
}
void Sensor::set_filters(const std::vector<Filter *> &filters) {
  this->clear_filters();
  this->add_filters(filters);
}
void Sensor::clear_filters() {
  if (this->filter_list_ != nullptr) {
    ESP_LOGVV(TAG, "Sensor(%p)::clear_filters()", this);
  }
  this->filter_list_ = nullptr;
}
float Sensor::get_state() const { return this->state; }
float Sensor::get_raw_state() const { return this->raw_state; }
std::string Sensor::unique_id() {
  // 如果子类没有覆盖，sensor_id 可能为空
  static int default_sensor_counter = 0;

  // 此处假设有一个 sensor_id，或者可以用 this->get_name() 来做
  std::string id_suffix = this->sensor_id;  // 如果有的话
  if (id_suffix.empty()) {
    default_sensor_counter++;
    // 若 get_name() 也为空，则使用 "default"
    std::string base_name = this->get_name();
    if (base_name.empty()) {
      base_name = "default";
    }
    // 拼接计数器
    id_suffix = base_name + "-" + std::to_string(default_sensor_counter);
  }

  // 如果能拿到 device_mac，则建议也加到前缀中
  // 假设这里没有，就用设备名称或 "unknown_device"
  std::string device_prefix = this->get_name();
  if (device_prefix.empty()) {
    device_prefix = "unknown_device";
  }
  return device_prefix + "-sensor-" + id_suffix;
}

void Sensor::internal_send_state_to_frontend(float state) {
  this->has_state_ = true;
  this->state = state;
  ESP_LOGD(TAG, "'%s': Sending state %.5f %s with %d decimals of accuracy", this->get_name().c_str(), state,
           this->get_unit_of_measurement().c_str(), this->get_accuracy_decimals());
  this->callback_.call(state);
}
bool Sensor::has_state() const { return this->has_state_; }

}  // namespace sensor
}  // namespace esphome
