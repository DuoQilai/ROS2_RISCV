#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using namespace std::chrono_literals;

class ParamDemo : public rclcpp::Node
{
public:
  ParamDemo() : Node("param_demo")
  {
    declare_parameter("robot_name", "burger");
    declare_parameter("max_speed", 2.0);
    declare_parameter("sensor_list", std::vector<std::string>{"lidar", "camera"});
    declare_parameter("mode", "auto");
    declare_parameter("enable_debug", false);
    const auto initial = validate(get_parameters({"max_speed", "mode"}));
    if (!initial.successful) {throw std::invalid_argument(initial.reason);}
    callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {return validate(params);});
    timer_ = create_wall_timer(1s, [this]() {
      std::string sensors;
      const auto sensor_list = get_parameter("sensor_list").as_string_array();
      for (const auto & sensor : sensor_list) {
        if (!sensors.empty()) {sensors += ", ";}
        sensors += sensor;
      }
      RCLCPP_INFO(get_logger(),
        "robot_name=%s | max_speed=%.2f m/s | sensor_list=[%s] | mode=%s | enable_debug=%s",
        get_parameter("robot_name").as_string().c_str(), get_parameter("max_speed").as_double(),
        sensors.c_str(), get_parameter("mode").as_string().c_str(),
        get_parameter("enable_debug").as_bool() ? "true" : "false");
    });
  }

private:
  rcl_interfaces::msg::SetParametersResult validate(const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & param : params) {
      if (param.get_name() == "max_speed" &&
        (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
        !std::isfinite(param.as_double()) || param.as_double() < 0.0 || param.as_double() > 10.0))
      {
        result.successful = false;
        result.reason = "max_speed 必须是 [0.0, 10.0] 范围内的有限浮点数";
      }
      if (param.get_name() == "mode" &&
        (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
        (param.as_string() != "auto" && param.as_string() != "manual" && param.as_string() != "hybrid")))
      {
        result.successful = false;
        result.reason = "mode 必须是 auto、manual 或 hybrid";
      }
      if (!result.successful) {break;}
    }
    return result;
  }
  OnSetParametersCallbackHandle::SharedPtr callback_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  try {rclcpp::spin(std::make_shared<ParamDemo>());}
  catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("param_demo"), "%s", error.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
