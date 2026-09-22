#include <algorithm>
#include <cctype>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "weather_interfaces/srv/weather_query.hpp"

class WeatherServer : public rclcpp::Node
{
public:
  WeatherServer()
  : Node("weather_server")
  {
    service_ = create_service<weather_interfaces::srv::WeatherQuery>(
      "weather_query",
      std::bind(&WeatherServer::handle, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "WEATHER_SERVER_READY");
  }

private:
  void handle(
    const std::shared_ptr<weather_interfaces::srv::WeatherQuery::Request> request,
    std::shared_ptr<weather_interfaces::srv::WeatherQuery::Response> response)
  {
    std::string city = request->city;
    city.erase(city.begin(), std::find_if(city.begin(), city.end(), [](unsigned char value) {
      return !std::isspace(value);
    }));
    city.erase(std::find_if(city.rbegin(), city.rend(), [](unsigned char value) {
      return !std::isspace(value);
    }).base(), city.end());
    std::transform(city.begin(), city.end(), city.begin(), [](unsigned char value) {
      return static_cast<char>(std::tolower(value));
    });
    if (city == "beijing") {
      response->temperature = 26.5;
      response->weather = "Sunny";
    } else if (city == "shanghai") {
      response->temperature = 24.0;
      response->weather = "Cloudy";
    } else if (city == "shenzhen") {
      response->temperature = 30.0;
      response->weather = "Rainy";
    } else {
      response->temperature = 0.0;
      response->weather = "Unknown city";
    }
    RCLCPP_INFO(get_logger(), "WEATHER_RESULT city=%s weather=%s", city.c_str(), response->weather.c_str());
  }

  rclcpp::Service<weather_interfaces::srv::WeatherQuery>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WeatherServer>());
  rclcpp::shutdown();
  return 0;
}
