#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"

class MyFirstNode : public rclcpp::Node
{
public:
  MyFirstNode() : Node("my_first_node")
  {
    // Vytvoření publisheru na téma "node_name"
    publisher_ = this->create_publisher<std_msgs::msg::String>("node_name", 10);

    // Vytvoření publisheru na téma "battery_percentage" s Float32
    battery_percentage_publisher_ = this->create_publisher<std_msgs::msg::Float32>("battery_percentage", 10);

    // Vytvoření subscriberu na téma "battery_voltage" s Float32
    battery_voltage_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
      "battery_voltage", 10,
      std::bind(&MyFirstNode::battery_voltage_callback, this, std::placeholders::_1));

    // Parametry pro maximální a minimální napětí (defaultní hodnoty)
    this->declare_parameter<float>("max_voltage", 42.0f);  // Max voltage defaultně 42V
    this->declare_parameter<float>("min_voltage", 32.0f);  // Min voltage defaultně 36V

    // Vytvoření timeru pro publikování zpráv na "node_name"
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&MyFirstNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "my_first_node";  // Název Node
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  }

  void battery_voltage_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
  	auto max_voltage_ = this->get_parameter("max_voltage").as_double();
    auto min_voltage_ = this->get_parameter("min_voltage").as_double();
    float voltage = msg->data;  // Napětí přijaté na tématu "battery_voltage"
    RCLCPP_INFO(this->get_logger(), "Received battery voltage: %.2f V", voltage);  // Logování přijaté hodnoty napětí

    // Výpočet procenta nabití na základě parametrů
    float percentage = (voltage - min_voltage_) / (max_voltage_ - min_voltage_) * 100.0f;

    // Ošetření případů, kdy je napětí mimo stanovený rozsah
    if (percentage < 0.0f) {
      percentage = 0.0f;  // Pokud je napětí nižší než min_voltage_, nastavíme 0%
    } else if (percentage > 100.0f) {
      percentage = 100.0f;  // Pokud je napětí vyšší než max_voltage_, nastavíme 100%
    }

    // Publikování přepočítaného procenta na téma "battery_percentage" s Float32
    auto percentage_msg = std_msgs::msg::Float32();
    percentage_msg.data = percentage;  // Publikování procenta jako float
    battery_percentage_publisher_->publish(percentage_msg);

    RCLCPP_INFO(this->get_logger(), "Battery percentage: %.2f%%", percentage);  // Logování publikované hodnoty
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_percentage_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_voltage_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;

  float max_voltage_;  // Maximální napětí pro výpočet
  float min_voltage_;  // Minimální napětí pro výpočet
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // Spuštění Node s parametry
  rclcpp::spin(std::make_shared<MyFirstNode>());
  rclcpp::shutdown();
  return 0;
}

