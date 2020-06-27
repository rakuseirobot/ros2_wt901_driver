#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "serial_message/msg/serial.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;

class Converter : public rclcpp::Node {
public:
    inline Converter();
private:
    inline void Convert(const serial_message::msg::Serial::SharedPtr data);
    rclcpp::Subscription<serial_message::msg::Serial>::SharedPtr input_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr output_;
};

Converter::Converter() : Node("imu_converter"){
    std::string input_topic, output_topic;

    this->declare_parameter<std::string>("Input_Topic", "RxSerial1");
    this->declare_parameter<std::string>("Output_Topic", "IMU1");

    this->get_parameter<std::string>("Input_Topic", input_topic);
    this->get_parameter<std::string>("Output_Topic", output_topic);

    input_ = this->create_subscription<serial_message::msg::Serial>(input_topic, 10, std::bind(&Converter::Convert, this, _1));
    output_ = this->create_publisher<sensor_msgs::msg::Imu>(output_topic, 10);
}

void Converter::Convert(const serial_message::msg::Serial::SharedPtr data){
    std::cout << std::hex << static_cast<int>(data->data) << std::endl;
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Converter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    exit(0);
}