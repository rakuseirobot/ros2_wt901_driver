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
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr output_;
};

Converter::Converter() : Node("imu_converter"){
    input_ = this->create_subscription<serial_message::msg::Serial>("RxSerial1", 10, std::bind(&Converter::Convert, this, _1));
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