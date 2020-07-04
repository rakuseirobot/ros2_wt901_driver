#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "serial_message/msg/serial.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

using std::placeholders::_1;

class Converter : public rclcpp::Node {
public:
    inline Converter();
private:
    inline void Convert(const serial_message::msg::Serial::SharedPtr data);
    template<typename T>
    inline std::vector<T> EularToQuaternion(T roll, T pitch, T yaw);
    rclcpp::Subscription<serial_message::msg::Serial>::SharedPtr input_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_output_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magnetic_output_;
    rclcpp::Clock clock;
};

Converter::Converter() : Node("imu_converter"){
    std::string input_topic, imu_out, magnetic_out;

    this->declare_parameter<std::string>("Input_Topic", "RxSerial1");
    this->declare_parameter<std::string>("IMU_Out", "IMU1");
    this->declare_parameter<std::string>("Magnetic_Out", "Magnetic1");

    this->get_parameter<std::string>("Input_Topic", input_topic);
    this->get_parameter<std::string>("IMU_Out", imu_out);
    this->get_parameter<std::string>("Magnetic_Out", magnetic_out);

    input_ = this->create_subscription<serial_message::msg::Serial>(input_topic, 10, std::bind(&Converter::Convert, this, _1));
    imu_output_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_out, 10);
    magnetic_output_ = this->create_publisher<sensor_msgs::msg::MagneticField>(magnetic_out, 10);
    
    clock = rclcpp::Clock(RCL_ROS_TIME);
}

template<typename T>
std::vector<T> Converter::EularToQuaternion(T roll, T pitch, T yaw){
    std::vector<T> q;
    double x, y, z, w;
    double a = roll / 2.0;
    double b = pitch / 2.0;
    double g = yaw / 2.0;

    w = cos(a) * cos(b) * cos(g) + sin(a) * sin(b) * sin(g);
    x = sin(a) * cos(b) * cos(g) - cos(a) * sin(b) * sin(g);
    y = cos(a) * sin(b) * cos(g) + sin(a) * cos(b) * sin(g);
    z = cos(a) * cos(b) * sin(g) - sin(a) * sin(b) * cos(g);

    q.push_back(w);
    q.push_back(x);
    q.push_back(y);
    q.push_back(z);
    return q;
}

void Converter::Convert(const serial_message::msg::Serial::SharedPtr data){
    static std::vector<uint8_t> buffer = std::vector<uint8_t>(44);
    auto imu = sensor_msgs::msg::Imu();
    auto magnetic_field = sensor_msgs::msg::MagneticField();
    std::vector<float> quaternion;
    float roll, pitch, yaw;

    //std::cout << std::hex << static_cast<int>(data->data) << std::endl;
    if(buffer[0] != 0x55) buffer.clear();
    if(buffer.size() >= 2  && buffer[1] != 0x51) buffer.clear();
    buffer.push_back(data->data);
    if(buffer.size() == 44){
        RCLCPP_DEBUG(this->get_logger(), "get_imu_data");

        imu.header.frame_id = "jy901";
        imu.header.stamp = clock.now();
        //linear_acceleration
        imu.linear_acceleration.x = (float)(((short)buffer[3] << 8) | buffer[2]) / 32768 * 16 * 9.8;
        imu.linear_acceleration.y = (float)(((short)buffer[5] << 8) | buffer[4]) / 32768 * 16 * 9.8;
        imu.linear_acceleration.z = (float)(((short)buffer[7] << 8) | buffer[6]) / 32768 * 16 * 9.8;
        imu.linear_acceleration_covariance[0] = -1;
        //angular_verocity
        imu.angular_velocity.x = (float)(((short)buffer[14] << 8) | buffer[13]) / 32768 * 2000 * M_PI / 180;
        imu.angular_velocity.y = (float)(((short)buffer[16] << 8) | buffer[15]) / 32768 * 2000 * M_PI / 180;
        imu.angular_velocity.z = (float)(((short)buffer[18] << 8) | buffer[17]) / 32768 * 2000 * M_PI / 180;
        imu.angular_velocity_covariance[0] = -1;
        //quaternion
        roll = (float)(((short)buffer[25] << 8) | buffer[24]) / 32768 * M_PI;
        pitch = (float)(((short)buffer[27] << 8) | buffer[26]) / 32768 * M_PI;
        yaw = (float)(((short)buffer[29] << 8) | buffer[28]) / 32768 * M_PI;
        quaternion = EularToQuaternion(roll, pitch, yaw);
        imu.orientation.w = quaternion[0];
        imu.orientation.x = quaternion[1];
        imu.orientation.y = quaternion[2];
        imu.orientation.z = quaternion[3];
        imu.orientation_covariance[0] = -1;
        //std::cout << std::dec << "roll: " << roll << " pitch: " << pitch << " yaw: " << yaw << std::endl;
        imu_output_ -> publish(imu);

        buffer.clear();
    } 
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Converter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    exit(0);
}