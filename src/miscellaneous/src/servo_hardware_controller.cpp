#include "rclcpp/rclcpp.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <memory>
#include <chrono>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <cmath>
#include "std_msgs/msg/u_int8.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class ServoHardwareController : public rclcpp::Node {
    public:
        ServoHardwareController() : Node("servo_hardware_controller"){
            // Configuring Serial Communications
            serialDevice_servo = open("/dev/servos", O_RDWR | O_NOCTTY);

            if (serialDevice_servo < 0) {
                RCLCPP_ERROR(this->get_logger(),
                            "Failed to open /dev/servos: %s",
                            strerror(errno));
                throw std::runtime_error("Failed to open servo serial device");
            }
            struct termios tty_servo{}; 
            tcgetattr(serialDevice_servo, &tty_servo);
            cfsetispeed(&tty_servo, B115200);
            cfsetospeed(&tty_servo, B115200);

            tty_servo.c_cflag |= (CLOCAL | CREAD);
            tty_servo.c_cflag &= ~PARENB;
            tty_servo.c_cflag &= ~CSTOPB;
            tty_servo.c_cflag &= ~CSIZE;
            tty_servo.c_cflag |= CS8;

            tty_servo.c_lflag = 0;
            tty_servo.c_iflag = 0;
            tty_servo.c_oflag = 0;

            tty_servo.c_cc[VMIN] = 0;
            tty_servo.c_cc[VTIME] = 0;
            
            tcsetattr(serialDevice_servo, TCSANOW, &tty_servo);
            joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
            auto startup_timer_callback = [this]() -> void {
                RCLCPP_INFO(this->get_logger(), "Comm with Servo is ready");
                servo_arduino_cmd_ = this->create_subscription<std_msgs::msg::UInt8>("/servo_cmd", 10, std::bind(&ServoHardwareController::send_cmd, this, std::placeholders::_1));
                startup_timer_->cancel();
            };
            startup_timer_ = create_wall_timer(5000ms, startup_timer_callback);
            tf_timer_ = create_wall_timer(100ms, [this](){joint_state_pub_->publish(make_state(camera_angle));});

        }
        ~ServoHardwareController(){
            close(serialDevice_servo);
        }

    private:
        rclcpp::TimerBase::SharedPtr startup_timer_;
        rclcpp::TimerBase::SharedPtr tf_timer_;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr servo_arduino_cmd_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

        uint8_t cmd_buffer;
        int camera_angle = 0;
        int serialDevice_servo;

        void send_cmd(const std_msgs::msg::UInt8::SharedPtr msg){
            uint8_t cmd_buffer = static_cast<uint8_t>(msg->data);
            ssize_t bytes_written = write(serialDevice_servo, &cmd_buffer, 1);
                if (bytes_written < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to write to servo: %s", strerror(errno));
            }

            if (msg->data <= 180){
                camera_angle = msg->data;
            }
        }

        sensor_msgs::msg::JointState make_state(int angle){
            double angle_rad = static_cast<double>(angle) * M_PI / 180.0;
            sensor_msgs::msg::JointState state;

            state.header.stamp = this->now();
            state.header.frame_id = "base_link";
            state.name.push_back("camera_servo_joint");
            state.position.push_back(angle_rad);
            return state;
        }
};

int main(int argc, char ** argv) {
    rclcpp::init (argc, argv);

    auto node = std::make_shared<ServoHardwareController>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}