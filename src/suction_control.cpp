//
// Created by zihao on 2026/6/11.
//

#include <cstddef>
#include <sys/types.h>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include "suction_control.h"

namespace suction_control
{

    R2SuctionControlNode::R2SuctionControlNode(const rclcpp::NodeOptions& options) :
            Node("suction_control", options),
            target_suck_(false),
            relay_fd_(-1) // 初始化串口为未打开状态 (-1)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing r2 suction control");

        rc_topic_ = this->declare_parameter<std::string>("rc_topic", "/sbus/read");
        serial_port_ = this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        channel7_index_ = this->declare_parameter<int>("channel7_index", 7); // 通道7 (索引为6)
        suck_threshold_ = this->declare_parameter<int>("suck_threshold", 1300);


        init_relay();

        rc_sub_ = this->create_subscription<custom_msgs::msg::ReadSBUSRC>(
                rc_topic_, rclcpp::SensorDataQoS(),
                std::bind(&R2SuctionControlNode::on_rc_read, this, std::placeholders::_1));

        auto_suck_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                "/cmd_suction_suck", 10,
                std::bind(&R2SuctionControlNode::on_auto_suck_trigger, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "====== [Phase 3: Hardware Integration] Node Started Successfully! ======");
    }


    R2SuctionControlNode::~R2SuctionControlNode()
    {
        if (relay_fd_ >= 0)
        {
            set_valve(false);
            close(relay_fd_);
            RCLCPP_INFO(this->get_logger(), "Successfully closed serial port: %s", serial_port_.c_str());
        }
    }

    // 初始化
    void R2SuctionControlNode::init_relay()
    {
        relay_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (relay_fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s!", serial_port_.c_str());
            return;
        }

        // 配置波特率为 9600，无校验，8位数据位，1位停止位
        struct termios options;
        tcgetattr(relay_fd_, &options);
        cfsetispeed(&options, B9600);
        cfsetospeed(&options, B9600);
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        tcsetattr(relay_fd_, TCSANOW, &options);

        RCLCPP_INFO(this->get_logger(), "Serial port %s opened successfully.", serial_port_.c_str());
    }


    void R2SuctionControlNode::set_valve(bool suck)
    {
        if (relay_fd_ < 0)
        {
            RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "Serial port is not open. Cannot control valves!");
            return;
        }

        if (suck)
        {
            // 第一路打开指令：A0 01 01 A2
            uint8_t open_ch1[] = {0xA0, 0x01, 0x01, 0xA2};
            write(relay_fd_, open_ch1, sizeof(open_ch1));

            // 第二路瞬间打开指令：A0 02 01 A3
            uint8_t open_ch2[] = {0xA0, 0x02, 0x01, 0xA3};
            write(relay_fd_, open_ch2, sizeof(open_ch2));

            RCLCPP_DEBUG(this->get_logger(), "Hardware: Both channels triggered ON.");
        }
        else
        {
            // 第一路关闭指令：A0 01 00 A1
            uint8_t close_ch1[] = {0xA0, 0x01, 0x00, 0xA1};
            write(relay_fd_, close_ch1, sizeof(close_ch1));

            // 第二路瞬间关闭指令：A0 02 00 A2
            uint8_t close_ch2[] = {0xA0, 0x02, 0x00, 0xA2};
            write(relay_fd_, close_ch2, sizeof(close_ch2));

            RCLCPP_DEBUG(this->get_logger(), "Hardware: Both channels triggered OFF.");
        }
    }


    void R2SuctionControlNode::on_rc_read(const custom_msgs::msg::ReadSBUSRC::SharedPtr msg)
    {
        if (channel7_index_ < 0 || static_cast<std::size_t>(channel7_index_) >= msg->channels.size())
        {
            return;
        }

        const uint16_t ch7 = msg->channels[static_cast<std::size_t>(channel7_index_)];

        RCLCPP_INFO(this->get_logger(), "CH7 raw value: %d", ch7);

        // 手动边缘触发逻辑
        if (ch7 >= static_cast<uint16_t>(suck_threshold_))
        {
            if (!target_suck_)
            {
                target_suck_ = true;
                RCLCPP_INFO(this->get_logger(), "MANUAL TRIGGER: -> [Suction Cup ON]");


                set_valve(true);
            }
        }
        else
        {
            if (target_suck_)
            {
                target_suck_ = false;
                RCLCPP_INFO(this->get_logger(), "MANUAL TRIGGER: -> [Suction Cup OFF]");


                set_valve(false);
            }
        }
    }

    void R2SuctionControlNode::on_auto_suck_trigger(const std_msgs::msg::Bool::SharedPtr msg)
    {

        if (!target_suck_)
        {
            RCLCPP_INFO(this->get_logger(), "AUTO TRIGGER: -> [Suction Cup %s]", msg->data ? "ON" : "OFF");


            set_valve(msg->data);
        }
    }
} // namespace suction_control

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<suction_control::R2SuctionControlNode>());
    rclcpp::shutdown();
    return 0;
}