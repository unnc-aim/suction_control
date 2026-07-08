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
            has_prev_rc_(false),
            prev_above_threshold_(false),
            relay_fd_(-1) // 初始化串口为未打开状态 (-1)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing r2 suction control");

        rc_topic_ = this->declare_parameter<std::string>("rc_topic", "/sbus/read");
        serial_port_ = this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        channel_index_ = this->declare_parameter<int>("channel_index", 7); // 监听通道索引
        suck_threshold_ = this->declare_parameter<int>("suck_threshold", 1300); // channel value 触发阈值，大于这个值触发打开吸盘泵


        init_relay();

        rc_sub_ = this->create_subscription<custom_msgs::msg::ReadSBUSRC>(
                rc_topic_, rclcpp::SensorDataQoS(),
                std::bind(&R2SuctionControlNode::on_rc_read, this, std::placeholders::_1));

        auto_suck_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                "/cmd_suction_suck", 10,
                std::bind(&R2SuctionControlNode::on_auto_suck_trigger, this, std::placeholders::_1));

        set_suction_srv_ = this->create_service<suction_control::srv::SetSuction>(
                "set_suction",
                std::bind(&R2SuctionControlNode::on_set_suction, this,
                          std::placeholders::_1, std::placeholders::_2));

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


    // 统一的吸盘状态设置入口：更新目标状态并驱动硬件，返回是否成功写入
    bool R2SuctionControlNode::apply_suck(bool suck)
    {
        if (relay_fd_ < 0)
        {
            RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "Serial port is not open. Cannot control valves!");
            return false;
        }

        target_suck_ = suck;
        set_valve(suck);
        return true;
    }


    void R2SuctionControlNode::on_rc_read(const custom_msgs::msg::ReadSBUSRC::SharedPtr msg)
    {
        if (channel_index_ < 0 || static_cast<std::size_t>(channel_index_) >= msg->channels.size())
        {
            return;
        }

        const uint16_t ch = msg->channels[static_cast<std::size_t>(channel_index_)];

        RCLCPP_INFO(this->get_logger(), "CH raw value: %d", ch);

        // 手动跳变切换逻辑：每次从阈值的一边跳变到另一边时，翻转一次吸盘状态
        // 为了防止和topic冲突，这里维护一下channel的状态，这样可以同时兼容service和 sbus channel 两者同时存在
        const bool above = (ch >= static_cast<uint16_t>(suck_threshold_));

        // 首帧仅记录初始侧，不触发切换
        if (!has_prev_rc_)
        {
            has_prev_rc_ = true;
            prev_above_threshold_ = above;
            return;
        }

        // 仅当跨越阈值（侧别发生变化）时才翻转
        if (above != prev_above_threshold_)
        {
            prev_above_threshold_ = above;
            const bool new_suck = !target_suck_;
            RCLCPP_INFO(this->get_logger(), "MANUAL TOGGLE (crossed threshold -> %s): -> [Suction Cup %s]",
                        above ? "above" : "below", new_suck ? "ON" : "OFF");
            apply_suck(new_suck);
        }
    }

    void R2SuctionControlNode::on_auto_suck_trigger(const std_msgs::msg::Bool::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "AUTO TRIGGER: -> [Suction Cup %s]", msg->data ? "ON" : "OFF");
        apply_suck(msg->data);
    }

    void R2SuctionControlNode::on_set_suction(
            const suction_control::srv::SetSuction::Request::SharedPtr req,
            suction_control::srv::SetSuction::Response::SharedPtr res)
    {
        const bool ok = apply_suck(req->suck);
        res->success = ok;
        res->current_suck = target_suck_;
        res->message = ok
                       ? (req->suck ? std::string("Suction cup turned ON") : std::string("Suction cup turned OFF"))
                       : std::string("Failed: serial port is not open");

        RCLCPP_INFO(this->get_logger(), "SERVICE TRIGGER (set_suction): suck=%d -> success=%d, current=%d",
                    static_cast<int>(req->suck), static_cast<int>(ok), static_cast<int>(target_suck_));
    }
} // namespace suction_control

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<suction_control::R2SuctionControlNode>());
    rclcpp::shutdown();
    return 0;
}