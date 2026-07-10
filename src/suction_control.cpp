//
// Created by zihao on 2026/6/11.
//

#include <cstddef>
#include <filesystem>
#include <sstream>
#include <sys/types.h>
#include <vector>

#include <algorithm>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include "suction_control.h"

namespace suction_control
{

    std::vector<std::string> R2SuctionControlNode::scan_serial_ports() const
    {
        std::vector<std::string> ports;
        constexpr const char* kDevPath = "/dev";

        try
        {
            for (const auto& entry : std::filesystem::directory_iterator(kDevPath))
            {
                if (!entry.is_character_file())
                {
                    continue;
                }

                const std::string name = entry.path().filename().string();
                if (name.rfind("ttyUSB", 0) == 0 || name.rfind("ttyACM", 0) == 0)
                {
                    ports.emplace_back(entry.path().string());
                }
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to scan serial ports: %s", e.what());
        }

        std::sort(ports.begin(), ports.end());
        return ports;
    }

    void R2SuctionControlNode::close_relay()
    {
        if (relay_fd_ >= 0)
        {
            close(relay_fd_);
            relay_fd_ = -1;
        }
    }

    void R2SuctionControlNode::refresh_serial_port_by_diff()
    {
        const auto current_ports = scan_serial_ports();
        std::vector<std::string> added_ports;
        std::vector<std::string> removed_ports;

        for (const auto& p : current_ports)
        {
            if (std::find(last_serial_ports_.begin(), last_serial_ports_.end(), p) == last_serial_ports_.end())
            {
                added_ports.push_back(p);
            }
        }
        for (const auto& p : last_serial_ports_)
        {
            if (std::find(current_ports.begin(), current_ports.end(), p) == current_ports.end())
            {
                removed_ports.push_back(p);
            }
        }

        if (!added_ports.empty() || !removed_ports.empty())
        {
            std::ostringstream oss;
            oss << "Serial diff detected.";
            if (!added_ports.empty())
            {
                oss << " added:";
                for (const auto& p : added_ports)
                {
                    oss << " " << p;
                }
            }
            if (!removed_ports.empty())
            {
                oss << " removed:";
                for (const auto& p : removed_ports)
                {
                    oss << " " << p;
                }
            }
            RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
        }

        std::string selected_port = serial_port_;
        const bool current_port_exists =
                std::find(current_ports.begin(), current_ports.end(), serial_port_) != current_ports.end();

        if (!current_port_exists && relay_fd_ >= 0)
        {
            RCLCPP_WARN(this->get_logger(), "Current serial port %s disappeared, closing stale fd.", serial_port_.c_str());
            close_relay();
        }

        if (!added_ports.empty())
        {
            selected_port = added_ports.front();
        }
        else if (!current_ports.empty() && !current_port_exists)
        {
            selected_port = current_ports.front();
        }

        if (selected_port != serial_port_)
        {
            RCLCPP_WARN(
                    this->get_logger(),
                    "Serial port switched from %s to %s due to port drift.",
                    serial_port_.c_str(), selected_port.c_str());
            serial_port_ = selected_port;
            close_relay();
        }

        last_serial_ports_ = current_ports;

        if (relay_fd_ < 0)
        {
            init_relay();
        }
    }

    R2SuctionControlNode::R2SuctionControlNode(const rclcpp::NodeOptions& options) :
            Node("suction_control", options),
            target_suck_(false),
            estop_active_(false),
            manual_ch_high_initialized_(false),
            manual_ch_high_(false),
            relay_fd_(-1) // 初始化串口为未打开状态 (-1)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing r2 suction control");

        rc_topic_ = this->declare_parameter<std::string>("rc_topic", "/sbus/read");
        serial_port_ = this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        channel_index_ = this->declare_parameter<int>("channel_index", 7); // 监听通道索引
        estop_channel_index_ = this->declare_parameter<int>("estop_channel_index", 4); // 急停监听通道索引（默认CH4）
        suck_threshold_ = this->declare_parameter<int>("suck_threshold", 1300); // channel value 触发阈值，大于这个值触发打开吸盘泵


        init_relay();
    last_serial_ports_ = scan_serial_ports();

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
            close_relay();
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
        refresh_serial_port_by_diff();

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
        if (channel_index_ < 0 || static_cast<std::size_t>(channel_index_) >= msg->channels.size())
        {
            return;
        }

        if (estop_channel_index_ >= 0 && static_cast<std::size_t>(estop_channel_index_) < msg->channels.size())
        {
            const uint16_t estop_ch = msg->channels[static_cast<std::size_t>(estop_channel_index_)];
            const bool estop_now = estop_ch >= static_cast<uint16_t>(suck_threshold_);

            if (estop_now)
            {
                if (!estop_active_)
                {
                    RCLCPP_WARN(this->get_logger(), "E-STOP ACTIVE (CH4 high): immediate shutdown.");
                }
                estop_active_ = true;

                if (target_suck_)
                {
                    target_suck_ = false;
                }

                // 急停优先级最高，高值时立即关闭并阻断后续控制
                set_valve(false);
                return;
            }

            if (estop_active_)
            {
                RCLCPP_INFO(this->get_logger(), "E-STOP RELEASED (CH4 low): control restored.");
            }
            estop_active_ = false;
        }

        const uint16_t ch = msg->channels[static_cast<std::size_t>(channel_index_)];
        const bool manual_high = ch >= static_cast<uint16_t>(suck_threshold_);

        if (!manual_ch_high_initialized_)
        {
            manual_ch_high_initialized_ = true;
            manual_ch_high_ = manual_high;
            return;
        }

        // 严格边沿触发：上升沿(低->高)关闭，下降沿(高->低)打开
        if (!manual_ch_high_ && manual_high)
        {
            if (target_suck_)
            {
                target_suck_ = false;
                RCLCPP_INFO(this->get_logger(), "MANUAL EDGE(RISING): -> [Suction Cup OFF]");
                set_valve(false);
            }
        }
        else if (manual_ch_high_ && !manual_high)
        {
            if (!target_suck_)
            {
                target_suck_ = true;
                RCLCPP_INFO(this->get_logger(), "MANUAL EDGE(FALLING): -> [Suction Cup ON]");
                set_valve(true);
            }
        }

        manual_ch_high_ = manual_high;
    }

    void R2SuctionControlNode::on_auto_suck_trigger(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (estop_active_)
        {
            RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "AUTO TRIGGER ignored: E-STOP is active.");
            return;
        }

        if (target_suck_ != msg->data)
        {
            RCLCPP_INFO(this->get_logger(), "AUTO TRIGGER: -> [Suction Cup %s]", msg->data ? "ON" : "OFF");
            target_suck_ = msg->data;
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