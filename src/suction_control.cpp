//
// Created by zihao on 2026/6/11.
//

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <sys/types.h>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include "suction_control.h"

namespace suction_control
{

    namespace fs = std::filesystem;

    namespace
    {

        bool has_prefix(const std::string & value, const std::string & prefix)
        {
            return value.rfind(prefix, 0U) == 0U;
        }

    }

    R2SuctionControlNode::R2SuctionControlNode(const rclcpp::NodeOptions& options) :
            Node("suction_control", options),
            target_suck_(false),
            has_prev_rc_(false),
            prev_above_threshold_(false),
            relay_fd_(-1) // 初始化串口为未打开状态 (-1)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing r2 suction control");

        rc_topic_ = this->declare_parameter<std::string>("rc_topic", "/sbus/read");
        serial_port_ = this->declare_parameter<std::string>("serial_port", "auto");
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
            "/set_suction",
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
        // 启动时尽力打开一次；失败不致命——apply_suck 会在收到指令时懒加载重试，
        // 以应对 USB 设备在节点启动之后才枚举出来的时序竞争。
        try_open_relay();
    }

    std::vector<std::string> R2SuctionControlNode::discover_serial_ports() const
    {
        std::vector<std::string> candidates;
        auto add_candidate = [&candidates](const std::string & candidate) {
            if (candidate.empty()) {
                return;
            }
            if (std::find(candidates.begin(), candidates.end(), candidate) == candidates.end()) {
                candidates.push_back(candidate);
            }
        };

        if (!serial_port_.empty() && (serial_port_ != "auto")) {
            add_candidate(serial_port_);
        }

        std::error_code ec;
        const fs::path by_id_dir("/dev/serial/by-id");
        if (fs::exists(by_id_dir, ec) && fs::is_directory(by_id_dir, ec)) {
            std::vector<std::string> by_id_candidates;
            for (const auto & entry : fs::directory_iterator(by_id_dir, ec)) {
                if (!entry.is_symlink()) {
                    continue;
                }

                by_id_candidates.push_back(entry.path().string());
            }
            std::sort(by_id_candidates.begin(), by_id_candidates.end());
            for (const auto & candidate : by_id_candidates) {
                add_candidate(candidate);
            }
        }

        const fs::path dev_dir("/dev");
        if (fs::exists(dev_dir, ec) && fs::is_directory(dev_dir, ec)) {
            std::vector<std::string> tty_candidates;
            for (const auto & entry : fs::directory_iterator(dev_dir, ec)) {
                if (!entry.is_character_file() && !entry.is_symlink()) {
                    continue;
                }

                const std::string name = entry.path().filename().string();
                if (has_prefix(name, "ttyUSB") || has_prefix(name, "ttyACM") ||
                    has_prefix(name, "ttyS") || has_prefix(name, "ttyAMA") ||
                    has_prefix(name, "rfcomm")) {
                    tty_candidates.push_back(entry.path().string());
                }
            }
            std::sort(tty_candidates.begin(), tty_candidates.end());
            for (const auto & candidate : tty_candidates) {
                add_candidate(candidate);
            }
        }

        return candidates;
    }

    bool R2SuctionControlNode::open_serial_port(const std::string & serial_port)
    {
        const int fd = open(serial_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd < 0) {
            return false;
        }

        struct termios options;
        if (tcgetattr(fd, &options) != 0) {
            close(fd);
            return false;
        }

        cfsetispeed(&options, B9600);
        cfsetospeed(&options, B9600);
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        if (tcsetattr(fd, TCSANOW, &options) != 0) {
            close(fd);
            return false;
        }

        relay_fd_ = fd;
        serial_port_ = serial_port;
        RCLCPP_INFO(this->get_logger(), "Serial port %s opened successfully.", serial_port_.c_str());
        return true;
    }

    bool R2SuctionControlNode::try_open_relay()
    {
        if (relay_fd_ >= 0)
        {
            return true; // 已打开
        }

        const auto candidates = discover_serial_ports();
        if (candidates.empty()) {
            RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "No serial candidates found while searching for suction control.");
            return false;
        }

        for (const auto & candidate : candidates) {
            RCLCPP_INFO(this->get_logger(), "Trying serial candidate: %s", candidate.c_str());
            if (open_serial_port(candidate)) {
                return true;
            }
        }

        RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Failed to open any serial candidate for suction control. Will retry on next request.");
        return false;
    }


    bool R2SuctionControlNode::set_valve(bool suck)
    {
        if (relay_fd_ < 0)
        {
            RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "Serial port is not open. Cannot control valves!");
            return false;
        }

        if (suck)
        {
            // 第一路打开指令：A0 01 01 A2
            uint8_t open_ch1[] = {0xA0, 0x01, 0x01, 0xA2};
            const ssize_t written_ch1 = write(relay_fd_, open_ch1, sizeof(open_ch1));

            // 第二路瞬间打开指令：A0 02 01 A3
            uint8_t open_ch2[] = {0xA0, 0x02, 0x01, 0xA3};
            const ssize_t written_ch2 = write(relay_fd_, open_ch2, sizeof(open_ch2));

            if ((written_ch1 != static_cast<ssize_t>(sizeof(open_ch1))) ||
                (written_ch2 != static_cast<ssize_t>(sizeof(open_ch2)))) {
                RCLCPP_WARN(
                        this->get_logger(),
                        "Failed to write suction ON command to %s, reopening on next request.",
                        serial_port_.c_str());
                close(relay_fd_);
                relay_fd_ = -1;
                return false;
            }

            RCLCPP_DEBUG(this->get_logger(), "Hardware: Both channels triggered ON.");
            return true;
        }
        else
        {
            // 第一路关闭指令：A0 01 00 A1
            uint8_t close_ch1[] = {0xA0, 0x01, 0x00, 0xA1};
            const ssize_t written_ch1 = write(relay_fd_, close_ch1, sizeof(close_ch1));

            // 第二路瞬间关闭指令：A0 02 00 A2
            uint8_t close_ch2[] = {0xA0, 0x02, 0x00, 0xA2};
            const ssize_t written_ch2 = write(relay_fd_, close_ch2, sizeof(close_ch2));

            if ((written_ch1 != static_cast<ssize_t>(sizeof(close_ch1))) ||
                (written_ch2 != static_cast<ssize_t>(sizeof(close_ch2)))) {
                RCLCPP_WARN(
                        this->get_logger(),
                        "Failed to write suction OFF command to %s, reopening on next request.",
                        serial_port_.c_str());
                close(relay_fd_);
                relay_fd_ = -1;
                return false;
            }

            RCLCPP_DEBUG(this->get_logger(), "Hardware: Both channels triggered OFF.");
            return true;
        }
    }


    // 统一的吸盘状态设置入口：更新目标状态并驱动硬件，返回是否成功写入
    bool R2SuctionControlNode::apply_suck(bool suck)
    {
        if (relay_fd_ < 0)
        {
            // 懒加载重试：串口未打开（如启动时设备尚未枚举）则在收到指令时再尝试打开
            if (!try_open_relay())
            {
                RCLCPP_WARN(this->get_logger(),
                            "apply_suck(%s) skipped: relay is unavailable.",
                            suck ? "ON" : "OFF");
                return false;
            }
        }

        const bool ok = set_valve(suck);
        if (ok)
        {
            target_suck_ = suck;
            return true;
        }

        RCLCPP_WARN(this->get_logger(),
                    "apply_suck(%s) failed: keep previous target state [%s].",
                    suck ? "ON" : "OFF", target_suck_ ? "ON" : "OFF");
        return false;
    }


    void R2SuctionControlNode::on_rc_read(const custom_msgs::msg::ReadSBUSRC::SharedPtr msg)
    {
        if (channel_index_ < 0 || static_cast<std::size_t>(channel_index_) >= msg->channels.size())
        {
            return;
        }

        const uint16_t ch = msg->channels[static_cast<std::size_t>(channel_index_)];

        // RCLCPP_INFO(this->get_logger(), "CH %d raw value: %d", channel_index_, ch);

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

        // 仅当跨越阈值（侧别发生变化）时才应用。
        // 语义保持与参数说明一致：高于阈值 -> ON，低于阈值 -> OFF。
        if (above != prev_above_threshold_)
        {
            prev_above_threshold_ = above;
            const bool new_suck = above;
            const bool ok = apply_suck(new_suck);
            RCLCPP_INFO(this->get_logger(),
                        "MANUAL SET (crossed threshold -> %s): request=[%s], applied=%d, current=[%s]",
                        above ? "above" : "below",
                        new_suck ? "ON" : "OFF",
                        static_cast<int>(ok),
                        target_suck_ ? "ON" : "OFF");
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