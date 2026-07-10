//
// Created by zihao on 2026/6/11.
//

#ifndef SUCTION_CONTROL_SUCTION_CONTROL_H
#define SUCTION_CONTROL_SUCTION_CONTROL_H

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "custom_msgs/msg/read_sbusrc.hpp"

namespace suction_control
{
    class R2SuctionControlNode : public rclcpp::Node
    {
    public:
        explicit R2SuctionControlNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        ~R2SuctionControlNode() override;

    private:

        std::vector<std::string> scan_serial_ports() const;
        void refresh_serial_port_by_diff();
        void close_relay();
        void init_relay();
        void set_valve(bool suck);

        void on_rc_read(const custom_msgs::msg::ReadSBUSRC::SharedPtr msg);
        void on_auto_suck_trigger(const std_msgs::msg::Bool::SharedPtr msg);

        std::string rc_topic_;
        std::string serial_port_;
        int channel_index_;
        int estop_channel_index_;
        int suck_threshold_;

        bool target_suck_;
        bool estop_active_;
        bool manual_ch_high_initialized_;
        bool manual_ch_high_;


        int relay_fd_;
        std::vector<std::string> last_serial_ports_;

        rclcpp::Subscription<custom_msgs::msg::ReadSBUSRC>::SharedPtr rc_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr auto_suck_sub_;
    };
}

#endif //SUCTION_CONTROL_SUCTION_CONTROL_H