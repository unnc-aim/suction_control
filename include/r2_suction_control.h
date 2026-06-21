//
// Created by zihao on 2026/6/11.
//

#ifndef R2_SUCTION_CONTROL_R2_SUCTION_CONTROL_H
#define R2_SUCTION_CONTROL_R2_SUCTION_CONTROL_H

#include <string>
#include <memory>

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

        void init_relay();
        void set_valve(bool suck);

        void on_rc_read(const custom_msgs::msg::ReadSBUSRC::SharedPtr msg);
        void on_auto_suck_trigger(const std_msgs::msg::Bool::SharedPtr msg);

        std::string rc_topic_;
        std::string serial_port_;
        int channel7_index_;
        int suck_threshold_;

        bool target_suck_;


        int relay_fd_;

        rclcpp::Subscription<custom_msgs::msg::ReadSBUSRC>::SharedPtr rc_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr auto_suck_sub_;
    };
}

#endif //R2_SUCTION_CONTROL_R2_SUCTION_CONTROL_H