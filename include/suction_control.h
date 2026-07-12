//
// Created by zihao on 2026/6/11.
//

#ifndef SUCTION_CONTROL_SUCTION_CONTROL_H
#define SUCTION_CONTROL_SUCTION_CONTROL_H

#include <filesystem>
#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "custom_msgs/msg/read_sbusrc.hpp"
#include "suction_control/srv/set_suction.hpp"

namespace suction_control
{
    class R2SuctionControlNode : public rclcpp::Node
    {
    public:
        explicit R2SuctionControlNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        ~R2SuctionControlNode() override;

    private:

        std::vector<std::string> discover_serial_ports() const;
        bool open_serial_port(const std::string & serial_port);

        void init_relay();         // 启动时尽力打开一次串口（失败不致命）
        bool try_open_relay();     // 串口未打开时尝试（重新）打开并配置；已打开返回 true

        // 写串口继电器
        bool set_valve(bool suck);

        // 统一的吸盘状态设置入口：更新 target_suck_ 并驱动阀门
        bool apply_suck(bool suck);

        void on_rc_read(const custom_msgs::msg::ReadSBUSRC::SharedPtr msg);
        void on_auto_suck_trigger(const std_msgs::msg::Bool::SharedPtr msg);
        void on_set_suction(const suction_control::srv::SetSuction::Request::SharedPtr req,
                            suction_control::srv::SetSuction::Response::SharedPtr res);

        std::string rc_topic_;
        std::string serial_port_;
        int channel_index_;
        int suck_threshold_;

        // 吸盘当前目标状态（由 RC / 自动 topic / 服务 三者共同维护）
        bool target_suck_;

        // RC 拨杆边缘检测状态
        bool has_prev_rc_;          // 是否已收到过首个 RC 样本
        bool prev_above_threshold_; // 上一帧拨杆是否处于阈值之上

        int relay_fd_;

        rclcpp::Subscription<custom_msgs::msg::ReadSBUSRC>::SharedPtr rc_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr auto_suck_sub_;
        rclcpp::Service<suction_control::srv::SetSuction>::SharedPtr set_suction_srv_;
    };
}

#endif //SUCTION_CONTROL_SUCTION_CONTROL_H
