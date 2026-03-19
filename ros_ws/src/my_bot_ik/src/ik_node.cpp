#include <cmath>
#include <array>
#include <vector>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using std::placeholders::_1;

namespace my_bot_ik
{
bool inverseKinematics(
    double x, double y, double z,
    double a, double b, double c,
    double &theta1, double &theta2, double &theta3)
{
    const double r = std::sqrt(x * x + y * y);

    const double D = ((r - a) * (r - a) + z * z - b * b - c * c) / (2.0 * b * c);

    if (D > 1.0 || D < -1.0) {
        return false;
    }

    theta3 = std::atan2(std::sqrt(1.0 - D * D), D);

    theta2 = std::atan2(-z, r - a)
           - std::atan2(c * std::sin(theta3),
                        b + c * std::cos(theta3));

    theta1 = std::atan2(x, y);

    return true;
}
}

class IKNode : public rclcpp::Node
{
public:
    IKNode() : Node("ik_node")
    {
        this->declare_parameter<double>("a", 0.05);
        this->declare_parameter<double>("b", 0.10);
        this->declare_parameter<double>("c", 0.10);

        this->declare_parameter<std::vector<double>>("xl", {0.15, 0.15, -0.15, -0.15});
        this->declare_parameter<std::vector<double>>("yl", {0.10, -0.10, 0.10, -0.10});
        this->declare_parameter<std::vector<double>>("zl", {0.0, 0.0, 0.0, 0.0});
        this->declare_parameter<std::vector<double>>("side_sign", {1.0, -1.0, 1.0, -1.0});

        a_ = this->get_parameter("a").as_double();
        b_ = this->get_parameter("b").as_double();
        c_ = this->get_parameter("c").as_double();

        auto xlv = this->get_parameter("xl").as_double_array();
        auto ylv = this->get_parameter("yl").as_double_array();
        auto zlv = this->get_parameter("zl").as_double_array();
        auto ssv = this->get_parameter("side_sign").as_double_array();

        for (size_t i = 0; i < 4; ++i) {
            xl_[i] = xlv[i];
            yl_[i] = ylv[i];
            zl_[i] = zlv[i];
            side_sign_[i] = ssv[i];
        }

        joint_names_ = {
            "joint1", "joint2", "joint3",
            "joint4", "joint5", "joint6",
            "joint7", "joint8", "joint9",
            "joint10", "joint11", "joint12"
        };

        traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);

        for (int i = 0; i < 4; ++i) {
            auto topic = "/leg" + std::to_string(i + 1) + "/foot_target";
            foot_subs_[i] = this->create_subscription<geometry_msgs::msg::Point>(
                topic, 10,
                [this, i](geometry_msgs::msg::Point::SharedPtr msg)
                {
                    foot_targets_[i] = *msg;
                    got_target_[i] = true;
                    publish_joint_command();
                });
        }

        RCLCPP_INFO(this->get_logger(), "IK node started.");
    }

private:
    void publish_joint_command()
    {
        for (bool ok : got_target_) {
            if (!ok) {
                return;
            }
        }

        std::array<double, 12> angles{};

        for (int i = 0; i < 4; ++i) {
            double x_local = foot_targets_[i].x - xl_[i];
            double y_local = (foot_targets_[i].y - yl_[i]) * side_sign_[i];
            double z_local = foot_targets_[i].z - zl_[i];

            double t1, t2, t3;
            bool reachable = my_bot_ik::inverseKinematics(
                x_local, y_local, z_local,
                a_, b_, c_,
                t1, t2, t3);

            if (!reachable) {
                RCLCPP_WARN(this->get_logger(), "Leg %d target unreachable.", i + 1);
                return;
            }

            t1 *= side_sign_[i];

            angles[i * 3 + 0] = t1;
            angles[i * 3 + 1] = t2;
            angles[i * 3 + 2] = t3;
        }

        trajectory_msgs::msg::JointTrajectory traj;
        traj.joint_names = joint_names_;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.resize(12);

        for (size_t i = 0; i < 12; ++i) {
            point.positions[i] = angles[i];
        }

        point.time_from_start.sec = 0;
        point.time_from_start.nanosec = 200000000;

        traj.points.push_back(point);
        traj_pub_->publish(traj);
    }

    double a_, b_, c_;
    std::array<double, 4> xl_{};
    std::array<double, 4> yl_{};
    std::array<double, 4> zl_{};
    std::array<double, 4> side_sign_{};

    std::array<geometry_msgs::msg::Point, 4> foot_targets_{};
    std::array<bool, 4> got_target_{false, false, false, false};

    std::array<rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr, 4> foot_subs_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;

    std::vector<std::string> joint_names_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IKNode>());
    rclcpp::shutdown();
    return 0;
}