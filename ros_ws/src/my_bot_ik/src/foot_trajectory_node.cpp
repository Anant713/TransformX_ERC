#include <cmath>
#include <memory>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

class FootTrajectoryNode : public rclcpp::Node
{
public:
    FootTrajectoryNode() : Node("foot_trajectory_node"), t_(0.0)
    {
        pubs_[0] = this->create_publisher<geometry_msgs::msg::Point>("/leg1/foot_target", 10);
        pubs_[1] = this->create_publisher<geometry_msgs::msg::Point>("/leg2/foot_target", 10);
        pubs_[2] = this->create_publisher<geometry_msgs::msg::Point>("/leg3/foot_target", 10);
        pubs_[3] = this->create_publisher<geometry_msgs::msg::Point>("/leg4/foot_target", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&FootTrajectoryNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        t_ += 0.05;

        for (int i = 0; i < 4; ++i) {
            geometry_msgs::msg::Point p;

            p.x = (i < 2) ? 0.15 : -0.15;
            p.y = (i % 2 == 0) ? 0.12 : -0.12;
            p.z = -0.18 + 0.03 * std::sin(t_);

            pubs_[i]->publish(p);
        }
    }

    double t_;
    std::array<rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr, 4> pubs_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FootTrajectoryNode>());
    rclcpp::shutdown();
    return 0;
}