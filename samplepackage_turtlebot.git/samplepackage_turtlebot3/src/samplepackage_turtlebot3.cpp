// Minimal example for getting a new turtlebot project started
// Not good coding style, this is just intended to get you started

// Original written by Tara Bartlett, 2019
// Updated 2020 Donald Dansereau
// Converted to ROS2 Jack Naylor, 2024

#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>


// Chances are, these functions could be defined within the SampleTurtlebotNode - there's really no need that these be separate functions.


// Declare some call back functions to publish velocity and receive laser data
void PublishVelocity(const std::vector<float>& lin_vel, const std::vector<float>& ang_vel)
{
    auto msg = geometry_msgs::msg::Twist();
    
    msg.linear.x = lin_vel[0];
    msg.linear.y = lin_vel[1];
    msg.linear.z = lin_vel[2];

    msg.angular.x = ang_vel[0];
    msg.angular.y = ang_vel[1];
    msg.angular.z = ang_vel[2];
    
    pub_vel_->publish(msg);
}



void LaserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Message members: https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/LaserScan.msg
    
    // Example for how to access members
    float min_angle = msg->angle_min;
}

// In ROS2 there are no "Node Handlers" - instantiate a class which inherits Node properties.
class SampleTurtlebotNode : public rclcpp::Node
{
  public:
    SampleTurtlebotNode() : Node("sample_turtlebot3_node")
    {
        // Initialize publisher and subscriber at 10 and 1000 Hz respectively
        pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        sub_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 1000, LaserCallback);

        // The timer callback will publish every 0.1s 
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&SampleTurtlebotNode::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        // Initialize two 0-vectors of length 3
        std::vector<float> lin_vel(3, 0);
        std::vector<float> ang_vel(3, 0);
        lin_vel[0] = 1; // Go forwards

        // Publish the velocity
        PublishVelocity(lin_vel, ang_vel);
    }


    // Declare a publisher, subscriber and timer
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
    rclcpp::Subscription<sensor_msgs::msg::Laser_scan>::SharedPtr sub_laser_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char** argv)
{
    // Create a ROS node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SampleTurtlebotNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
