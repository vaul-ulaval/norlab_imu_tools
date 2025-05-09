// This node observes gyro bias on a given topic for a given time and then publishes the observation on a given topic.
// The system must be stationary during the estimation.
// Author: Vladimir Kubelka, kubelvla@gmail.com
// Maintainer: Haha, good luck
// Licence: BSD
// Laval University, NORLAB, 2019

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <cmath>
#include <sstream>

namespace norlab_imu_tools {
class imuBiasObserverNode : public rclcpp::Node
{
public:
    imuBiasObserverNode(const rclcpp::NodeOptions& options) :
            Node("imu_bias_observer_node", options)
    {
        bias_pub = this->create_publisher<sensor_msgs::msg::Imu>("bias_topic_out", 10);

        imu_subscription = this->create_subscription<sensor_msgs::msg::Imu>("imu_topic_in", 10,
                                                                            std::bind(&imuBiasObserverNode::imuMsgCallback, this,
                                                                                      std::placeholders::_1));

        led_mode_pub = this->create_publisher<std_msgs::msg::String>("/leds/mode", 10);
        led_color_pub = this->create_publisher<std_msgs::msg::ColorRGBA>("/leds/color", 10);

        this->declare_parameter<int>("target_observation_samples", 4000);
        this->get_parameter("target_observation_samples", target_observation_samples);

        observe_now = true;

    }
private:
    double angular_velocity_sum_x = 0.0;
    double angular_velocity_sum_y = 0.0;
    double angular_velocity_sum_z = 0.0;
    double linear_acceleration_sum_x = 0.0;
    double linear_acceleration_sum_y = 0.0;
    double linear_acceleration_sum_z = 0.0;
    int number_of_samples = 0;
    int target_observation_samples = 4000;
    bool observe_now = false;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr bias_pub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr led_mode_pub;
    rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr led_color_pub;


    void imuMsgCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
        if(observe_now)
        {
            start_leds_blinking();
            if(number_of_samples <= target_observation_samples)
            {
                angular_velocity_sum_x += imu_msg->angular_velocity.x;
                angular_velocity_sum_y += imu_msg->angular_velocity.y;
                angular_velocity_sum_z += imu_msg->angular_velocity.z;
                linear_acceleration_sum_x += imu_msg->linear_acceleration.x;
                linear_acceleration_sum_y += imu_msg->linear_acceleration.y;
                linear_acceleration_sum_z += imu_msg->linear_acceleration.z;
                number_of_samples += 1;

                if(number_of_samples % (target_observation_samples/5) == 0){
                    RCLCPP_INFO(this->get_logger(), "IMU bias observer: Collected %d samples (%d%%) of %d.", number_of_samples, (100*number_of_samples/target_observation_samples), target_observation_samples);
                }

            }
            else
            {
                sensor_msgs::msg::Imu bias_msg;
                bias_msg.header.stamp = this->now();
                bias_msg.angular_velocity.x = angular_velocity_sum_x / double(number_of_samples);
                bias_msg.angular_velocity.y = angular_velocity_sum_y / double(number_of_samples);
                bias_msg.angular_velocity.z = angular_velocity_sum_z / double(number_of_samples);
                bias_msg.linear_acceleration.x = linear_acceleration_sum_x / double(number_of_samples);
                bias_msg.linear_acceleration.y = linear_acceleration_sum_y / double(number_of_samples);
                bias_msg.linear_acceleration.z = linear_acceleration_sum_z / double(number_of_samples);
                bias_pub->publish(bias_msg);

                observe_now = false;
                RCLCPP_WARN(this->get_logger(), "IMU bias observer: Done, publishing the result and shutting down.");
                stop_leds_blinking();
            }

        }
    }

    void start_leds_blinking()
    {
        std_msgs::msg::String mode_msg;
        mode_msg.data = "blink";
        led_mode_pub->publish(mode_msg);
    }

    void stop_leds_blinking()
    {
        std_msgs::msg::String mode_msg;
        mode_msg.data = "normal";
        led_mode_pub->publish(mode_msg);

        std_msgs::msg::ColorRGBA color_msg;
        color_msg.r = 150.0;
        color_msg.g = 180.0;
        color_msg.b = 0.0;
        led_color_pub->publish(color_msg);
    }
};
};

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(norlab_imu_tools::imuBiasObserverNode)