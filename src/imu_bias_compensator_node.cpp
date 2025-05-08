#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <cmath>
#include <sstream>

namespace norlab_imu_tools
{
class imuBiasCompensatorNode : public rclcpp::Node
{
public:
    imuBiasCompensatorNode(const rclcpp::NodeOptions& options) :
            Node("imu_bias_compensator_node", options)
    {
        biasSub = this->create_subscription<sensor_msgs::msg::Imu>("bias_topic_in", 10,
                                                                                std::bind(&imuBiasCompensatorNode::biasMsgCallback, this,
                                                                                           std::placeholders::_1));

        imuSubscription = this->create_subscription<sensor_msgs::msg::Imu>("imu_topic_in", 10,
                                                                           std::bind(&imuBiasCompensatorNode::imuMsgCallback, this,
                                                                                      std::placeholders::_1));
        imuCompensatedPub = this->create_publisher<sensor_msgs::msg::Imu>("imu_topic_out", 10);
    }
private:
    double xVelBias = 0.0;
    double yVelBias = 0.0;
    double zVelBias = 0.0;
    double xAccelBias = 0.0;
    double yAccelBias = 0.0;
    double zAccelBias = 0.0;
    bool bias_acquired = false;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuCompensatedPub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscription;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr biasSub;

    void biasMsgCallback(const sensor_msgs::msg::Imu::SharedPtr biasMsg)
    {
        this->xVelBias = biasMsg->angular_velocity.x;
        this->yVelBias = biasMsg->angular_velocity.y;
        this->zVelBias = biasMsg->angular_velocity.z;
        this->xAccelBias = biasMsg->linear_acceleration.x;
        this->yAccelBias = biasMsg->linear_acceleration.y;
        this->zAccelBias = biasMsg->linear_acceleration.z;
        bias_acquired = true;
        RCLCPP_INFO(this->get_logger(), "Bias acquired.");
    }

    void imuMsgCallback(const sensor_msgs::msg::Imu::SharedPtr imuMsg) {

        if (bias_acquired)
        {
            sensor_msgs::msg::Imu imuMsgUnbiased;
            imuMsgUnbiased = *imuMsg;
            imuMsgUnbiased.angular_velocity.x -= this->xVelBias;
            imuMsgUnbiased.angular_velocity.y -= this->yVelBias;
            imuMsgUnbiased.angular_velocity.z -= this->zVelBias;
            imuMsgUnbiased.linear_acceleration.x -= this->xAccelBias;
            imuMsgUnbiased.linear_acceleration.y -= this->yAccelBias;
            imuMsgUnbiased.linear_acceleration.z -= this->zAccelBias;
            imuCompensatedPub->publish(imuMsgUnbiased);
        }
    }
};
};


#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(norlab_imu_tools::imuBiasCompensatorNode)