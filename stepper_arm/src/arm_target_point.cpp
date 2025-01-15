#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "stepper_arm/arm_control.h"

using namespace std;

class ArmTargetPointControlNode : public rclcpp::Node {
public:
    ArmTargetPointControlNode() : Node("arm_target_point_control_node") {
        // Create publishers
        joint_state_publisher = this->create_publisher<sensor_msgs::msg::JointState>(
            "Arm_2_JointStates", 10);

        // Create subscribers
        target_point_subscriber = this->create_subscription<geometry_msgs::msg::Point>(
            "arm_target_point", 10,
            std::bind(&ArmTargetPointControlNode::target_point_callback, this, std::placeholders::_1));

        // Create timer for publishing joint states
        timer = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&ArmTargetPointControlNode::timer_callback, this));

        // Get parameters
        this->declare_parameter("Angle_B_bias", 1.571);
        this->declare_parameter("Angle_C_bias", 0.384);
        this->declare_parameter("arm_base_height", 0.0);

        this->get_parameter("Angle_B_bias", Angle_B_bias);
        this->get_parameter("Angle_C_bias", Angle_C_bias);
        this->get_parameter("arm_base_height", arm_base_height);

        RCLCPP_INFO(this->get_logger(), "Arm target point control node initialized");
    }

private:
    void target_point_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        double target_x = msg->x;
        double target_y = msg->y;

        RCLCPP_INFO(this->get_logger(), "Received target point: x=%f, y=%f", target_x, target_y);

        // Check if target is within range
        int range_check = arm_range_judge(target_x, target_y);
        if (range_check == 0) {
            double target_joint_b, target_joint_c;
            // Calculate inverse kinematics to get required joint angles
            arm_2_inverse_solution(target_x, target_y, target_joint_b, target_joint_c);
            
            // Update the global joint positions
            Joint_B = target_joint_b;
            Joint_C = target_joint_c;
            
            // Update the end position tracking
            arm_2_positive_solution(ArmC_End_Position_X, ArmC_End_Position_Y, Joint_B, Joint_C);
            RCLCPP_INFO(this->get_logger(), "Moving to target point");
        } else {
            RCLCPP_ERROR(this->get_logger(), 
                "Target point out of range (error code: %d)", range_check);
        }
    }

    void timer_callback() {
        sensor_msgs::msg::JointState joint_states;
        joint_states.header.stamp = this->get_clock()->now();
        
        // Add joint positions in the correct order
        joint_states.position.push_back(Joint_A);
        joint_states.position.push_back(Joint_B - Angle_B_bias);
        joint_states.position.push_back(Joint_C - Angle_C_bias);
        joint_states.position.push_back(Joint_End);
        joint_states.position.push_back(Joint_Hand_left1);
        joint_states.position.push_back(Joint_Hand_left2);
        joint_states.position.push_back(Joint_Hand_right1);
        joint_states.position.push_back(Joint_Hand_right2);
        joint_states.position.push_back(Joint_Grasper);

        // Add joint names
        joint_states.name = {
            "joint_a", "joint_b", "joint_c", "joint_end",
            "joint_hand_left1", "joint_hand_left2",
            "joint_hand_right1", "joint_hand_right2",
            "joint_grasper"
        };

        joint_state_publisher->publish(joint_states);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_point_subscriber;
    rclcpp::TimerBase::SharedPtr timer;

    double Angle_B_bias;
    double Angle_C_bias;
    double arm_base_height;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmTargetPointControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}