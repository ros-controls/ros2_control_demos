// header for Franka arm demo node

#ifndef FRANKA_DEMO_CONTROLLER_H
#define FRANKA_DEMO_CONTROLLER_H

// system headers
#include <stdio.h>
#include <assert.h>

// ROS headers
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// iDynTree headers
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/InverseKinematics.h>

// C headers
#include <cmath>
#include <cstdlib>

namespace ros2_control_demo{

class FrankaController
{
    // controller. handles subscriptions, publications, control logic
    // hold the arm in place if no new trajectory points received
    // TODO: Double check that the controller PID gains are being set correctly
    public:

        FrankaController(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<FrankaModel> model);
        void set_joint_position_targets(std::vector<float> position_targets);
        void set_joint_velocity_targets(std::vector<float> velocity_targets);
        void set_joint_acceleration_targets(std::vector<float> acceleration_targets);

    private:

        void control_callback_();
        void set_controller_period_(float dT);
        bool add_panda_controller_(float dT);
        void joint_states_callback_();
        void get_panda_ik_(std::vector<std::string> optimized_joints); // TODO: read joint names from the parameter server
        void solve_ik_(std::vector<float> target_position, std::vector<float> target_orientation, ik_nlp); // TODO: fill in the correct function parameters for the IK solver

        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<FrankaModel> robot_;
        float control_period_;
        const std::string joint_control_topic_ = "joint_group_position_controller/command";
        
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_commands_publisher_;

        tf2_ros::Buffer tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        geometry_msgs::TransformStamped transform_base_to_ee_;

        struct state{
            std::vector<float> position;
            std::vector<float> velocity;
            std::vector<float> acceleration;
        } joint_target_, end_effector_target_, joint_state_, end_effector_state_;
    
};

class FrankaModel
{
    // contains the model for the Franka robot
    public:

        FrankaModel(std::shared_ptr<rclcpp::Node> node);
        void reset_joint_positions(std::vector<float> joint_positions);
        geometry_msgs::Pose get_initial_pose();
        void set_max_generalized_force(std::string joint_name, float max_force);
        std::vector<std::string> get_joint_names();

    private:

        void load_model_(const std::string model_dir);

        std::shared_ptr<rclcpp::Node> node_;
        std::string model_name_;
        geometry_msgs::Pose initial_pose_;
        rclcpp::Service<gazebo_msgs::SetModelConfiguration>::SharedPtr set_joint_position_srv_;
        rclcpp::Service<std_srvs::Empty>::SharedPtr pause_gazebo_srv_;
        rclcpp::Service<std_srvs::Empty>::SharedPtr unpause_gazebo_srv_;
        std::vector<std::string> joint_names_;

        // for kinematic model
        iDynTree::ModelLoader mdlLoader_;
        iDynTree::KinDynComputations forward_kinematics_;
        iDynTree::InverseKinematics inverse_kinematics_;
}

class FrankaDemo : public rclcpp::Node
{
    // main class. contains class members for controller, model, etc.
    public:

        FrankaDemo();

    private:

        std::shared_ptr<FrankaController> franka_ik_controller_;
        std::shared_ptr<FrankaModel> franka_model_;
};

}