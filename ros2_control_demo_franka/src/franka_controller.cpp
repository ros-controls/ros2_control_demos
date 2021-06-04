// source code for Franka demo controller

#include <franka_controller.h>

namespace ros2_control_demo{

// MAIN NODE SOURCE
FrankaDemo::FrankaDemo() :
    Node(rclcpp::Node::make_shared("franka_demo")),
    franka_ik_controller_(Node),
    franka_model_(Node)
{

}

// CONTROLLER SOURCE
FrankaController::FrankaController(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<FrankaModel> model) :
    node_(node),
    robot_(model),
    joint_states_subscriber_(node_->create_subscription<sensor_msgs::JointState>("joint_states", 10, std::bind(&FrankaController::joint_states_callback_, this, _1))),
    buffer_(node_->get_clock()),
    transform_listener_(std::make_shared<tf2_ros::TransformListener>(buffer_))
{
    // controller. handles subscriptions, publications, control logic
    // TODO: reset the end effector target using forward kinematics when the joint positions are reset
    // TODO: control loop should run at fixed frequency

    // get the default joint positions from the parameter server

    // reset the joint positions
    robot_->reset_joint_positions(initial_joint_position);

    joint_target_.position.resize(robot_.get_joint_names.size());
    joint_target_.velocity.resize(robot_.get_joint_names.size());
    joint_target_.acceleration.resize(robot_.get_joint_names.size());

    end_effector_target_.position.resize(6);
    end_effector_target_.velocity.resize(6);
    end_effector_target_.acceleration.resize(6);

    joint_state_.position.resize(robot_.get_joint_names.size());
    joint_state_.velocity.resize(robot_.get_joint_names.size());
    joint_state_.acceleration.resize(robot_.get_joint_names.size());

    end_effector_state_.position.resize(6);
    end_effector_state_.velocity.resize(6);
    end_effector_state_.acceleration.resize(6);
}

void FrankaController::joint_states_callback_(const sensor_msgs::msg::JointState::SharedPtr msg)
{   
    for(int i = 0; i < robot_.get_joint_names.size(); i++){ // TODO: clean up this for-loop
        joint_state_.position[i] = msg->position[i];
        joint_state_.velocity[i] = msg->velocity[i];
        joint_state_.effort[i] = msg->effort[i];
    }

    // lookup the transform for the end effector (its location w.r.t. the base)
    try{
        transform_base_to_ee_ = tf_buffer_.lookupTransform("end_effector_frame", "base_link", tf2::get_now(), tf2::durationFromSec(0));
    }
    catch(tf2_ros::TransformException &ex){
        ROS_WARN("%s",ex.what());
        continue;
    }
    end_effector_state_.position[0] = transform_base_to_ee_.transform.translation.x;
    end_effector_state_.position[1] = transform_base_to_ee_.transform.translation.y;
    end_effector_state_.position[2] = transform_base_to_ee_.transform.translation.z;
    end_effector_state_.position[3] = transform_base_to_ee_.transform.orientation.x;
    end_effector_state_.position[4] = transform_base_to_ee_.transform.orientation.y;
    end_effector_state_.position[5] = transform_base_to_ee_.transform.orientation.z;
    end_effector_state_.position[6] = transform_base_to_ee_.transform.orientation.w;
}

void FrankaController::set_controller_period_(float dT){ control_period_ = dT; }

void FrankaController::add_panda_controller_(float dT){

    // Set the controller period
    set_controller_period_(dT);

    // Increase the maximum effort of the fingers
    // TODO: set maximum force of joint

    // Set joint targets (position, velocity, acceleration)
    end_effector_target_.position[0];
    end_effector_target_.position[1];

    // set the joint positions based on the output of the IK controller

    return true; // controller successfully launched
}

// ROBOT MODEL SOURCE
FrankaModel::FrankaModel(std::shared_ptr<rclcpp::Node> node) :
    node_(node)
{

    // Create the services for resetting the simulation: resetting the joint positions, pausing, and unpausing Gazebo
    pause_gazebo_srv_ = node_->create_service<std_srvs::Empty>("/gazebo/pause_physics");
    set_joint_position_srv_ = node_->create_service<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
    unpause_gazebo_srv_ = node_->create_service<std_srvs::Empty>("/gazebo/unpause_physics");

    // read the initial joint positions from the parameter server (loaded by the params yaml file)
}

geometry_msgs::Pose FrankaModel::get_initial_pose(){

    geometry_msgs::Pose initial_pose;
    initial_pose.orientation.w = 1.0;

    return initial_pose;
}

void FrankaModel::reset_joint_positions(std::vector<float> joint_positions){
    // make sure that we're specifying joint positions for all the joints
    int num_joints = std::min(joint_names.size(), joint_positions.size());
    assert(num_joints == joint_names_.size());

    auto model_config_srv_request = std::make_shared<gazebo_msgs::SetModelConfiguration>();
    model_config_srv_request->model_name = model_name_;
    model_config_srv_request->joint_names.resize(joint_names_.size());
    model_config_srv_request->joint_positions.resize(joint_names_.size());

    for(int i = 0; i < joint_names_.size(); i++){
        model_config_srv_request->joint_names[i] = joint_names[i];
        model_config_srv_request->joint_positions[i] = joint_positions[i];
    }

    // call the service to reset the joint positions
    auto result = pause_gazebo_srv_->async_send_request(std::make_shared<std_srvs::Empty>());
    result = set_joint_position_srv_->async_send_request(model_config_srv_request);
    result = unpause_gazebo_srv_->async_send_request(std::make_shared<std_srvs::Empty>());

}

void FrankaModel::set_max_generalized_force(std::string joint_name, float max_force)
{
    // TODO: set maximum force of joint
}

std::vector<std::string> FrankaModel::get_joint_names()
{
    return joint_names_;
}

}