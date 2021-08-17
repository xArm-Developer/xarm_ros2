/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 * 
 * Refer to: https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins
 ============================================================================*/
#ifndef XARM_GAZEBO_MIMIC_JOINT_PLUGIN_H
#define XARM_GAZEBO_MIMIC_JOINT_PLUGIN_H

#include <rclcpp/rclcpp.hpp>

#include "controller_manager/controller_manager.hpp"

// ros_control
#include <control_toolbox/pid.hpp>

// Gazebo includes
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo_ros/node.hpp>

namespace gazebo {
  class GazeboMimicJointPlugin : public ModelPlugin {
  public:
    GazeboMimicJointPlugin();
    virtual ~GazeboMimicJointPlugin() override;

    // Overloaded Gazebo entry point
    virtual void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf) override;

  private:
    void OnUpdate();

    // Node Handles
    gazebo_ros::Node::SharedPtr model_nh_;
    // Pointer to the update event connection
    gazebo::event::ConnectionPtr update_connection_;

    // PID controller if needed
    std::shared_ptr<control_toolbox::Pid> pid_;

    // Pointers to the joints
    physics::JointPtr joint_, mimic_joint_;

    // Pointer to the model
    physics::ModelPtr model_;

    // Parameters
    std::string joint_name_, mimic_joint_name_;
    double multiplier_, offset_, sensitiveness_, max_effort_;
    bool has_pid_;

    std::shared_ptr<rclcpp::Rate> loop_rate_;
  };
}


#endif