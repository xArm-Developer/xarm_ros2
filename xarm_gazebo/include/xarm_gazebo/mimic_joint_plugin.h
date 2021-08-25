/* 

Copyright (c) 2014, Konstantinos Chatzilygeroudis
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer
    in the documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Refer to: https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins

Modified for ROS2 compatibility and UFACTORY xArm gripper simulation

Modified by: Vinman <vinman.cub@gmail.com>
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