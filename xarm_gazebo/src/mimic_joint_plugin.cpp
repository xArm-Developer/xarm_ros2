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

#include "xarm_gazebo/mimic_joint_plugin.h"


namespace gazebo {
  GazeboMimicJointPlugin::GazeboMimicJointPlugin()
  {
    joint_.reset();
    mimic_joint_.reset();
  }

  GazeboMimicJointPlugin::~GazeboMimicJointPlugin()
  {
    update_connection_.reset();
  }

  void GazeboMimicJointPlugin::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf)
  {
    model_ = parent;
    model_nh_ = gazebo_ros::Node::Get(sdf);

    RCLCPP_INFO(
      model_nh_->get_logger(), "Starting GazeboMimicJointPlugin in namespace: %s, name: %s",
      model_nh_->get_namespace(), model_nh_->get_name());

    // Error message if the model couldn't be found
    if (!model_) {
      RCLCPP_ERROR_STREAM(model_nh_->get_logger(), "parent model is NULL");
      return;
    }

    // Check that ROS has been initialized
    if (!rclcpp::ok()) {
      RCLCPP_FATAL_STREAM(
        model_nh_->get_logger(),
        "A ROS node for Gazebo has not been initialized, unable to load plugin. " <<
          "Load the Gazebo system plugin 'libgazebo_ros2_mimic_joint_plugin.so' in the gazebo_ros package)");
      return;
    }

    // Check for joint element
    if (!sdf->HasElement("joint")) {
      RCLCPP_ERROR(model_nh_->get_logger(), "No joint element present. GazeboMimicJointPlugin could not be loaded.");
      return;
    }

    joint_name_ = sdf->GetElement("joint")->Get<std::string>();

    // Check for mimicJoint element
    if (!sdf->HasElement("mimicJoint")) {
      RCLCPP_ERROR(model_nh_->get_logger(), "No mimicJoint element present. GazeboMimicJointPlugin could not be loaded.");
      return;
    }

    mimic_joint_name_ = sdf->GetElement("mimicJoint")->Get<std::string>();

    // Check if PID controller wanted
    has_pid_ = sdf->HasElement("hasPID");
    if (has_pid_) {
      const std::string prefix = "gains." + joint_name_;
      const auto k_p = model_nh_->declare_parameter<double>(prefix + ".p", 10.0);
      const auto k_i = model_nh_->declare_parameter<double>(prefix + ".i", 0.1);
      const auto k_d = model_nh_->declare_parameter<double>(prefix + ".d", 0.0);
      const auto i_clamp = model_nh_->declare_parameter<double>(prefix + ".i_clamp", 0.2);
      // Initialize PID
      pid_ = std::make_shared<control_toolbox::Pid>(k_p, k_i, k_d, i_clamp, -i_clamp);
    }

    // Check for multiplier element
    multiplier_ = 1.0;
    if (sdf->HasElement("multiplier"))
      multiplier_ = sdf->GetElement("multiplier")->Get<double>();

    // Check for offset element
    offset_ = 0.0;
    if (sdf->HasElement("offset"))
      offset_ = sdf->GetElement("offset")->Get<double>();

    // Check for sensitiveness element
    sensitiveness_ = 0.0;
    if (sdf->HasElement("sensitiveness"))
      sensitiveness_ = sdf->GetElement("sensitiveness")->Get<double>();

    // Get pointers to joints
    joint_ = model_->GetJoint(joint_name_);
    if (!joint_) {
      RCLCPP_ERROR(model_nh_->get_logger(), "No joint named \"%s\". GazeboMimicJointPlugin could not be loaded.", joint_name_.c_str());
      return;
    }
    mimic_joint_ = model_->GetJoint(mimic_joint_name_);
    if (!mimic_joint_) {
      RCLCPP_ERROR(model_nh_->get_logger(), "No (mimic) joint named \"%s\". GazeboMimicJointPlugin could not be loaded.", mimic_joint_name_.c_str());
      return;
    }

      // Check for max effort
    if (sdf->HasElement("maxEffort")) {
      max_effort_ = sdf->GetElement("maxEffort")->Get<double>();
    }
    else {
      max_effort_ = mimic_joint_->GetEffortLimit(0);
    }

    // Set max effort
    if (!has_pid_) {
      mimic_joint_->SetParam("fmax", 0, max_effort_);
    }

    loop_rate_ = rclcpp::Rate::make_shared(1.0 / model_->GetWorld()->Physics()->GetMaxStepSize());

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&GazeboMimicJointPlugin::OnUpdate, this));

    // Output some confirmation
    RCLCPP_INFO_STREAM(model_nh_->get_logger(), "GazeboMimicJointPlugin loaded! Joint: \"" << joint_name_ << "\", Mimic joint: \"" << mimic_joint_name_ << "\""
                                                          << ", Multiplier: " << multiplier_ << ", Offset: " << offset_
                                                          << ", MaxEffort: " << max_effort_ << ", Sensitiveness: " << sensitiveness_);
  }

  void GazeboMimicJointPlugin::OnUpdate()
  {
    // Set mimic joint's angle based on joint's angle
    double angle = joint_->Position(0) * multiplier_ + offset_;
    double a = mimic_joint_->Position(0);

    if (fabs(angle - a) >= sensitiveness_) {
      if (has_pid_) {
        double error = angle - a;
        double effort = ignition::math::clamp(pid_->computeCommand(error, loop_rate_->period().count()), -max_effort_, max_effort_);
        mimic_joint_->SetForce(0, effort);
      }
      else {
        mimic_joint_->SetPosition(0, angle, true);
      }
    }
  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboMimicJointPlugin)

}  // namespace gazebo