/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2017, Philipp Sebastian Ruppel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <bio_ik/goal.h>

#include <bio_ik/forward_kinematics.h>
#include <bio_ik/ik_base.h>
#include <bio_ik/ik_parallel.h>
#include <bio_ik/problem.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <kdl_parser/kdl_parser.hpp>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <pluginlib/class_list_macros.hpp>
#include <srdfdom/model.h>
#include <urdf/model.h>
#include <urdf_model/model.h>

#include <tf2_eigen/tf2_eigen.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <atomic>
#include <mutex>
#include <random>
#include <tuple>
#include <type_traits>

#include <bio_ik/goal_types.h>

// implement BioIKKinematicsQueryOptions

namespace bio_ik {

std::mutex bioIKKinematicsQueryOptionsMutex;
std::unordered_set<const void *> bioIKKinematicsQueryOptionsList;

BioIKKinematicsQueryOptions::BioIKKinematicsQueryOptions()
    : replace(false), solution_fitness(0) {
  std::lock_guard<std::mutex> lock(bioIKKinematicsQueryOptionsMutex);
  bioIKKinematicsQueryOptionsList.insert(this);
}

BioIKKinematicsQueryOptions::~BioIKKinematicsQueryOptions() {
  std::lock_guard<std::mutex> lock(bioIKKinematicsQueryOptionsMutex);
  bioIKKinematicsQueryOptionsList.erase(this);
}

bool isBioIKKinematicsQueryOptions(const void *ptr) {
  std::lock_guard<std::mutex> lock(bioIKKinematicsQueryOptionsMutex);
  return bioIKKinematicsQueryOptionsList.find(ptr) !=
         bioIKKinematicsQueryOptionsList.end();
}

const BioIKKinematicsQueryOptions *
toBioIKKinematicsQueryOptions(const void *ptr) {
  if (isBioIKKinematicsQueryOptions(ptr))
    return (const BioIKKinematicsQueryOptions *)ptr;
  else
    return 0;
}

} // namespace bio_ik

// BioIK Kinematics Plugin

namespace bio_ik_kinematics_plugin {

struct BioIKKinematicsPlugin : kinematics::KinematicsBase {
  std::vector<std::string> joint_names, link_names;
  const moveit::core::JointModelGroup *joint_model_group;
  mutable std::unique_ptr<bio_ik::IKParallel> ik;
  mutable std::vector<double> state, temp;
  mutable std::unique_ptr<moveit::core::RobotState> temp_state;
  mutable std::vector<bio_ik::Frame> tipFrames;
  bio_ik::RobotInfo robot_info;
  bool enable_profiler;

  BioIKKinematicsPlugin()
    : enable_profiler(false)
    {}

  const std::vector<std::string> &getJointNames() const override {
    LOG_FNC();
    return joint_names;
  }

  const std::vector<std::string> &getLinkNames() const override {
    LOG_FNC();
    return link_names;
  }

  bool getPositionFK(const std::vector<std::string> &link_names,
                             const std::vector<double> &joint_angles,
                             std::vector<geometry_msgs::msg::Pose> &poses) const override {
    LOG_FNC();
    return false;
  }

  bool getPositionIK(const geometry_msgs::msg::Pose &ik_pose,
                             const std::vector<double> &ik_seed_state,
                             std::vector<double> &solution,
                             moveit_msgs::msg::MoveItErrorCodes &error_code,
                             const kinematics::KinematicsQueryOptions &options =
                                 kinematics::KinematicsQueryOptions()) const override {
    LOG_FNC();
    return false;
  }

  EigenSTL::vector_Isometry3d tip_reference_frames;

  mutable std::vector<std::unique_ptr<bio_ik::Goal>> default_goals;

  mutable std::vector<const bio_ik::Goal *> all_goals;

  bio_ik::IKParams ikparams;

  mutable bio_ik::Problem problem;

  bool initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModel& robot_model,
    const std::string& group_name, const std::string& base_frame,
    const std::vector<std::string>& tip_frames, double search_discretization) override;

  bool
  searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
                   const std::vector<double> &ik_seed_state, double timeout,
                   std::vector<double> &solution,
                   moveit_msgs::msg::MoveItErrorCodes &error_code,
                   const kinematics::KinematicsQueryOptions &options =
                       kinematics::KinematicsQueryOptions()) const override {
    LOG_FNC();
    return searchPositionIK(std::vector<geometry_msgs::msg::Pose>{ik_pose},
                            ik_seed_state, timeout, std::vector<double>(),
                            solution, IKCallbackFn(), error_code, options);
  }

  bool
  searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
                   const std::vector<double> &ik_seed_state, double timeout,
                   const std::vector<double> &consistency_limits,
                   std::vector<double> &solution,
                   moveit_msgs::msg::MoveItErrorCodes &error_code,
                   const kinematics::KinematicsQueryOptions &options =
                       kinematics::KinematicsQueryOptions()) const override {
    LOG_FNC();
    return searchPositionIK(std::vector<geometry_msgs::msg::Pose>{ik_pose},
                            ik_seed_state, timeout, consistency_limits,
                            solution, IKCallbackFn(), error_code, options);
  }

  bool
  searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
                   const std::vector<double> &ik_seed_state, double timeout,
                   std::vector<double> &solution,
                   const IKCallbackFn &solution_callback,
                   moveit_msgs::msg::MoveItErrorCodes &error_code,
                   const kinematics::KinematicsQueryOptions &options =
                       kinematics::KinematicsQueryOptions()) const override {
    LOG_FNC();
    return searchPositionIK(std::vector<geometry_msgs::msg::Pose>{ik_pose},
                            ik_seed_state, timeout, std::vector<double>(),
                            solution, solution_callback, error_code, options);
  }

  bool
  searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
                   const std::vector<double> &ik_seed_state, double timeout,
                   const std::vector<double> &consistency_limits,
                   std::vector<double> &solution,
                   const IKCallbackFn &solution_callback,
                   moveit_msgs::msg::MoveItErrorCodes &error_code,
                   const kinematics::KinematicsQueryOptions &options =
                       kinematics::KinematicsQueryOptions()) const override {
    LOG_FNC();
    return searchPositionIK(std::vector<geometry_msgs::msg::Pose>{ik_pose},
                            ik_seed_state, timeout, consistency_limits,
                            solution, solution_callback, error_code, options);
  }

  /*struct OptMod : kinematics::KinematicsQueryOptions
  {
      int test;
  };*/

  bool
  searchPositionIK(const std::vector<geometry_msgs::msg::Pose> &ik_poses,
                   const std::vector<double> &ik_seed_state, double timeout,
                   const std::vector<double> &consistency_limits,
                   std::vector<double> &solution,
                   const IKCallbackFn &solution_callback,
                   moveit_msgs::msg::MoveItErrorCodes &error_code,
                   const kinematics::KinematicsQueryOptions &options =
                       kinematics::KinematicsQueryOptions(),
                   const moveit::core::RobotState *context_state = NULL) const override;

  virtual bool supportsGroup(const moveit::core::JointModelGroup *jmg,
                             std::string *error_text_out = 0) const {
    LOG_FNC();
    // LOG_VAR(jmg->getName());
    return true;
  }
};
} // namespace bio_ik_kinematics_plugin

// register plugin

#undef LOG
#undef ERROR
