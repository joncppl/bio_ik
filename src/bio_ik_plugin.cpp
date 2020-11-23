//
// Created by jonathan on 2020-10-26.
//

#include <bio_ik/bio_ik_plugin.hpp>

#include "bio_ik/utils.h"

namespace bio_ik
{
std::mutex bioIKKinematicsQueryOptionsMutex;
std::unordered_set<const void *> bioIKKinematicsQueryOptionsList;
}

using namespace bio_ik;

namespace bio_ik_kinematics_plugin
{

static rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_kdl_kinematics_plugin.bio_ik_kinematics_plugin");

bool BioIKKinematicsPlugin::initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModel& robot_model,
  const std::string& group_name, const std::string& base_frame,
  const std::vector<std::string>& tip_frames, double search_discretization)
{
  LOG_FNC();

  // LOG_VAR(robot_description);
  // LOG_VAR(group_name);

  node_ = node;

  RCLCPP_INFO_STREAM(LOGGER, "Initializing BioIK for group \"" << group_name << "\"");

  storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);
  joint_model_group = robot_model_->getJointModelGroup(group_name);
  if (!joint_model_group)
    return false;

  joint_names.clear();

  for (auto *joint_model : joint_model_group->getJointModels())
    if (joint_model->getName() != base_frame_ &&
        joint_model->getType() != moveit::core::JointModel::UNKNOWN &&
        joint_model->getType() != moveit::core::JointModel::FIXED)
      joint_names.push_back(joint_model->getName());

  auto tips2 = tip_frames_;
  joint_model_group->getEndEffectorTips(tips2);
  if (!tips2.empty())
    tip_frames_ = tips2;

  link_names = tip_frames_;

  // for(auto& n : joint_names) LOG("joint", n);
  // for(auto& n : link_names) LOG("link", n);

  // bool enable_profiler;
  lookupParam(node_, "profiler", enable_profiler, false);
  // if(enable_profiler) Profiler::start();

  robot_info = RobotInfo(robot_model_);

  ikparams.robot_model = robot_model_;
  ikparams.joint_model_group = joint_model_group;

  // initialize parameters for IKParallel
  lookupParam(node_, "mode", ikparams.solver_class_name,
    std::string("bio2_memetic"));
  lookupParam(node_, "counter", ikparams.enable_counter, false);
  lookupParam(node_, "threads", ikparams.thread_count, 0);

  // initialize parameters for Problem
  lookupParam(node_, "dpos", ikparams.dpos, DBL_MAX);
  lookupParam(node_, "drot", ikparams.drot, DBL_MAX);
  lookupParam(node_, "dtwist", ikparams.dtwist, 1e-5);

  // initialize parameters for ik_evolution_1
  lookupParam(node_, "no_wipeout", ikparams.opt_no_wipeout, false);
  lookupParam(node_, "population_size", ikparams.population_size, 8);
  lookupParam(node_, "elite_count", ikparams.elite_count, 4);
  lookupParam(node_, "linear_fitness", ikparams.linear_fitness, false);

  temp_state.reset(new moveit::core::RobotState(robot_model_));

  ik.reset(new IKParallel(ikparams));

  {

    BLOCKPROFILER("default ik goals");

    default_goals.clear();

    for (size_t i = 0; i < tip_frames_.size(); i++) {
      PoseGoal *goal = new PoseGoal();

      goal->setLinkName(tip_frames_[i]);

      // LOG_VAR(goal->link_name);

      double rotation_scale = 0.5;

      lookupParam(node_, "rotation_scale", rotation_scale, rotation_scale);

      bool position_only_ik = false;
      lookupParam(node_, "position_only_ik", position_only_ik, position_only_ik);
      if (position_only_ik)
        rotation_scale = 0;

      goal->setRotationScale(rotation_scale);

      default_goals.emplace_back(goal);
    }

    {
      double weight = 0;
      lookupParam(node_, "center_joints_weight", weight, weight);
      if (weight > 0.0) {
        auto *avoid_joint_limits_goal = new bio_ik::CenterJointsGoal();
        avoid_joint_limits_goal->setWeight(weight);
        default_goals.emplace_back(avoid_joint_limits_goal);
      }
    }

    {
      double weight = 0;
      lookupParam(node_, "avoid_joint_limits_weight", weight, weight);
      if (weight > 0.0) {
        auto *avoid_joint_limits_goal = new bio_ik::AvoidJointLimitsGoal();
        avoid_joint_limits_goal->setWeight(weight);
        default_goals.emplace_back(avoid_joint_limits_goal);
      }
    }

    {
      double weight = 0;
      lookupParam(node_, "minimal_displacement_weight", weight, weight);
      if (weight > 0.0) {
        auto *minimal_displacement_goal =
          new bio_ik::MinimalDisplacementGoal();
        minimal_displacement_goal->setWeight(weight);
        default_goals.emplace_back(minimal_displacement_goal);
      }
    }
  }

  // LOG("init ready");

  RCLCPP_INFO_STREAM(LOGGER, "Finished initializing BioIK for group \"" << group_name << "\"");

  return true;
}

bool BioIKKinematicsPlugin::searchPositionIK(const std::vector<geometry_msgs::msg::Pose>& ik_poses,
  const std::vector<double>& ik_seed_state,
  double timeout,
  const std::vector<double>& consistency_limits,
  std::vector<double>& solution,
  const kinematics::KinematicsBase::IKCallbackFn& solution_callback,
  moveit_msgs::msg::MoveItErrorCodes& error_code,
  const kinematics::KinematicsQueryOptions& options,
  const moveit::core::RobotState* context_state) const
{
  double t0 = rclcpp::Clock().now().seconds();

  // timeout = 0.1;

  // LOG("a");

  if (enable_profiler)
    Profiler::start();

  auto *bio_ik_options = toBioIKKinematicsQueryOptions(&options);

  LOG_FNC();

  FNPROFILER();

  // LOG(typeid(options).name());
  // LOG(((OptMod*)&options)->test);

  // get variable default positions / context state
  state.resize(robot_model_->getVariableCount());
  if (context_state)
    for (size_t i = 0; i < robot_model_->getVariableCount(); i++)
      state[i] = context_state->getVariablePositions()[i];
  else
    robot_model_->getVariableDefaultPositions(state);

  // overwrite used variables with seed state
  solution = ik_seed_state;
  {
    int i = 0;
    for (auto &joint_name : getJointNames()) {
      auto *joint_model = robot_model_->getJointModel(joint_name);
      if (!joint_model)
        continue;
      for (size_t vi = 0; vi < joint_model->getVariableCount(); vi++)
        state.at(joint_model->getFirstVariableIndex() + vi) =
          solution.at(i++);
    }
  }

  if (!bio_ik_options || !bio_ik_options->replace) {
    // transform tips to baseframe
    tipFrames.clear();
    for (size_t i = 0; i < ik_poses.size(); i++) {
      Eigen::Isometry3d p, r;
      tf2::fromMsg(ik_poses[i], p);
      if (context_state) {
        r = context_state->getGlobalLinkTransform(getBaseFrame());
      } else {
        if (i == 0)
          temp_state->setToDefaultValues();
        r = temp_state->getGlobalLinkTransform(getBaseFrame());
      }
      tipFrames.emplace_back(r * p);
    }
  }

  // init ik

  problem.timeout = t0 + timeout;
  problem.initial_guess = state;

  // for(auto& v : state) LOG("var", &v - &state.front(), v);

  // problem.tip_objectives = tipFrames;

  /*for(size_t i = 0; i < problem.goals.size(); i++)
  {
      problem.goals[i].frame = tipFrames[i];
  }*/

  // LOG("---");

  /*{
      BLOCKPROFILER("ik goals");
      std::vector<std::unique_ptr<Goal>> goals;
      for(size_t i = 0; i < tip_frames_.size(); i++)
      {
          //if(rand() % 2) break;
          PoseGoal* goal = new PoseGoal();
          goal->link_name = tip_frames_[i];
          goal->position = tipFrames[i].pos;
          goal->orientation = tipFrames[i].rot;
          goals.emplace_back(goal);
          //if(rand() % 20) break;
      }
      //std::random_shuffle(goals.begin(), goals.end());
      //LOG_VAR(goals.size());
      setRequestGoals(problem, goals, ikparams);
  }*/

  {

    if (!bio_ik_options || !bio_ik_options->replace) {
      for (size_t i = 0; i < tip_frames_.size(); i++) {
        auto *goal = (PoseGoal *)default_goals[i].get();
        goal->setPosition(tipFrames[i].pos);
        goal->setOrientation(tipFrames[i].rot);
      }
    }

    all_goals.clear();

    if (!bio_ik_options || !bio_ik_options->replace)
      for (auto &goal : default_goals)
        all_goals.push_back(goal.get());

    if (bio_ik_options)
      for (auto &goal : bio_ik_options->goals)
        all_goals.push_back(goal.get());

    {
      BLOCKPROFILER("problem init");
      problem.initialize(ikparams.robot_model, ikparams.joint_model_group,
        ikparams, all_goals, bio_ik_options);
      // problem.setGoals(default_goals, ikparams);
    }
  }

  {
    BLOCKPROFILER("ik init");
    ik->initialize(problem);
  }

  // run ik solver
  {
    BLOCKPROFILER("ik_solve");
    ik->solve();
  }

  // get solution
  state = ik->getSolution();

  // wrap angles
  for (auto ivar : problem.active_variables) {
    auto v = state[ivar];
    if (robot_info.isRevolute(ivar) &&
      robot_model_->getMimicJointModels().empty()) {
      auto r = problem.initial_guess[ivar];
      auto lo = robot_info.getMin(ivar);
      auto hi = robot_info.getMax(ivar);

      // move close to initial guess
      if (r < v - M_PI || r > v + M_PI) {
        v -= r;
        v /= (2 * M_PI);
        v += 0.5;
        v -= std::floor(v);
        v -= 0.5;
        v *= (2 * M_PI);
        v += r;
      }

      // wrap at joint limits
      if (v > hi)
        v -= std::ceil(std::max(0.0, v - hi) / (2 * M_PI)) * (2 * M_PI);
      if (v < lo)
        v += std::ceil(std::max(0.0, lo - v) / (2 * M_PI)) * (2 * M_PI);

      // clamp at edges
      if (v < lo)
        v = lo;
      if (v > hi)
        v = hi;
    }
    state[ivar] = v;
  }

  // wrap angles
  robot_model_->enforcePositionBounds(state.data());

  // map result to jointgroup variables
  {
    solution.clear();
    for (auto &joint_name : getJointNames()) {
      auto *joint_model = robot_model_->getJointModel(joint_name);
      if (!joint_model)
        continue;
      for (size_t vi = 0; vi < joint_model->getVariableCount(); vi++)
        solution.push_back(
          state.at(joint_model->getFirstVariableIndex() + vi));
    }
  }

  // set solution fitness
  if (bio_ik_options) {
    bio_ik_options->solution_fitness = ik->getSolutionFitness();
  }

  // return an error if an accurate solution was requested, but no accurate
  // solution was found
  if (!ik->getSuccess() && !options.return_approximate_solution) {
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  // callback?
  if (!solution_callback.empty()) {
    // run callback
    solution_callback(ik_poses.front(), solution, error_code);

    // return success if callback has accepted the solution
    return error_code.val == error_code.SUCCESS;
  } else {
    // return success
    error_code.val = error_code.SUCCESS;
    return true;
  }
}
}

PLUGINLIB_EXPORT_CLASS(bio_ik_kinematics_plugin::BioIKKinematicsPlugin,
  kinematics::KinematicsBase);