/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef DESCARTES_LIGHT_GANTRY_KINEMATICS_HPP
#define DESCARTES_LIGHT_GANTRY_KINEMATICS_HPP

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
DESCARTES_IGNORE_WARNINGS_POP

#include <descartes_light/impl/gantry_kinematics.h>
#include <descartes_light/utils.h>

namespace descartes_light
{
template <typename FloatType>
GantryKinematics<FloatType>::GantryKinematics(
    const typename KinematicsInterface<FloatType>::ConstPtr robot_kinematics,
    const Eigen::Transform<FloatType, 3, Eigen::Isometry>& world_to_rail_base,
    const Eigen::Transform<FloatType, 3, Eigen::Isometry>& rail_base_to_robot_base,
    const Eigen::Matrix<FloatType, 2, 2>& rail_limits,
    const Eigen::Matrix<FloatType, 2, 1>& rail_sample_resolution,
    const FloatType robot_reach)
  : robot_kinematics_(std::move(robot_kinematics))
  , world_to_rail_base_(world_to_rail_base)
  , rail_base_to_robot_base_(rail_base_to_robot_base)
  , rail_limits_(rail_limits)
  , rail_sample_resolution_(rail_sample_resolution)
  , robot_reach_(robot_reach)
{
}

template <typename FloatType>
bool GantryKinematics<FloatType>::ik(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
                                     std::vector<FloatType>& solution_set) const
{
  // Tool pose in rail coordinate system
  Eigen::Transform<FloatType, 3, Eigen::Isometry> tool_pose = world_to_rail_base_.inverse() * p;

  const Eigen::Matrix<FloatType, 2, 1>& rail_lower_limit = rail_limits_.col(0);
  const Eigen::Matrix<FloatType, 2, 1>& rail_upper_limit = rail_limits_.col(1);

  const Eigen::Matrix<FloatType, 2, 1> origin(tool_pose.translation().x() - rail_base_to_robot_base_.translation().x(),
                                              tool_pose.translation().y() - rail_base_to_robot_base_.translation().y());

  const Eigen::Matrix<FloatType, 2, 1> x_range = getRange(origin.x(), rail_lower_limit.x(), rail_upper_limit.x());
  const Eigen::Matrix<FloatType, 2, 1> y_range = getRange(origin.y(), rail_lower_limit.y(), rail_upper_limit.y());

  const FloatType res_x =
      (x_range[1] - x_range[0]) / std::ceil((x_range[1] - x_range[0]) / rail_sample_resolution_.x());
  const FloatType res_y =
      (y_range[1] - y_range[0]) / std::ceil((y_range[1] - y_range[0]) / rail_sample_resolution_.y());

  for (FloatType x = x_range[0]; x < x_range[1]; x += res_x)
    for (FloatType y = y_range[0]; y < y_range[1]; y += res_y)
      ikAt(p, Eigen::Matrix<FloatType, 2, 1>(x, y), solution_set);

  return !solution_set.empty();
}

template <typename FloatType>
bool GantryKinematics<FloatType>::ikAt(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
                                       const Eigen::Matrix<FloatType, 2, 1>& rail_pose,
                                       std::vector<FloatType>& solution_set) const
{
  const Eigen::Transform<FloatType, 3, Eigen::Isometry> world_to_robot_base =
      world_to_rail_base_ *
      Eigen::Translation<FloatType, 3>(rail_pose.x(), rail_pose.y(), static_cast<FloatType>(0.0)) *
      rail_base_to_robot_base_;
  const Eigen::Transform<FloatType, 3, Eigen::Isometry> in_robot = world_to_robot_base.inverse() * p;

  std::vector<FloatType> sols;
  int robot_dof = robot_kinematics_->dof();
  if (!robot_kinematics_->ik(in_robot, sols))
    return false;

  int num_sols = static_cast<int>(sols.size()) / robot_dof;
  // Check the output
  for (int i = 0; i < num_sols; i++)
  {
    FloatType* sol = sols.data() + robot_dof * i;
    solution_set.insert(end(solution_set), rail_pose.data(), rail_pose.data() + 2);  // Insert the X-Y pose of the rail
    solution_set.insert(end(solution_set), sol, sol + robot_dof);  // And then insert the robot arm configuration
  }

  return !solution_set.empty();
}

template <typename FloatType>
bool GantryKinematics<FloatType>::fkAt(const Eigen::Matrix<FloatType, 2, 1>& rail_pose,
                                       const std::vector<FloatType>& pose,
                                       Eigen::Transform<FloatType, 3, Eigen::Isometry>& solution) const
{
  if (!robot_kinematics_->fk(pose.data(), solution))
    return false;

  Eigen::Transform<FloatType, 3, Eigen::Isometry> rail_tf = Eigen::Transform<FloatType, 3, Eigen::Isometry>::Identity();
  rail_tf.translation().head(2) = rail_pose;
  solution = world_to_rail_base_ * rail_tf * rail_base_to_robot_base_ * solution;
  return true;
}

template <typename FloatType>
bool GantryKinematics<FloatType>::fk(const FloatType* pose,
                                     Eigen::Transform<FloatType, 3, Eigen::Isometry>& solution) const
{
  Eigen::Matrix<FloatType, 2, 1> rail_pose;
  rail_pose(0) = pose[0];
  rail_pose(1) = pose[1];

  std::vector<FloatType> robot_pose(pose + 2, pose + dof());

  return fkAt(rail_pose, robot_pose, solution);
}

template <typename FloatType>
int GantryKinematics<FloatType>::dof() const
{
  return robot_kinematics_->dof() + 2;
}

template <typename FloatType>
void GantryKinematics<FloatType>::analyzeIK(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p) const
{
  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "AnalyzeIK: ", ";");
  std::stringstream ss;
  ss << p.matrix().format(CommaInitFmt);
  CONSOLE_BRIDGE_logInform(ss.str().c_str());

  // Tool pose in rail coordinate system
  Eigen::Transform<FloatType, 3, Eigen::Isometry> tool_pose = world_to_rail_base_.inverse() * p;

  const Eigen::Matrix<FloatType, 2, 1>& rail_lower_limit = rail_limits_.col(0);
  const Eigen::Matrix<FloatType, 2, 1>& rail_upper_limit = rail_limits_.col(1);

  const Eigen::Matrix<FloatType, 2, 1> origin(tool_pose.translation().x() - rail_base_to_robot_base_.translation().x(),
                                              tool_pose.translation().y() - rail_base_to_robot_base_.translation().y());

  const Eigen::Matrix<FloatType, 2, 1> x_range = getRange(origin.x(), rail_lower_limit.x(), rail_upper_limit.x());
  const Eigen::Matrix<FloatType, 2, 1> y_range = getRange(origin.y(), rail_lower_limit.y(), rail_upper_limit.y());

  const FloatType res_x =
      (x_range[1] - x_range[0]) / std::ceil((x_range[1] - x_range[0]) / rail_sample_resolution_.x());
  const FloatType res_y =
      (y_range[1] - y_range[0]) / std::ceil((y_range[1] - y_range[0]) / rail_sample_resolution_.y());

  for (FloatType x = x_range[0]; x < x_range[1]; x += res_x)
  {
    for (FloatType y = y_range[0]; y < y_range[1]; y += res_y)
    {
      const Eigen::Transform<FloatType, 3, Eigen::Isometry> world_to_robot_base =
          world_to_rail_base_ * Eigen::Translation<FloatType, 3>(x, y, static_cast<FloatType>(0.0)) *
          rail_base_to_robot_base_;
      const Eigen::Transform<FloatType, 3, Eigen::Isometry> in_robot = world_to_robot_base.inverse() * p;

      robot_kinematics_->analyzeIK(in_robot);
    }
  }
}

template <typename FloatType>
Eigen::Matrix<FloatType, 2, 1> GantryKinematics<FloatType>::getRange(const FloatType val,
                                                                     const FloatType min_val,
                                                                     const FloatType max_val) const
{
  Eigen::Matrix<FloatType, 2, 1> rng;
  if (val - robot_reach_ < min_val)
    rng[0] = min_val;
  else if (val - robot_reach_ > max_val)
    rng[0] = (max_val - robot_reach_ < min_val) ? min_val : max_val - robot_reach_;
  else
    rng[0] = val - robot_reach_;

  if (val + robot_reach_ > max_val)
    rng[1] = max_val;
  else if (val + robot_reach_ < min_val)
    rng[1] = (min_val + robot_reach_ > max_val) ? max_val : min_val + robot_reach_;
  else
    rng[1] = val + robot_reach_;

  return rng;
}

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_GANTRY_KINEMATICS_HPP
