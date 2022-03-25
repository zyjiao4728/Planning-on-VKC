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
#ifndef DESCARTES_LIGHT_GANTRY_KINEMATICS_H
#define DESCARTES_LIGHT_GANTRY_KINEMATICS_H

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <Eigen/Dense>
#include <vector>
DESCARTES_IGNORE_WARNINGS_POP

#include <descartes_light/visibility_control.h>
#include <descartes_light/interface/kinematics_interface.h>
#include <descartes_light/utils.h>

namespace descartes_light
{
/**
 * @brief This takes a kinematics object that represents the robot and adds gantry sampling
 *
 * This class is intended to be used for a robots mounted on two-axis gantries, where the
 * gantry axes are assumed to be orthogonal and along the X and Y axes.
 *
 */
template <typename FloatType>
class GantryKinematics : public KinematicsInterface<FloatType>
{
public:
  /**
   * @brief This takes a kinematics object that represents the robot and adds gantry sampling
   *
   * When provided a point in world coordinate system this will find the gantry {X, Y} values
   * that center the robots base coordinate system directly over the point. It then searches
   * from {X - Xres, Y - Yres} to {X + Xres, Y + Yres} and provides all solutions.
   *
   * @param robot_kinematics The kinematic object atached to a two-axis gantry
   * @param world_to_rail_base The transformation from the world coordinate system to the origin of the two-axis gantry
   * @param rail_base_to_robot_base The transformation from the two-axis gantry origin to the robot's base coordinate
   * system
   * @param rail_limits The rails limit {Xmin, Xmax; Ymin, Ymax}
   * @param rail_sample_resolution The resolution at which to sample the gantry {Xres, Yres}
   * @param robot_reach This defines how far to search {X, Y} gantry around a location.
   */
  GantryKinematics(const typename KinematicsInterface<FloatType>::ConstPtr robot_kinematics,
                   const Eigen::Transform<FloatType, 3, Eigen::Isometry>& world_to_rail_base,
                   const Eigen::Transform<FloatType, 3, Eigen::Isometry>& rail_base_to_robot_base,
                   const Eigen::Matrix<FloatType, 2, 2>& rail_limits,
                   const Eigen::Matrix<FloatType, 2, 1>& rail_sample_resolution,
                   const FloatType robot_reach);

  bool ik(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
          std::vector<FloatType>& solution_set) const override;

  bool fk(const FloatType* pose, Eigen::Transform<FloatType, 3, Eigen::Isometry>& solution) const override;

  bool ikAt(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
            const Eigen::Matrix<FloatType, 2, 1>& rail_pose,
            std::vector<FloatType>& solution_set) const;

  bool fkAt(const Eigen::Matrix<FloatType, 2, 1>& rail_pose,
            const std::vector<FloatType>& pose,
            Eigen::Transform<FloatType, 3, Eigen::Isometry>& solution) const;

  int dof() const override;

  void analyzeIK(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p) const override;

private:
  typename KinematicsInterface<FloatType>::ConstPtr robot_kinematics_;
  Eigen::Transform<FloatType, 3, Eigen::Isometry> world_to_rail_base_;
  Eigen::Transform<FloatType, 3, Eigen::Isometry> rail_base_to_robot_base_;
  Eigen::Matrix<FloatType, 2, 2> rail_limits_;
  Eigen::Matrix<FloatType, 2, 1> rail_sample_resolution_;
  FloatType robot_reach_;

  Eigen::Matrix<FloatType, 2, 1> getRange(const FloatType val, const FloatType min_val, const FloatType max_val) const;
};

using GantryKinematicsD = GantryKinematics<double>;
using GantryKinematicsF = GantryKinematics<float>;
}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_GANTRY_KINEMATICS_H
