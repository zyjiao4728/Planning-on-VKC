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
#ifndef DESCARTES_IKFAST_IKFAST_KINEMATICS_H
#define DESCARTES_IKFAST_IKFAST_KINEMATICS_H

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
template <typename FloatType>
class IKFastKinematics : public KinematicsInterface<FloatType>
{
public:
  IKFastKinematics(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& world_to_robot_base =
                       Eigen::Transform<FloatType, 3, Eigen::Isometry>::Identity(),
                   const Eigen::Transform<FloatType, 3, Eigen::Isometry>& tool0_to_tip =
                       Eigen::Transform<FloatType, 3, Eigen::Isometry>::Identity(),
                   const IsValidFn<FloatType>& is_valid_fn = nullptr,
                   const GetRedundantSolutionsFn<FloatType>& redundant_sol_fn = nullptr);

  bool ik(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
          std::vector<FloatType>& solution_set) const override;

  bool fk(const FloatType* pose, Eigen::Transform<FloatType, 3, Eigen::Isometry>& solution) const override;

  int dof() const override;

  void analyzeIK(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p) const override;

protected:
  Eigen::Transform<FloatType, 3, Eigen::Isometry> world_to_robot_base_;
  Eigen::Transform<FloatType, 3, Eigen::Isometry> tool0_to_tip_;
  IsValidFn<FloatType> is_valid_fn_;
  GetRedundantSolutionsFn<FloatType> redundant_sol_fn_;

  bool ik(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
          const IsValidFn<FloatType>& is_valid_fn,
          const GetRedundantSolutionsFn<FloatType>& redundant_sol_fn,
          std::vector<FloatType>& solution_set) const;
};

using IKFastKinematicsD = IKFastKinematics<double>;
using IKFastKinematicsF = IKFastKinematics<float>;

}  // namespace descartes_light
#endif  // DESCARTES_IKFAST_IKFAST_KINEMATICS_H
