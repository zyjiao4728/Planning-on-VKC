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
#ifndef DESCARTES_SAMPLERS_SAMPLERS_CARTESIAN_POINT_SAMPLER_H
#define DESCARTES_SAMPLERS_SAMPLERS_CARTESIAN_POINT_SAMPLER_H

#include <descartes_light/visibility_control.h>
#include <descartes_light/interface/kinematics_interface.h>
#include <descartes_light/interface/collision_interface.h>
#include <descartes_light/interface/position_sampler.h>
#include <descartes_light/utils.h>

namespace descartes_light
{
template <typename FloatType>
class CartesianPointSampler : public PositionSampler<FloatType>
{
public:
  CartesianPointSampler(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& tool_pose,
                        const typename KinematicsInterface<FloatType>::Ptr robot_kin,
                        const typename CollisionInterface<FloatType>::Ptr collision,
                        const bool allow_collision);

  bool sample(std::vector<FloatType>& solution_set) override;

private:
  bool isCollisionFree(const FloatType* vertex);
  bool getBestSolution(std::vector<FloatType>& solution_set);

  Eigen::Transform<FloatType, 3, Eigen::Isometry> tool_pose_;
  typename KinematicsInterface<FloatType>::Ptr kin_;
  typename CollisionInterface<FloatType>::Ptr collision_;
  bool allow_collision_;
};

using CartesianPointSamplerF = CartesianPointSampler<float>;
using CartesianPointSamplerD = CartesianPointSampler<double>;

}  // namespace descartes_light

#endif  // DESCARTES_SAMPLERS_SAMPLERS_CARTESIAN_POINT_SAMPLER_H
