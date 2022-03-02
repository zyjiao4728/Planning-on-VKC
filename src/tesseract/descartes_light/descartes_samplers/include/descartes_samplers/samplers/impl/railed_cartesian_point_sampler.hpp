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
#ifndef DESCARTES_SAMPLERS_SAMPLERS_RAILED_CARTESIAN_POINT_SAMPLER_HPP
#define DESCARTES_SAMPLERS_SAMPLERS_RAILED_CARTESIAN_POINT_SAMPLER_HPP

#include <descartes_samplers/samplers/railed_cartesian_point_sampler.h>

const static std::size_t dof = 8;

namespace descartes_light
{
template <typename FloatType>
RailedCartesianPointSampler<FloatType>::RailedCartesianPointSampler(
    const Eigen::Transform<FloatType, 3, Eigen::Isometry>& tool_pose,
    const typename KinematicsInterface<FloatType>::Ptr robot_kin,
    const typename CollisionInterface<FloatType>::Ptr collision,
    const bool allow_collision)
  : tool_pose_(tool_pose), kin_(robot_kin), collision_(std::move(collision)), allow_collision_(allow_collision)
{
}

template <typename FloatType>
bool RailedCartesianPointSampler<FloatType>::sample(std::vector<FloatType>& solution_set)
{
  std::vector<FloatType> buffer;
  kin_->ik(tool_pose_, buffer);

  const auto nSamplesInBuffer = [](const std::vector<FloatType>& v) -> std::size_t { return v.size() / dof; };

  const auto n_sols = nSamplesInBuffer(buffer);

  for (std::size_t i = 0; i < n_sols; ++i)
  {
    const auto* sol_data = buffer.data() + i * dof;
    if (isCollisionFree(sol_data))
      solution_set.insert(end(solution_set), sol_data, sol_data + dof);
  }

  if (solution_set.empty() && allow_collision_)
    getBestSolution(solution_set);

  return !solution_set.empty();
}

template <typename FloatType>
bool RailedCartesianPointSampler<FloatType>::isCollisionFree(const FloatType* vertex)
{
  if (collision_ == nullptr)
    return true;
  else
    return collision_->validate(vertex, dof);
}

template <typename FloatType>
bool RailedCartesianPointSampler<FloatType>::getBestSolution(std::vector<FloatType>& solution_set)
{
  FloatType distance = -std::numeric_limits<FloatType>::max();
  std::vector<FloatType> buffer;
  kin_->ik(tool_pose_, buffer);

  const auto nSamplesInBuffer = [](const std::vector<FloatType>& v) -> std::size_t { return v.size() / dof; };

  const auto n_sols = nSamplesInBuffer(buffer);

  for (std::size_t i = 0; i < n_sols; ++i)
  {
    const auto* sol_data = buffer.data() + i * dof;
    FloatType cur_distance = collision_->distance(sol_data, dof);
    if (cur_distance > distance)
    {
      distance = cur_distance;
      solution_set.clear();
      solution_set.insert(end(solution_set), sol_data, sol_data + dof);
    }
  }

  return !solution_set.empty();
}

}  // namespace descartes_light

#endif  // DESCARTES_SAMPLERS_SAMPLERS_RAILED_CARTESIAN_POINT_SAMPLER_HPP
