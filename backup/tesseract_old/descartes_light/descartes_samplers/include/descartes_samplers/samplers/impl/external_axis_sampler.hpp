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
#ifndef DESCARTES_SAMPLERS_SAMPLERS_IMPL_EXTERNAL_AXIS_SAMPLER_HPP
#define DESCARTES_SAMPLERS_SAMPLERS_IMPL_EXTERNAL_AXIS_SAMPLER_HPP

#include <descartes_samplers/samplers/external_axis_sampler.h>

namespace descartes_light
{
template <typename FloatType>
ExternalAxisSampler<FloatType>::ExternalAxisSampler(
    const Eigen::Transform<FloatType, 3, Eigen::Isometry>& tool_in_positioner,
    const typename KinematicsInterface<FloatType>::Ptr robot_kin,
    const typename CollisionInterface<FloatType>::Ptr collision)
  : tool_pose_(tool_in_positioner), kin_(robot_kin), collision_(std::move(collision))
{
}

template <typename FloatType>
bool ExternalAxisSampler<FloatType>::isCollisionFree(const FloatType* vertex)
{
  return collision_->validate(vertex, 7);
}

template <typename FloatType>
bool ExternalAxisSampler<FloatType>::sample(std::vector<FloatType>& solution_set)
{
  // We need to translate the tool pose to the "robot" frame
  // We need some strategy for moving the positioner around to generate many of these frames
  //    - In the simple case, we can sample the positioner limits evenly but this will often
  //      lead to terrible performance

  auto to_robot_frame = [](const Eigen::Transform<FloatType, 3, Eigen::Isometry>& pose_in_positioner,
                           const FloatType positioner_angle) {
    const FloatType x = static_cast<FloatType>(1.25);
    const FloatType y = static_cast<FloatType>(0.0);
    const FloatType z = static_cast<FloatType>(0.0);
    return Eigen::Translation<FloatType, 3>(x, y, z) *
           Eigen::AngleAxis<FloatType>(positioner_angle, Eigen::Matrix<FloatType, 3, 1>::UnitZ()) * pose_in_positioner;
  };

  // So we just loop
  const static FloatType discretization = static_cast<FloatType>(M_PI / 36.0);
  for (FloatType angle = static_cast<FloatType>(-1.0 * M_PI); angle <= static_cast<FloatType>(M_PI);
       angle += discretization)
  {
    std::vector<FloatType> buffer;
    kin_->ik(to_robot_frame(tool_pose_, angle), buffer);

    // Now test the solutions
    const auto n_sols = buffer.size() / 6;
    for (std::size_t i = 0; i < n_sols; ++i)
    {
      const auto* sol_data = buffer.data() + i * 6;
      if (isCollisionFree(sol_data))
      {
        solution_set.insert(end(solution_set), sol_data, sol_data + 6);
        solution_set.insert(end(solution_set), angle);
      }
    }
  }

  return !solution_set.empty();
}

template <typename FloatType>
SpoolSampler<FloatType>::SpoolSampler(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& tool_in_positioner,
                                      const typename KinematicsInterface<FloatType>::Ptr robot_kin,
                                      const typename CollisionInterface<FloatType>::Ptr collision)
  : tool_pose_(tool_in_positioner), kin_(robot_kin), collision_(std::move(collision))
{
}

template <typename FloatType>
bool SpoolSampler<FloatType>::isCollisionFree(const FloatType* vertex)
{
  return collision_->validate(vertex, 7);
}

template <typename FloatType>
bool SpoolSampler<FloatType>::sample(std::vector<FloatType>& solution_set)
{
  // We need to translate the tool pose to the "robot" frame
  // We need some strategy for moving the positioner around to generate many of these frames
  //    - In the simple case, we can sample the positioner limits evenly but this will often
  //      lead to terrible performance

  auto to_robot_frame = [](const Eigen::Transform<FloatType, 3, Eigen::Isometry>& pose_in_positioner,
                           const FloatType positioner_angle) {
    const FloatType x = static_cast<FloatType>(1.25);
    const FloatType y = static_cast<FloatType>(0.0);
    const FloatType z = static_cast<FloatType>(0.0);
    return Eigen::Translation<FloatType, 3>(x, y, z) *
           Eigen::AngleAxis<FloatType>(static_cast<FloatType>(M_PI / 2.0), Eigen::Matrix<FloatType, 3, 1>::UnitX()) *
           Eigen::AngleAxis<FloatType>(positioner_angle, Eigen::Matrix<FloatType, 3, 1>::UnitZ()) * pose_in_positioner;
  };

  // So we just loop
  const static FloatType discretization = static_cast<FloatType>(M_PI / 36.0);
  for (FloatType angle = static_cast<FloatType>(-2.0 * M_PI); angle <= static_cast<FloatType>(2.0 * M_PI);
       angle += discretization)
  {
    std::vector<FloatType> buffer;
    kin_->ik(to_robot_frame(tool_pose_, angle), buffer);

    // Now test the solutions
    const auto n_sols = buffer.size() / 6;
    for (std::size_t i = 0; i < n_sols; ++i)
    {
      const auto* sol_data = buffer.data() + i * 6;
      if (SpoolSampler<FloatType>::isCollisionFree(sol_data))
      {
        solution_set.insert(end(solution_set), sol_data, sol_data + 6);
        solution_set.insert(end(solution_set), angle);
      }
    }
  }

  return !solution_set.empty();
}

}  // namespace descartes_light

#endif  // DESCARTES_SAMPLERS_SAMPLERS_IMPL_EXTERNAL_AXIS_SAMPLER_HPP
