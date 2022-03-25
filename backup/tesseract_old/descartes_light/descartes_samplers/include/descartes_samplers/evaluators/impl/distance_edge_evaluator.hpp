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
#ifndef DESCARTES_SAMPLERS_EVALUATORS_IMPL_DISTANCE_EDGE_EVALUATOR_HPP
#define DESCARTES_SAMPLERS_EVALUATORS_IMPL_DISTANCE_EDGE_EVALUATOR_HPP

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <cmath>
DESCARTES_IGNORE_WARNINGS_POP

#include <descartes_samplers/evaluators/distance_edge_evaluator.h>

namespace
{
template <typename FloatType>
static void considerEdge(const FloatType* start,
                         const FloatType* end,
                         const std::vector<FloatType>& delta_thresholds,
                         std::size_t next_idx,
                         typename descartes_light::LadderGraph<FloatType>::EdgeList& out)
{
  FloatType cost = static_cast<FloatType>(0.0);
  for (std::size_t i = 0; i < delta_thresholds.size(); ++i)
  {
    FloatType step = end[i] - start[i];
    if (std::abs(step) > delta_thresholds[i])
      return;

    cost += std::pow(step, FloatType(2));
  }

  out.emplace_back(cost, next_idx);
}

}  // namespace

namespace descartes_light
{
template <typename FloatType>
DistanceEdgeEvaluator<FloatType>::DistanceEdgeEvaluator(const std::vector<FloatType>& velocity_limits)
  : velocity_limits_(velocity_limits)
{
}

template <typename FloatType>
bool DistanceEdgeEvaluator<FloatType>::evaluate(const Rung_<FloatType>& from,
                                                const Rung_<FloatType>& to,
                                                std::vector<typename LadderGraph<FloatType>::EdgeList>& edges)
{
  const auto dof = velocity_limits_.size();
  const auto n_start = from.data.size() / dof;
  const auto n_end = to.data.size() / dof;

  // Compute thresholds
  const auto dt = to.timing;

  std::vector<FloatType> delta_thresholds(velocity_limits_.size());
  std::transform(velocity_limits_.begin(),
                 velocity_limits_.end(),
                 delta_thresholds.begin(),
                 [dt](const FloatType& vel_limit) -> FloatType {
                   if (dt.upper != static_cast<FloatType>(0.0))
                   {
                     const static FloatType safety_factor = static_cast<FloatType>(0.9);
                     return dt.upper * vel_limit * safety_factor;
                   }
                   else
                   {
                     return std::numeric_limits<FloatType>::max();
                   }
                 });

  // Allocate
  edges.resize(n_start);

  for (std::size_t i = 0; i < n_start; ++i)
  {
    const auto* start_vertex = from.data.data() + dof * i;
    for (std::size_t j = 0; j < n_end; ++j)
    {
      const auto* end_vertex = to.data.data() + dof * j;

      // Consider the edge:
      considerEdge(start_vertex, end_vertex, delta_thresholds, j, edges[i]);
    }
  }

  for (const auto& rung : edges)
    if (!rung.empty())
      return true;

  return false;
}

}  // namespace descartes_light

#endif  // DESCARTES_SAMPLERS_EVALUATORS_IMPL_DISTANCE_EDGE_EVALUATOR_HPP
