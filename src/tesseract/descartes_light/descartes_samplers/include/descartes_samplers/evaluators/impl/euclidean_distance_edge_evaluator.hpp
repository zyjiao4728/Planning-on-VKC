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
#ifndef DESCARTES_SAMPLERS_EVALUATORS_EUCLIDEAN_DISTANCE_EDGE_EVALUATOR_HPP
#define DESCARTES_SAMPLERS_EVALUATORS_EUCLIDEAN_DISTANCE_EDGE_EVALUATOR_HPP

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <cmath>
DESCARTES_IGNORE_WARNINGS_POP

#include <descartes_samplers/evaluators/euclidean_distance_edge_evaluator.h>

namespace
{
template <typename FloatType>
static void considerEdge(const FloatType* start,
                         const FloatType* end,
                         std::size_t dof,
                         std::size_t next_idx,
                         typename descartes_light::LadderGraph<FloatType>::EdgeList& out)
{
  FloatType cost = 0.0;
  for (std::size_t i = 0; i < dof; ++i)
  {
    cost += std::pow(end[i] - start[i], FloatType(2));
  }

  out.emplace_back(cost, next_idx);
}

}  // namespace

namespace descartes_light
{
template <typename FloatType>
EuclideanDistanceEdgeEvaluator<FloatType>::EuclideanDistanceEdgeEvaluator(int dof) : dof_(static_cast<std::size_t>(dof))
{
}

template <typename FloatType>
bool EuclideanDistanceEdgeEvaluator<FloatType>::evaluate(const Rung_<FloatType>& from,
                                                         const Rung_<FloatType>& to,
                                                         std::vector<typename LadderGraph<FloatType>::EdgeList>& edges)
{
  const auto n_start = from.data.size() / dof_;
  const auto n_end = to.data.size() / dof_;

  // Allocate
  edges.resize(n_start);

  for (std::size_t i = 0; i < n_start; ++i)
  {
    const auto* start_vertex = from.data.data() + dof_ * i;
    for (std::size_t j = 0; j < n_end; ++j)
    {
      const auto* end_vertex = to.data.data() + dof_ * j;

      // Consider the edge:
      considerEdge(start_vertex, end_vertex, dof_, j, edges[i]);
    }
  }

  for (const auto& rung : edges)
    if (!rung.empty())
      return true;

  return false;
}

}  // namespace descartes_light

#endif  // DESCARTES_SAMPLERS_EVALUATORS_EUCLIDEAN_DISTANCE_EDGE_EVALUATOR_HPP
