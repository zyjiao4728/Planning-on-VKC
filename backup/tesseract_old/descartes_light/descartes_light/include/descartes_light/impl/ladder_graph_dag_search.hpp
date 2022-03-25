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
#ifndef DESCARTES_LIGHT_IMPL_LADDER_GRAPH_DAG_SEARCH_HPP
#define DESCARTES_LIGHT_IMPL_LADDER_GRAPH_DAG_SEARCH_HPP

#include <descartes_light/ladder_graph_dag_search.h>

namespace descartes_light
{
template <typename FloatType>
DAGSearch<FloatType>::DAGSearch(const LadderGraph<FloatType>& graph) : graph_(graph)
{
  // On creating an object, let's allocate everything we need
  solution_.resize(graph.size());

  for (size_t i = 0; i < graph.size(); ++i)
  {
    const auto n_vertices = graph.rungSize(i);
    solution_[i].distance.resize(n_vertices);
    solution_[i].predecessor.resize(n_vertices);
  }
}

template <typename FloatType>
FloatType DAGSearch<FloatType>::run()
{
  // Cost to the first rung should be set to zero
  std::fill(solution_.front().distance.begin(), solution_.front().distance.end(), 0.0);

  // Other rows initialize to zero
  for (size_type i = 1; i < solution_.size(); ++i)
  {
    std::fill(solution_[i].distance.begin(), solution_[i].distance.end(), std::numeric_limits<FloatType>::max());
  }

  // Now we iterate over the graph in 'topological' order
  for (size_type rung = 0; rung < solution_.size() - 1; ++rung)
  {
    const auto n_vertices = graph_.rungSize(rung);
    const auto next_rung = rung + 1;
    // For each vertex in the out edge list
    for (size_t index = 0; index < n_vertices; ++index)
    {
      const auto u_cost = distance(rung, index);
      const auto& edges = graph_.getEdges(rung)[index];
      // for each out edge
      for (const auto& edge : edges)
      {
        auto dv = u_cost + edge.cost;  // new cost
        if (dv < distance(next_rung, edge.idx))
        {
          distance(next_rung, edge.idx) = dv;
          predecessor(next_rung, edge.idx) =
              static_cast<unsigned>(index);  // the predecessor's rung is implied to be the current rung
        }
      }
    }  // vertex for loop
  }    // rung for loop

  return *std::min_element(solution_.back().distance.begin(), solution_.back().distance.end());
}

template <typename FloatType>
std::vector<typename DAGSearch<FloatType>::predecessor_t> DAGSearch<FloatType>::shortestPath() const
{
  auto min_it = std::min_element(solution_.back().distance.begin(), solution_.back().distance.end());
  auto min_idx = std::distance(solution_.back().distance.begin(), min_it);
  assert(min_idx >= 0);

  std::vector<predecessor_t> path(solution_.size());

  size_type current_rung = path.size() - 1;
  size_type current_index = static_cast<size_type>(min_idx);

  for (unsigned i = 0; i < path.size(); ++i)
  {
    auto count = path.size() - 1 - i;
    assert(current_rung == count);
    path[count] = static_cast<unsigned>(current_index);
    current_index = predecessor(current_rung, current_index);
    current_rung -= 1;
  }

  return path;
}

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_IMPL_LADDER_GRAPH_DAG_SEARCH_HPP
