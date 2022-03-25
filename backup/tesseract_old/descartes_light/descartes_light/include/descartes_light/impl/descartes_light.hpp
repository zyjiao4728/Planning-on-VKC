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
#ifndef DESCARTES_LIGHT_IMPL_DESCARTES_LIGHT_HPP
#define DESCARTES_LIGHT_IMPL_DESCARTES_LIGHT_HPP

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <sstream>
#include <algorithm>
DESCARTES_IGNORE_WARNINGS_POP

#include <descartes_light/descartes_light.h>
#include <descartes_light/ladder_graph_dag_search.h>

#define UNUSED(x) (void)(x)

static void reportFailedEdges(const std::vector<std::size_t>& indices)
{
  if (indices.empty())
    CONSOLE_BRIDGE_logInform("No failed edges");
  else
  {
    std::stringstream ss;
    ss << "Failed edges:\n";
    for (const auto& i : indices)
      ss << "\t" << i << "\n";

    CONSOLE_BRIDGE_logWarn(ss.str().c_str());
  }
}

static void reportFailedVertices(const std::vector<std::size_t>& indices)
{
  if (indices.empty())
    CONSOLE_BRIDGE_logInform("No failed vertices");
  else
  {
    std::stringstream ss;
    ss << "Failed vertices:\n";
    for (const auto& i : indices)
      ss << "\t" << i << "\n";

    CONSOLE_BRIDGE_logWarn(ss.str().c_str());
  }
}

namespace descartes_light
{
template <typename FloatType>
Solver<FloatType>::Solver(const std::size_t dof) : graph_{ dof }
{
}

template <typename FloatType>
bool Solver<FloatType>::build(const std::vector<typename PositionSampler<FloatType>::Ptr>& trajectory,
                              const std::vector<typename descartes_core::TimingConstraint<FloatType>>& times,
                              const std::vector<typename EdgeEvaluator<FloatType>::Ptr>& edge_eval,
                              int num_threads)
{
  graph_.resize(trajectory.size());
  failed_vertices_.clear();
  failed_edges_.clear();

  // Build Vertices
  long num_waypoints = static_cast<long>(trajectory.size());
  long cnt = 0;
#pragma omp parallel for num_threads(num_threads)
  for (long i = 0; i < static_cast<long>(trajectory.size()); ++i)
  {
    std::vector<FloatType> vertex_data;
    if (trajectory[static_cast<size_t>(i)]->sample(vertex_data))
    {
      graph_.getRung(static_cast<size_t>(i)).data = std::move(vertex_data);
      graph_.getRung(static_cast<size_t>(i)).timing = times[static_cast<size_t>(i)];
    }
    else
    {
#pragma omp critical
      {
        failed_vertices_.push_back(static_cast<size_t>(i));
      }
    }
#ifndef NDEBUG
#pragma omp critical
    {
      ++cnt;
      std::stringstream ss;
      ss << "Descartes Processed: " << cnt << " of " << num_waypoints << " vertices";
      CONSOLE_BRIDGE_logInform(ss.str().c_str());
    }
#endif
  }

  // Build Edges
  cnt = 0;
#pragma omp parallel for num_threads(num_threads)
  for (long i = 1; i < static_cast<long>(trajectory.size()); ++i)
  {
    const auto& from = graph_.getRung(static_cast<size_t>(i) - static_cast<size_t>(1));
    const auto& to = graph_.getRung(static_cast<size_t>(i));

    if (!edge_eval[static_cast<size_t>(i - 1)]->evaluate(
            from, to, graph_.getEdges(static_cast<size_t>(i) - static_cast<size_t>(1))))
    {
#pragma omp critical
      {
        failed_edges_.push_back(static_cast<size_t>(i) - static_cast<size_t>(1));
      }
    }
#ifndef NDEBUG
#pragma omp critical
    {
      ++cnt;
      std::stringstream ss;
      ss << "Descartes Processed: " << cnt << " of " << (num_waypoints - 1) << " edges";
      CONSOLE_BRIDGE_logInform(ss.str().c_str());
    }
#endif
  }
  UNUSED(cnt);
  UNUSED(num_waypoints);

  std::sort(failed_vertices_.begin(), failed_vertices_.end());
  std::sort(failed_edges_.begin(), failed_edges_.end());

  reportFailedVertices(failed_vertices_);
  reportFailedEdges(failed_edges_);

  return failed_edges_.empty() && failed_vertices_.empty();
}

template <typename FloatType>
bool Solver<FloatType>::build(const std::vector<typename PositionSampler<FloatType>::Ptr>& trajectory,
                              const std::vector<typename descartes_core::TimingConstraint<FloatType>>& times,
                              typename EdgeEvaluator<FloatType>::Ptr edge_eval,
                              int num_threads)
{
  std::vector<typename EdgeEvaluator<FloatType>::Ptr> evaluators(trajectory.size() - 1, edge_eval);
  return build(trajectory, times, evaluators, num_threads);
}

template <typename FloatType>
bool Solver<FloatType>::search(std::vector<FloatType>& solution)
{
  DAGSearch<FloatType> s(graph_);
  const auto cost = s.run();

  if (cost == std::numeric_limits<FloatType>::max())
    return false;

  const auto indices = s.shortestPath();

  for (std::size_t i = 0; i < indices.size(); ++i)
  {
    const auto* pose = graph_.vertex(i, indices[i]);
    solution.insert(end(solution), pose, pose + graph_.dof());
  }

  std::stringstream ss;
  ss << "Solution found w/ cost = " << cost;
  CONSOLE_BRIDGE_logInform(ss.str().c_str());

  return true;
}

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_IMPL_DESCARTES_LIGHT_HPP
