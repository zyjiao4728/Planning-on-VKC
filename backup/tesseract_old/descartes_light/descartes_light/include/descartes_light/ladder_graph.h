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
#ifndef DESCARTES_LIGHT_LADDER_GRAPH_H
#define DESCARTES_LIGHT_LADDER_GRAPH_H

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <algorithm>
#include <cassert>
#include <vector>
DESCARTES_IGNORE_WARNINGS_POP

#include <descartes_light/visibility_control.h>

namespace descartes_core
{
using TrajectoryID = std::size_t;

template <typename FloatType>
struct TimingConstraint
{
  TimingConstraint() : upper(0.0) {}
  TimingConstraint(FloatType time) : upper(time) {}
  FloatType upper;
};

using TimingConstraintF = TimingConstraint<float>;
using TimingConstraintD = TimingConstraint<double>;

}  // namespace descartes_core

namespace descartes_light
{
template <typename FloatT>
struct Edge_
{
  Edge_() noexcept = default;
  Edge_(FloatT cost, unsigned index) noexcept : cost{ cost }, idx{ index } {}
  FloatT cost;  /** @brief transition cost from vertex who owns this object to 'idx' in next rung */
  unsigned idx; /** @brief from THIS rung to 'idx' into the NEXT rung */
};

template <typename FloatType>
struct Rung_
{
  using Edge = Edge_<FloatType>;
  using EdgeList = std::vector<Edge>;

  descartes_core::TrajectoryID id;                     // corresponds to user's input ID
  descartes_core::TimingConstraint<FloatType> timing;  // user input timing
  std::vector<FloatType> data;                         // joint values stored in one contiguous array
  std::vector<EdgeList> edges;
};

/**
 * @brief LadderGraph is an adjacency list based, directed graph structure with vertices
 *        arranged into "rungs" which have connections only to vertices in the adjacent
 *        rungs. Assumes a fixed DOF.
 */
template <typename FloatType>
class LadderGraph
{
public:
  using Rung = Rung_<FloatType>;
  using EdgeList = typename Rung::EdgeList;

  /**
   * @brief LadderGraph
   * @param dof The number of joints that constitute a single 'DOF'
   */
  explicit LadderGraph(const std::size_t dof) noexcept;

  /**
   * @brief resize Resizes the internal ladder to have 'n_rung' rungs
   * @param n_rungs Number of individual rungs
   */
  void resize(const std::size_t n_rungs);

  Rung& getRung(const std::size_t index) noexcept;
  const Rung& getRung(const std::size_t index) const noexcept;

  std::vector<EdgeList>& getEdges(const std::size_t index) noexcept;  // see p.23 Effective C++ (Scott Meyers)
  const std::vector<EdgeList>& getEdges(const std::size_t index) const noexcept;

  std::size_t rungSize(const std::size_t index) const noexcept;

  /**
   * @brief numVertices Counts the total number of vertices in the graph
   */
  std::size_t numVertices() const noexcept;

  /**
   * @brief indexOf returns a pair describing whether the given ID is in the graph and if so, what
   *        index it has.
   * @param id The ID to
   * @return std::pair(index, was_found)
   */
  std::pair<std::size_t, bool> indexOf(descartes_core::TrajectoryID id) const noexcept;

  /**
   * @brief isLast tests to see if a given index is the last one in the graph
   */
  bool isLast(const std::size_t index) const noexcept;

  /**
   * @brief isFirst tests to see if given index is the first in the graph
   */
  bool isFirst(const std::size_t index) const noexcept;

  /**
   * @brief vertex returns a pointer to the data that constitutes the Nth vertex in the Jth row
   *        where N = index & J = rung
   */
  const FloatType* vertex(const std::size_t rung, const std::size_t index) const;

  /**
   * @brief The number of rungs
   * @return
   */
  std::size_t size() const noexcept;

  /**
   * @brief The number of degrees of freedom
   * @return
   */
  std::size_t dof() const noexcept;

  /**
   * @brief assign Consumes the given edge list and assigns it to the rung-index given by 'rung'
   */
  void assignEdges(const std::size_t rung, std::vector<EdgeList>&& edges);  // noexcept?

  /**
   * @brief assignRung Special helper function to assign a solution set associated with a Descartes point &
   *        it's meta-info. Also resizes the associated edge list to the size of 'sols'.
   * @param sols All of the joint solutions for this point.
   */
  void assignRung(const std::size_t index,
                  descartes_core::TrajectoryID id,
                  descartes_core::TimingConstraint<FloatType> time,
                  const std::vector<std::vector<FloatType>>& sols);

  void removeRung(const std::size_t index);

  void clearVertices(const std::size_t index);

  void clearEdges(const std::size_t index);

  /**
   * @brief insertRung Adds a new rung at the 'index'-th position. E.g., insertRung(0) will add a new
   *        rung to the beginning of the graph and the previous 0th index is now at 1.
   */
  void insertRung(const std::size_t index);

  /**
   * @brief Clears all existing rungs & associated data
   */
  void clear();

private:
  const std::size_t dof_;
  std::vector<Rung> rungs_;
};

using LadderGraphF = LadderGraph<float>;
using LadderGraphD = LadderGraph<double>;

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_LADDER_GRAPH_H
