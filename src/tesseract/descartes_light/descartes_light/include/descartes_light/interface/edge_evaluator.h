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
#ifndef DESCARTES_LIGHT_CORE_EDGE_EVALUATOR_H
#define DESCARTES_LIGHT_CORE_EDGE_EVALUATOR_H

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <memory>
#include <vector>
DESCARTES_IGNORE_WARNINGS_POP

#include "descartes_light/ladder_graph.h"

namespace descartes_light
{
template <typename FloatType>
class EdgeEvaluator
{
public:
  virtual ~EdgeEvaluator() {}

  virtual bool evaluate(const Rung_<FloatType>& from,
                        const Rung_<FloatType>& to,
                        std::vector<typename LadderGraph<FloatType>::EdgeList>& edges) = 0;

  typedef typename std::shared_ptr<EdgeEvaluator<FloatType>> Ptr;
};

using EdgeEvaluatorF = EdgeEvaluator<float>;
using EdgeEvaluatorD = EdgeEvaluator<double>;

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_CORE_EDGE_EVALUATOR_H
