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
#ifndef DESCARTES_LIGHT_CORE_POSITION_SAMPLER_H
#define DESCARTES_LIGHT_CORE_POSITION_SAMPLER_H

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <memory>
#include <vector>
DESCARTES_IGNORE_WARNINGS_POP

#include <descartes_light/visibility_control.h>

namespace descartes_light
{
template <typename FloatType>
class PositionSampler
{
public:
  virtual ~PositionSampler() {}

  virtual bool sample(std::vector<FloatType>& solution_set) = 0;

  typedef typename std::shared_ptr<PositionSampler<FloatType>> Ptr;
};

using PositionSamplerF = PositionSampler<float>;
using PositionSamplerD = PositionSampler<double>;

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_CORE_POSITION_SAMPLER_H
