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
#ifndef DESCARTES_LIGHT_CORE_COLLISION_INTERFACE_H
#define DESCARTES_LIGHT_CORE_COLLISION_INTERFACE_H

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <memory>
DESCARTES_IGNORE_WARNINGS_POP

#include <descartes_light/visibility_control.h>

namespace descartes_light
{
template <typename FloatType>
class CollisionInterface
{
public:
  virtual ~CollisionInterface() = default;

  virtual bool validate(const FloatType* pos, const std::size_t size) = 0;

  virtual FloatType distance(const FloatType* pos, const std::size_t size) = 0;

  /** You assume ownership of return value */
  virtual std::shared_ptr<CollisionInterface> clone() const = 0;

  typedef typename std::shared_ptr<CollisionInterface> Ptr;
  typedef typename std::shared_ptr<const CollisionInterface> ConstPtr;
};

using CollisionInterfaceF = CollisionInterface<float>;
using CollisionInterfaceD = CollisionInterface<double>;

}  // namespace descartes_light

#endif  // DESCARTES_LIGHT_CORE_COLLISION_INTERFACE_H
