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
#ifndef DESCARTES_OPW_IMPL_DESCARTES_OPW_KINEMATICS_HPP
#define DESCARTES_OPW_IMPL_DESCARTES_OPW_KINEMATICS_HPP

//#include "descartes_opw/impl/descartes_opw_half.hpp"
#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <opw_kinematics/opw_utilities.h>
#include <console_bridge/console.h>
#include <array>
DESCARTES_IGNORE_WARNINGS_POP

#include <descartes_opw/descartes_opw_kinematics.h>

namespace descartes_light
{
template <typename FloatType>
OPWKinematics<FloatType>::OPWKinematics(const opw_kinematics::Parameters<FloatType>& params,
                                        const Eigen::Transform<FloatType, 3, Eigen::Isometry>& world_to_base,
                                        const Eigen::Transform<FloatType, 3, Eigen::Isometry>& tool0_to_tip,
                                        const IsValidFn<FloatType>& is_valid_fn,
                                        const GetRedundantSolutionsFn<FloatType>& redundant_sol_fn)
  : params_(params)
  , world_to_base_(world_to_base)
  , tool0_to_tip_(tool0_to_tip)
  , is_valid_fn_(is_valid_fn)
  , redundant_sol_fn_(redundant_sol_fn)
{
}

template <typename FloatType>
bool OPWKinematics<FloatType>::ik(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
                                  std::vector<FloatType>& solution_set) const
{
  return ik(p, is_valid_fn_, redundant_sol_fn_, solution_set);
}

template <typename FloatType>
bool OPWKinematics<FloatType>::ik(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
                                  const IsValidFn<FloatType>& is_valid_fn,
                                  const GetRedundantSolutionsFn<FloatType>& redundant_sol_fn,
                                  std::vector<FloatType>& solution_set) const
{
  Eigen::Transform<FloatType, 3, Eigen::Isometry> tool_pose = world_to_base_.inverse() * p * tool0_to_tip_.inverse();

  std::array<FloatType, 6 * 8> sols;
  opw_kinematics::inverse(params_, tool_pose, sols.data());

  // Check the output
  for (int i = 0; i < 8; i++)
  {
    FloatType* sol = sols.data() + 6 * i;
    if (opw_kinematics::isValid(sol))
    {
      opw_kinematics::harmonizeTowardZero(sol);  // Modifies 'sol' in place

      if (is_valid_fn && redundant_sol_fn)
      {
        if (is_valid_fn_(sol))
          solution_set.insert(end(solution_set), sol, sol + 6);  // If good then add to solution set

        std::vector<FloatType> redundant_sols = redundant_sol_fn(sol);
        if (!redundant_sols.empty())
        {
          int num_sol = static_cast<int>(redundant_sols.size() / 6);
          for (int s = 0; s < num_sol; ++s)
          {
            FloatType* redundant_sol = redundant_sols.data() + 6 * s;
            if (is_valid_fn_(redundant_sol))
              solution_set.insert(end(solution_set), redundant_sol, redundant_sol + 6);  // If good then add to solution
                                                                                         // set
          }
        }
      }
      else if (is_valid_fn && !redundant_sol_fn)
      {
        if (is_valid_fn(sol))
          solution_set.insert(end(solution_set), sol, sol + 6);  // If good then add to solution set
      }
      else if (!is_valid_fn && redundant_sol_fn)
      {
        solution_set.insert(end(solution_set), sol, sol + 6);  // If good then add to solution set

        std::vector<FloatType> redundant_sols = redundant_sol_fn(sol);
        if (!redundant_sols.empty())
        {
          int num_sol = static_cast<int>(redundant_sols.size() / 6);
          for (int s = 0; s < num_sol; ++s)
          {
            FloatType* redundant_sol = redundant_sols.data() + 6 * s;
            solution_set.insert(end(solution_set), redundant_sol, redundant_sol + 6);  // If good then add to solution
                                                                                       // set
          }
        }
      }
      else
      {
        solution_set.insert(end(solution_set), sol, sol + 6);
      }
    }
  }

  return !solution_set.empty();
}

template <typename FloatType>
bool OPWKinematics<FloatType>::fk(const FloatType* pose,
                                  Eigen::Transform<FloatType, 3, Eigen::Isometry>& solution) const
{
  solution = opw_kinematics::forward<FloatType>(params_, pose);
  solution = world_to_base_ * solution * tool0_to_tip_.inverse();
  return true;
}

template <typename FloatType>
int OPWKinematics<FloatType>::dof() const
{
  return 6;
}

template <typename FloatType>
void OPWKinematics<FloatType>::analyzeIK(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p) const
{
  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "AnalyzeIK: ", ";");

  std::stringstream ss;
  ss << p.matrix().format(CommaInitFmt);
  CONSOLE_BRIDGE_logInform(ss.str().c_str());

  std::string valid_fn_defined = "\tIs Valid Function: " + std::string(is_valid_fn_ ? "True" : "False");
  CONSOLE_BRIDGE_logInform(valid_fn_defined.c_str());
  std::string redundant_fn_defined = "\tIs Valid Function: " + std::string(is_valid_fn_ ? "True" : "False");
  CONSOLE_BRIDGE_logInform(redundant_fn_defined.c_str());

  std::vector<FloatType> solution_set;
  ik(p, nullptr, nullptr, solution_set);
  ss.clear();
  ss << "\tSampling without functions, found solutions: " << solution_set.size() / 8;
  CONSOLE_BRIDGE_logInform(ss.str().c_str());

  solution_set.clear();
  ik(p, is_valid_fn_, nullptr, solution_set);
  ss.clear();
  ss << "\tSampling with only IsValid functions, found solutions: " << solution_set.size() / 8;
  CONSOLE_BRIDGE_logInform(ss.str().c_str());

  solution_set.clear();
  ik(p, nullptr, redundant_sol_fn_, solution_set);
  ss.clear();
  ss << "\tSampling with only Redundant Solutions functions, found solutions: " << solution_set.size() / 8;
  CONSOLE_BRIDGE_logInform(ss.str().c_str());

  solution_set.clear();
  ik(p, is_valid_fn_, redundant_sol_fn_, solution_set);
  ss.clear();
  ss << "\tSampling with both functions, found solutions: " << solution_set.size() / 8;
  CONSOLE_BRIDGE_logInform(ss.str().c_str());
}

}  // namespace descartes_light

#endif  // DESCARTES_OPW_IMPL_DESCARTES_OPW_KINEMATICS_HPP
