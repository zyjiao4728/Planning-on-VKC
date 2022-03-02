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
#ifndef DESCARTES_IKFAST_IKFAST_KINEMATICS_HPP
#define DESCARTES_IKFAST_IKFAST_KINEMATICS_HPP

#define IKFAST_HAS_LIBRARY
#define IKFAST_NO_MAIN

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <descartes_ikfast/external/ikfast.h>
#include <console_bridge/console.h>
DESCARTES_IGNORE_WARNINGS_POP

#include <descartes_ikfast/ikfast_kinematics.h>
#include <descartes_light/utils.h>

namespace descartes_light
{
template <typename FloatType>
IKFastKinematics<FloatType>::IKFastKinematics(
    const Eigen::Transform<FloatType, 3, Eigen::Isometry>& world_to_robot_base,
    const Eigen::Transform<FloatType, 3, Eigen::Isometry>& tool0_to_tip,
    const IsValidFn<FloatType>& is_valid_fn,
    const GetRedundantSolutionsFn<FloatType>& redundant_sol_fn)
  : world_to_robot_base_(world_to_robot_base)
  , tool0_to_tip_(tool0_to_tip)
  , is_valid_fn_(is_valid_fn)
  , redundant_sol_fn_(redundant_sol_fn)
{
}

template <typename FloatType>
bool IKFastKinematics<FloatType>::ik(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
                                     std::vector<FloatType>& solution_set) const
{
  return IKFastKinematics<FloatType>::ik(p, is_valid_fn_, redundant_sol_fn_, solution_set);
}

template <typename FloatType>
bool IKFastKinematics<FloatType>::ik(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
                                     const IsValidFn<FloatType>& is_valid_fn,
                                     const GetRedundantSolutionsFn<FloatType>& redundant_sol_fn,
                                     std::vector<FloatType>& solution_set) const
{
  const Eigen::Transform<FloatType, 3, Eigen::Isometry> in_robot =
      world_to_robot_base_.inverse() * p * tool0_to_tip_.inverse();

  // Convert to ikfast data type
  Eigen::Transform<IkReal, 3, Eigen::Isometry> ikfast_tcp = in_robot.template cast<IkReal>();

  // Decompose
  const Eigen::Matrix<IkReal, 3, 1> translation = ikfast_tcp.translation();

  // Note the row major ordering here: IkFast expects the matrix in r00, r01, r02, ..., r11, r12, r13
  // ordering
  const Eigen::Matrix<IkReal, 3, 3, Eigen::RowMajor> rotation = ikfast_tcp.rotation();

  // Call IK (TODO: Make a better solution list class? One that uses vector instead of list)
  ikfast::IkSolutionList<IkReal> ikfast_solution_set;
  ComputeIk(translation.data(), rotation.data(), nullptr, ikfast_solution_set);

  // Unpack the solutions into the output vector
  const auto n_sols = ikfast_solution_set.GetNumSolutions();
  std::size_t ikfast_dof = static_cast<std::size_t>(dof());

  std::vector<IkReal> ikfast_output;
  ikfast_output.resize(n_sols * ikfast_dof);

  for (std::size_t i = 0; i < n_sols; ++i)
  {
    // This actually walks the list EVERY time from the start of i.
    const auto& sol = ikfast_solution_set.GetSolution(i);
    auto* out = ikfast_output.data() + i * ikfast_dof;
    sol.GetSolution(out, nullptr);
  }

  std::vector<FloatType> sols;
  sols.insert(end(sols), ikfast_output.begin(), ikfast_output.end());

  // Check the output
  std::size_t num_sol = sols.size() / ikfast_dof;
  for (std::size_t i = 0; i < num_sol; i++)
  {
    FloatType* sol = sols.data() + ikfast_dof * i;
    if (isValid<FloatType>(sol, static_cast<int>(ikfast_dof)))
    {
      harmonizeTowardZero<FloatType>(sol, static_cast<int>(ikfast_dof));  // Modifies 'sol' in place

      std::vector<FloatType> full_sol;
      full_sol.insert(end(full_sol), sol, sol + ikfast_dof);  // And then insert the robot arm configuration

      if (is_valid_fn && redundant_sol_fn)
      {
        if (is_valid_fn(full_sol.data()))
          solution_set.insert(end(solution_set), full_sol.data(), full_sol.data() + ikfast_dof);  // If good then add to
                                                                                                  // solution set

        std::vector<FloatType> redundant_sols = redundant_sol_fn(full_sol.data());
        if (!redundant_sols.empty())
        {
          std::size_t num_sol = redundant_sols.size() / ikfast_dof;
          for (std::size_t s = 0; s < num_sol; ++s)
          {
            FloatType* redundant_sol = redundant_sols.data() + ikfast_dof * s;
            if (is_valid_fn(redundant_sol))
              solution_set.insert(end(solution_set), redundant_sol, redundant_sol + ikfast_dof);  // If good then add to
                                                                                                  // solution set
          }
        }
      }
      else if (is_valid_fn && !redundant_sol_fn)
      {
        if (is_valid_fn(full_sol.data()))
          solution_set.insert(end(solution_set), full_sol.data(), full_sol.data() + ikfast_dof);  // If good then add to
                                                                                                  // solution set
      }
      else if (!is_valid_fn && redundant_sol_fn)
      {
        solution_set.insert(end(solution_set), full_sol.data(), full_sol.data() + ikfast_dof);  // If good then add to
                                                                                                // solution set

        std::vector<FloatType> redundant_sols = redundant_sol_fn(full_sol.data());
        if (!redundant_sols.empty())
        {
          std::size_t num_sol = redundant_sols.size() / ikfast_dof;
          for (std::size_t s = 0; s < num_sol; ++s)
          {
            FloatType* redundant_sol = redundant_sols.data() + ikfast_dof * s;
            solution_set.insert(end(solution_set), redundant_sol, redundant_sol + ikfast_dof);  // If good then add to
                                                                                                // solution set
          }
        }
      }
      else
      {
        solution_set.insert(end(solution_set), full_sol.data(), full_sol.data() + ikfast_dof);
      }
    }
  }

  return !solution_set.empty();
}

template <typename FloatType>
bool IKFastKinematics<FloatType>::fk(const FloatType* pose,
                                     Eigen::Transform<FloatType, 3, Eigen::Isometry>& solution) const
{
  Eigen::Matrix<IkReal, 3, 1> translation;
  // Note the row major ordering here
  Eigen::Matrix<IkReal, 3, 3, Eigen::RowMajor> rotation;

  std::vector<IkReal> ikfast_joint_pose(pose, pose + dof());  // TODO: Need to convert joint_pose
  ComputeFk(ikfast_joint_pose.data(), translation.data(), rotation.data());

  Eigen::Transform<FloatType, 3, Eigen::Isometry> output;
  output.setIdentity();
  output.translation() = translation.cast<FloatType>();
  output.linear() = rotation.cast<FloatType>();

  solution = world_to_robot_base_ * output * tool0_to_tip_;
  return true;
}

template <typename FloatType>
int IKFastKinematics<FloatType>::dof() const
{
  return GetNumJoints();
}

template <typename FloatType>
void IKFastKinematics<FloatType>::analyzeIK(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p) const
{
  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "AnalyzeIK: ", ";");
  std::stringstream ss;
  ss << p.matrix().format(CommaInitFmt);
  CONSOLE_BRIDGE_logInform(ss.str().c_str());

  std::string is_valid_fn_defined = "\tIs Valid Function: " ? "True" : "False";  // NOLINT
  CONSOLE_BRIDGE_logInform(is_valid_fn_defined.c_str());
  std::string is_redundant_fn_defined = "\tGet Redundant Solutions Function: " ? "True" : "False";  // NOLINT
  CONSOLE_BRIDGE_logInform(is_redundant_fn_defined.c_str());

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
#endif  // DESCARTES_IKFAST_IKFAST_KINEMATICS_HPP
