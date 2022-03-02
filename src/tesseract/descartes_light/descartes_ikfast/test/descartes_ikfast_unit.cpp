
#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <algorithm>
#include <memory>
#include <ostream>
DESCARTES_IGNORE_WARNINGS_POP

#include <descartes_ikfast_fanuc_m20ia10l_manipulator.h>

TEST(DescartesIkFastUnit, Instantiation)
{
  Eigen::Isometry3d world_to_robot_base = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tool0_to_tip = Eigen::Isometry3d::Identity();
  descartes_ikfast_unit::FanucM20ia10lKinematicsD robot(world_to_robot_base, tool0_to_tip, nullptr, nullptr);

  Eigen::Isometry3d pose;
  std::vector<double> all_zeros(6, 0.0);

  EXPECT_TRUE(robot.fk(all_zeros.data(), pose));

  std::vector<double> ik_solution;
  EXPECT_TRUE(robot.ik(pose, ik_solution));

  int num_dof = robot.dof();
  int num_sol = static_cast<int>(ik_solution.size()) / num_dof;
  for (int i = 0; i < num_sol; ++i)
  {
    double* sol = ik_solution.data() + i * num_dof;
    Eigen::Isometry3d sol_pose;
    EXPECT_TRUE(robot.fk(sol, sol_pose));
    EXPECT_TRUE(pose.isApprox(sol_pose, 1e-8));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
