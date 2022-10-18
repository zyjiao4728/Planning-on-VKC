#include <tesseract_motion_planners/3mo/3mo_motion_planner.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_solver_profile.h>
#include <vkc_example/utils.h>

using namespace std;
using namespace tesseract_rosutils;
using namespace tesseract_planning;
using namespace trajopt;

void solveProb(PlannerRequest request, PlannerResponse &response, int n_iter) {
  // Set the optimization parameters (Most are being left as defaults)

  ROS_WARN("Constructed optimization problem. Starting optimization.");

  // Solve problem. Results are stored in the response
  TrajOptMotionPlanner planner;

  auto trajopt_status = planner.solve(request, response, true);

  ROS_WARN("%d, %s", trajopt_status.value(), trajopt_status.message().c_str());

  return;
}

void solveOmplProb(PlannerRequest request, PlannerResponse &response,
                   int n_iter) {
  ROS_WARN("constructed ompl problem, solving...");

  OMPLMotionPlanner planner;
  tesseract_planning::MMMOMotionPlanner ik_planner;
  tesseract_common::StatusCode planning_status;
  if (request.name == "3MO_IK_TRAJ") {
    planning_status = ik_planner.solve(request, response, true);
  } else {
    planning_status = planner.solve(request, response, true);
  }

  CONSOLE_BRIDGE_logWarn("%d, %s", planning_status.value(),
                         planning_status.message().c_str());
  return;
}

CostInfo solveProb_cost(TrajOptProb::Ptr prob_ptr, PlannerResponse &response,
                        int n_iter, bool enable_plotting) {
  // ProcessPlanningServer
  // planning_server(std::make_shared<ROSProcessEnvironmentCache>(monitor_), 5);
  // planning_server.loadDefaultProcessPlanners();

  // auto trajopt_plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();

  // auto trajopt_composite_profile =
  // std::make_shared<TrajOptDefaultCompositeProfile>();
  // trajopt_composite_profile->collision_constraint_config.enabled = false;
  // trajopt_composite_profile->collision_cost_config.safety_margin = 0.005;
  // trajopt_composite_profile->collision_cost_config.coeff = 50;

  // // Set the optimization parameters (Most are being left as defaults)
  // auto trajopt_solver_profile =
  // std::make_shared<TrajOptDefaultSolverProfile>();
  // // TrajOptPlannerConfig config(prob_ptr);
  // trajopt_solver_profile->opt_info.max_iter = n_iter;
  // trajopt_solver_profile->opt_info.cnt_tolerance = 1e-2;
  // trajopt_solver_profile->opt_info.trust_expand_ratio = 1.5;
  // trajopt_solver_profile->opt_info.trust_shrink_ratio = 0.5;
  // trajopt_solver_profile->opt_info.min_trust_box_size = 1e-3;
  // trajopt_solver_profile->opt_info.min_approx_improve = 1e-3;

  // planning_server.getProfiles()->addProfile<TrajOptPlanProfile>(
  //     profile_ns::TRAJOPT_DEFAULT_NAMESPACE, "CARTESIAN",
  //     trajopt_plan_profile);
  // planning_server.getProfiles()->addProfile<TrajOptCompositeProfile>(
  //     profile_ns::TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT",
  //     trajopt_composite_profile);
  // planning_server.getProfiles()->addProfile<TrajOptSolverProfile>(
  //     profile_ns::TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT",
  //     trajopt_solver_profile);

  // // Create the planner and the responses that will store the results
  // TrajOptMotionPlanner planner;

  // // Set Planner Configuration
  // // planner.c(config);

  // // ROS_WARN("Constructed optimization problem with cost constraint.
  // Starting optimization.");
  // // // std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  // // CostInfo cost;
  // // // Solve problem. Results are stored in the response
  // // planner.solve(response, cost.cost_vals, cost.cnt_viols,
  // cost.total_cost);

  // // if(enable_plotting)
  // // {
  // //   ROS_WARN("[%s]plotting is enabled, plot callback is configured.",
  // __func__);
  // //   tesseract_rosutils::ROSPlottingPtr plotter =
  // // std::make_shared<tesseract_rosutils::ROSPlotting>(prob_ptr->GetEnv());

  // //   config.callbacks.push_back(PlotCallback(*prob_ptr, plotter));
  // // }
  CostInfo cost;

  return cost;
}

void refineTrajectory(tesseract_common::JointTrajectory &traj,
                      vkc::VKCEnvBasic &env) {
  std::vector<std::string> joint_names =
      env.getVKCEnv()->getTesseract()->getActiveJointNames();
  Eigen::VectorXd initial_state =
      env.getVKCEnv()->getTesseract()->getCurrentJointValues();
  std::vector<std::string> planned_joint_names = traj.states[0].joint_names;

  long int n_rows = traj.states.size();  // number of waypoints
  long int n_cols = joint_names.size();  // number of joints

  for (int i = 0; i < n_rows; i++) {
    Eigen::VectorXd current_waypoint = traj.states[i].position;

    for (int j = 0; j < planned_joint_names.size(); j++) {
      auto it =
          find(joint_names.begin(), joint_names.end(), planned_joint_names[j]);
      if (it != joint_names.end()) {
        int index = it - joint_names.begin();
        initial_state[index] = current_waypoint[j];
      } else {
        continue;
      }
    }
    traj.states[i].joint_names = joint_names;
    traj.states[i].position = initial_state;
  }

  double prev_orientation = 0.0;

  for (int i = 0; i < n_rows - 1; i++) {
    double delta_y =
        traj.states[i + 1].position[1] - traj.states[i].position[1];
    double delta_x =
        traj.states[i + 1].position[0] - traj.states[i].position[0];
    double delta_orientation = atan2(delta_y, delta_x);

    if (i == 0) {
      prev_orientation = delta_orientation;
    }

    if (abs(delta_orientation - prev_orientation) > M_PI) {
      if (delta_orientation > prev_orientation) {
        delta_orientation -= 2.0 * M_PI;
      } else {
        delta_orientation += 2.0 * M_PI;
      }
    }
    prev_orientation = delta_orientation;

    if (n_cols > 3) {
      traj.states[i].position[3] +=
          traj.states[i].position[2] - delta_orientation;
    }
    traj.states[i].position[2] = delta_orientation;
  }

  // std::cout << traj.states[0].position  << std::endl;

  // the last orientation is as same as the 2nd last
  if (n_rows > 1) {
    if (n_cols > 3) {
      traj.states[n_rows - 1].position[3] +=
          traj.states[n_rows - 1].position[2] -
          traj.states[n_rows - 2].position[2];
    }
    traj.states[n_rows - 1].position[2] = traj.states[n_rows - 2].position[2];
  }

  // long int n_rows = traj.rows();
  // long int n_cols = traj.cols();

  // for (int i = 0; i < n_rows - 1; i++) {
  //   double delta_y = traj(i + 1, 1) - traj(i, 1);
  //   double delta_x = traj(i + 1, 0) - traj(i, 0);
  //   double delta_orientation = atan2(delta_y, delta_x);

  //   if (delta_orientation > 3.14159265359) delta_orientation
  //   -= 3.14159265359;

  //   if (delta_orientation < -3.14159265359) delta_orientation
  //   += 3.14159265359;

  //   if (n_cols > 3) {
  //     traj(i, 3) += traj(i, 2) - delta_orientation;
  //   }
  //   traj(i, 2) = delta_orientation;
  // }

  // // the last orientation is as same as the 2nd last
  // if (n_rows > 1) {
  //   if (n_cols > 3) {
  //     traj(n_rows - 1, 3) += traj(n_rows - 1, 2) - traj(n_rows - 2, 2);
  //   }
  //   traj(n_rows - 1, 2) = traj(n_rows - 2, 2);
  // }
}

int saveTrajToFile(const tesseract_common::TrajArray &traj,
                   const std::string filename) {
  ofstream fileout;
  fileout.open(filename, std::ios_base::app);

  if (!fileout.is_open()) {
    std::cout << "Cannot open file: " << filename << std::endl;
    return -1;
  }

  long n_rows = traj.rows();
  long n_cols = traj.cols();

  for (size_t i = 0; i < n_rows; i++) {
    for (size_t j = 0; j < n_cols; j++) {
      fileout << traj(i, j) << ",\n"[j == n_cols - 1];
    }
  }

  fileout.close();
  std::cout << "Trajectory file save at: " << filename << std::endl;

  return 0;
}

int saveDataToFile(const std::vector<double> &data,
                   const std::string filename) {
  ofstream fileout;
  fileout.open(filename, std::ios_base::app);

  if (!fileout.is_open()) {
    std::cout << "Cannot open file: " << filename << std::endl;
    return 0;
  }

  for (auto item : data) {
    fileout << item << ",";
  }
  fileout << "\n";
  fileout.close();
  std::cout << "Trajectory file save at: " << filename << std::endl;

  return 1;
}

void interpVKCData(
    std::vector<double> &data, std::vector<double> &elapsed_time,
    std::vector<tesseract_common::JointTrajectory> &joint_trajs) {
  for (int i = 0; i < elapsed_time.size(); i++) {
    data.emplace_back(elapsed_time[i]);
    if (elapsed_time[i] < 0.) {
      data.emplace_back(-1);
      data.emplace_back(-1);
      data.emplace_back(0);
      continue;
    }
    std::vector<Eigen::VectorXd> base_trajectory;
    std::vector<Eigen::VectorXd> arm_trajectory;
    for (auto state : joint_trajs[i].states) {
      base_trajectory.emplace_back(state.position.head(2));
      arm_trajectory.emplace_back(state.position.segment(3, 6));
    }
    double base_cost = computeTrajLength(base_trajectory);
    double arm_cost = computeTrajLength(arm_trajectory);
    data.emplace_back(base_cost);
    data.emplace_back(arm_cost);
    data.emplace_back(1);
  }
}

void setBaseJoint(vkc::ActionBase::Ptr action) {
  action->setBaseJoint(std::string("base_y_base_x"),
                       std::string("base_theta_base_y"));
}

std::vector<vkc::JointDesiredPose> getJointHome(
    std::unordered_map<std::string, double> home_pose) {
  std::vector<vkc::JointDesiredPose> joint_home;
  for (auto &pose : home_pose) {
    if (pose.first.substr(0, pose.first.find("_", 0)) == "base") continue;
    joint_home.push_back(vkc::JointDesiredPose(pose.first, pose.second));
  }
  return joint_home;
}

void TrajectoryVisualize(
    vkc::VKCEnvBasic &env, vkc::ActionSeq &actions,
    vector<tesseract_common::JointTrajectory> &joint_trajs) {
  ROS_INFO("[%s]actions size: %ld, traj size: %ld", __func__, actions.size(),
           joint_trajs.size());

  // reset rviz
  ROS_INFO("[%s]please reset rviz and presss ENTER to go on...", __func__);
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  env.reInit();

  long int max_traj_len{0};
  for (auto &traj : joint_trajs) {
    max_traj_len = traj.size() > max_traj_len ? traj.size() : max_traj_len;
    // std::cout << ">> traj details: " << std::endl
    //           << traj << std::endl;
  }

  // plot current `action` result
  auto action_iter = actions.begin();
  auto joint_traj_iter = joint_trajs.begin();

  // plot current `action` trajectory data
  for (; joint_traj_iter != joint_trajs.end();
       ++action_iter, ++joint_traj_iter) {
    // ROS_INFO("joints names number: %d, joint number: %d, joint states number:
    // %d, revision: %d",
    //          joint_traj_iter->joint_names.size(),
    //          joint_traj_iter->trajectory.cols(),
    //          joint_traj_iter->trajectory.rows(),
    //          env.getPlotVKCEnv()->getTesseract()->getRevision());
    // tesseract_common::TrajArray traj = joint_traj_iter->
    // ->trajectory.leftCols(static_cast<long>(joint_traj_iter->joint_names.size()));
    // std::cout << "Traj:" << std::endl
    //           << traj << std::endl;

    // ROSPlottingPtr plotter =
    // std::make_shared<ROSPlotting>(env.getPlotVKCEnv()->getTesseract()->getEnvironment());
    // plotter->plotTrajectory(joint_traj_iter->joint_names, traj);
    // usleep((useconds_t)(joint_traj_iter->trajectory.size() * 1500000.0 /
    // max_traj_len));

    // // update env according to the action
    // // std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    // env.updatePlotEnv(joint_traj_iter->joint_names,
    // joint_traj_iter->trajectory.bottomRows(1).transpose(), *action_iter);
    // plotter->clear();
  }
}

double computeTrajLength(const std::vector<Eigen::VectorXd> trajectory) {
  double dist_travelled = 0.0;

  for (int i = 1; i < trajectory.size(); i++) {
    Eigen::VectorXd curr_state = trajectory[i];
    Eigen::VectorXd prev_state = trajectory[i - 1];
    dist_travelled += (curr_state - prev_state).array().abs().sum();
  }

  return dist_travelled;
}

Eigen::VectorXd sampleBasePose(vkc::VKCEnvBasic &env,
                               Eigen::Isometry3d ee_goal) {
  tesseract_kinematics::KinematicGroup::Ptr kin =
      std::move(env.getVKCEnv()->getTesseract()->getKinematicGroup("vkc"));
  tesseract_kinematics::KinGroupIKInputs ik_inputs;

  ik_inputs.push_back(tesseract_kinematics::KinGroupIKInput(
      ee_goal, "world", "robotiq_arg2f_base_link"));

  tesseract_common::KinematicLimits limits = kin->getLimits();
  tesseract_collision::ContactResultMap contact_results;
  tesseract_srdf::JointGroup joint_names = kin->getJointNames();
  Eigen::VectorXd initial_joint_values =
      env.getVKCEnv()->getTesseract()->getCurrentJointValues(
          kin->getJointNames());

  double max_cost = 1e6;
  Eigen::VectorXd ik_result = initial_joint_values;
  int sample_base_pose_tries = 0;
  int feasible_poses = 0;
  while (sample_base_pose_tries < 10000 && feasible_poses < 100) {
    sample_base_pose_tries++;
    Eigen::VectorXd ik_seed =
        tesseract_common::generateRandomNumber(limits.joint_limits);
    tesseract_kinematics::IKSolutions result =
        kin->calcInvKin(ik_inputs, ik_seed);
    for (const auto &res : result) {
      env.getVKCEnv()->getTesseract()->setState(joint_names, res);
      env.getVKCEnv()->getTesseract()->getDiscreteContactManager()->contactTest(
          contact_results, tesseract_collision::ContactTestType::ALL);
      if (contact_results.size() == 0) {
        if ((ik_seed.head(2) - res.head(2)).array().abs().sum() < max_cost) {
          ik_result = res;
          max_cost = (ik_seed.head(2) - res.head(2)).array().abs().sum();
        }
        feasible_poses++;
      }
      contact_results.clear();
    }
  }
  env.getVKCEnv()->getTesseract()->setState(joint_names, initial_joint_values);
  return ik_result;
}

bool sampleArmPose1(vkc::VKCEnvBasic &env, Eigen::Isometry3d ee_goal,
                    Eigen::VectorXd &ik_result) {
  tesseract_kinematics::KinematicGroup::Ptr kin =
      std::move(env.getVKCEnv()->getTesseract()->getKinematicGroup("arm"));
  tesseract_kinematics::KinGroupIKInputs ik_inputs;

  ik_inputs.push_back(tesseract_kinematics::KinGroupIKInput(
      ee_goal, "world", "robotiq_arg2f_base_link"));

  tesseract_collision::ContactResultMap contact_results;
  tesseract_srdf::JointGroup joint_names = kin->getJointNames();
  Eigen::VectorXd ik_seed =
      env.getVKCEnv()->getTesseract()->getCurrentJointValues(
          kin->getJointNames());

  ik_result = ik_seed;
  tesseract_kinematics::IKSolutions result;
  for (int tries = 0; tries < 200; tries++) {
    result = kin->calcInvKin(ik_inputs, ik_seed);
    if (result.size() > 0) break;
  }

  double max_cost = 1e6;
  bool in_collision = true;
  for (const auto &res : result) {
    if ((ik_seed - res).array().abs().sum() < max_cost) {
      env.getVKCEnv()->getTesseract()->setState(joint_names, res);
      env.getVKCEnv()->getTesseract()->getDiscreteContactManager()->contactTest(
          contact_results, tesseract_collision::ContactTestType::ALL);
      if (contact_results.size() > 0) {
        in_collision = true;
      } else {
        ik_result = res;
        max_cost = (ik_seed - res).array().abs().sum();
        in_collision = false;
        std::cout << "found ik, collision: " << in_collision
                  << ", cost: " << max_cost << std::endl;
      }
    }
  }
  if (in_collision) {
    std::cout << "ik not found." << std::endl;
  }
  contact_results.clear();
  return !in_collision;
}

bool sampleArmPose2(
    vkc::VKCEnvBasic &env, std::string target_joint, double target_value,
    Eigen::VectorXd &ik_result,
    vkc::BaseObject::AttachLocation::ConstPtr attach_location_ptr,
    int remaining_steps) {
  std::unordered_map<std::string, double> joint_init;
  joint_init[target_joint] =
      env.getVKCEnv()->getTesseract()->getCurrentJointValues(
          std::vector<std::string>({target_joint}))[0];

  // std::cout << 1 << std::endl;
  tesseract_kinematics::KinematicGroup::Ptr kin =
      std::move(env.getVKCEnv()->getTesseract()->getKinematicGroup("arm"));

  if (abs(joint_init[target_joint] - target_value) < 0.02) {
    ik_result = env.getVKCEnv()->getTesseract()->getCurrentJointValues(
        kin->getJointNames());
    return true;
  }

  int max_inc = 100;
  double joint_res =
      (target_value - joint_init[target_joint]) / remaining_steps;
  double joint_fineres =
      (target_value - joint_init[target_joint]) / remaining_steps / 10;
  int n_inc = 0;

  bool no_collision = false;
  bool found_ik = false;

  joint_init[target_joint] += joint_res;

  env.getVKCEnv()->getTesseract()->setState(joint_init);

  tesseract_kinematics::IKSolutions result;

  tesseract_kinematics::KinGroupIKInputs ik_inputs;
  Eigen::Isometry3d ee_target =
      env.getVKCEnv()->getTesseract()->getLinkTransform(
          attach_location_ptr->link_name_) *
      attach_location_ptr->local_joint_origin_transform;
  ik_inputs.push_back(tesseract_kinematics::KinGroupIKInput(
      ee_target, "world", "robotiq_arg2f_base_link"));
  tesseract_srdf::JointGroup joint_names = kin->getJointNames();
  Eigen::VectorXd ik_seed =
      env.getVKCEnv()->getTesseract()->getCurrentJointValues(
          kin->getJointNames());
  ik_result = ik_seed;
  // std::cout << 2 << std::endl;
  for (int tries = 0; tries < 200; tries++) {
    result = kin->calcInvKin(ik_inputs, ik_seed);
    if (result.size() > 0) {
      found_ik = true;
      ik_result = result[0];
      break;
    }
  }
  // std::cout << 3 << std::endl;
  tesseract_collision::ContactResultMap contact_results;
  double max_cost = 1e6;
  for (const auto &res : result) {
    if ((ik_seed - res).array().abs().sum() < max_cost) {
      env.getVKCEnv()->getTesseract()->setState(joint_names, res);
      env.getVKCEnv()->getTesseract()->getDiscreteContactManager()->contactTest(
          contact_results, tesseract_collision::ContactTestType::ALL);
      if (contact_results.size() > 0) {
        for (auto contact_result : contact_results) {
          std::cout << contact_result.first.first << ":\t"
                    << contact_result.first.second << std::endl;
        }
        no_collision = false;
      } else {
        max_cost = (ik_seed - res).array().abs().sum();
        ik_result = res;
        no_collision = true;
      }
    }
  }
  // std::cout << 4 << std::endl;
  if (no_collision) {
    std::cout << std::boolalpha;
    std::cout << "found ik " << found_ik << " in collision: " << !no_collision
              << ", cost: " << max_cost
              << ", joint_init: " << joint_init[target_joint] << std::endl;
    return (found_ik && no_collision);
  }

  auto temp_joint_init = joint_init;
  // std::cout << 5 << std::endl;
  while (!no_collision && n_inc < max_inc) {
    n_inc++;
    ik_inputs.clear();

    if (found_ik) {
      temp_joint_init[target_joint] += joint_fineres;
      if (abs(temp_joint_init[target_joint]-joint_init[target_joint])>abs(target_value-joint_init[target_joint])){
        temp_joint_init[target_joint] = target_value;
      }
      env.getVKCEnv()->getTesseract()->setState(temp_joint_init);
    } else {
      temp_joint_init[target_joint] -= joint_fineres;
      if (target_value > 0){
        temp_joint_init[target_joint] =
          std::max(0.0, temp_joint_init[target_joint]);
      }
      else{
        temp_joint_init[target_joint] =
          std::min(0.0, temp_joint_init[target_joint]);
      }
      
      env.getVKCEnv()->getTesseract()->setState(temp_joint_init);
    }
    // std::cout << 6 << std::endl;
    Eigen::Isometry3d ee_target =
        env.getVKCEnv()->getTesseract()->getLinkTransform(
            attach_location_ptr->link_name_) *
        attach_location_ptr->local_joint_origin_transform;
    ik_inputs.push_back(tesseract_kinematics::KinGroupIKInput(
        ee_target, "world", "robotiq_arg2f_base_link"));

    for (int tries = 0; tries < 200; tries++) {
      result = kin->calcInvKin(ik_inputs, ik_seed);
      if (result.size() > 0) {
        ik_result = result[0];
        found_ik = true;
        break;
      } else {
        found_ik = false;
      }
    }
    max_cost = 1e6;
    // std::cout << 7 << std::endl;
    for (const auto &res : result) {
      if ((ik_seed - res).array().abs().sum() < max_cost) {
        env.getVKCEnv()->getTesseract()->setState(joint_names, res);
        env.getVKCEnv()
            ->getTesseract()
            ->getDiscreteContactManager()
            ->contactTest(contact_results,
                          tesseract_collision::ContactTestType::ALL);
        if (contact_results.size() > 0) {
          // for (auto contact_result : contact_results) {
          //   std::cout << contact_result.first.first << ":\t"
          //             << contact_result.first.second << std::endl;
          // }
          no_collision = false;
        } else {
          max_cost = (ik_seed - res).array().abs().sum();
          ik_result = res;
          no_collision = true;
        }
      }
      contact_results.clear();
    }
    // std::cout << 8 << std::endl;
    if (found_ik) {
      std::cout << std::boolalpha;
      std::cout << "found ik " << found_ik << " in collision: " << !no_collision
                << ", cost: " << max_cost
                << ", temp_joint_init: " << temp_joint_init[target_joint]
                << std::endl;
    }
  }
  return (found_ik && no_collision);
}
