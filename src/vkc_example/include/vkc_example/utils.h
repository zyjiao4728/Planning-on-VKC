#ifndef VKC_EXAMPLE_UTILS_H
#define VKC_EXAMPLE_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>

#include <trajopt/problem_description.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <vkc/action/actions.h>
#include <vkc/env/vkc_env_basic.h>

#include <fstream>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/logging.hpp>
#include <vector>

class CostInfo {
 public:
  std::vector<double> cost_vals;
  std::vector<double> cnt_viols;
  double total_cost;

 public:
  double getSmoothnessCost() {
    double ret = 0;

    for (unsigned int i = 0; i < cost_vals.size(); i++) {
      ret += cost_vals[i];
    }
    return ret;
  }

  double getCollisionCost() {
    double ret = 0;

    for (unsigned int i = 1; i < cnt_viols.size(); i++) {
      ret += cnt_viols[i];
    }

    return ret;
  }

  double getTotalCost() { return total_cost; }

  double getGoalCost() { return cnt_viols[0]; }
};

void solveProb(tesseract_planning::PlannerRequest request,
               tesseract_planning::PlannerResponse &response, int n_iter);

void solveOmplProb(tesseract_planning::PlannerRequest request,
                   tesseract_planning::PlannerResponse &response, int n_iter);

// CostInfo solveProb_cost(trajopt::TrajOptProb::Ptr prob_ptr,
// tesseract_planning::PlannerResponse &response,
//                         int n_iter, bool enable_ploting = false);

void refineTrajectory(tesseract_common::JointTrajectory &traj,
                      vkc::VKCEnvBasic &env);

int saveTrajToFile(const tesseract_common::TrajArray &traj,
                   const std::string filename);

double computeTrajLength(const std::vector<Eigen::VectorXd> trajectory);

std::vector<vkc::JointDesiredPose> getJointHome(
    std::unordered_map<std::string, double> home_pose);

void TrajectoryVisualize(
    vkc::VKCEnvBasic &env, vkc::ActionSeq &actions,
    std::vector<tesseract_common::JointTrajectory> &joint_trajs);

int saveDataToFile(const std::vector<double> &data, const std::string filename);

void interpVKCData(std::vector<double> &data, std::vector<double> &elapsed_time,
                   std::vector<tesseract_common::JointTrajectory> &joint_trajs);

void setBaseJoint(vkc::ActionBase::Ptr action);

class OutputHandlerColorSTD : public console_bridge::OutputHandlerSTD {
  void log(const std::string &text, console_bridge::LogLevel level,
           const char *filename, int line) {
    auto current_time =
        std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    auto t = std::localtime(&current_time);
    char buf[80];
    std::strftime(buf, sizeof(buf), "%T", t);
    switch (level) {
      case console_bridge::CONSOLE_BRIDGE_LOG_DEBUG:
        std::cout << buf << "[DEBUG]: " << text << std::endl;
        break;
      case console_bridge::CONSOLE_BRIDGE_LOG_INFO:
        std::cout << "\033[34m" << buf << "[INFO]: " << text << "\033[0m"
                  << std::endl;
        break;
      case console_bridge::CONSOLE_BRIDGE_LOG_WARN:
        std::cerr << "\033[33m" << buf << "[WARN]: " << text << "\033[0m"
                  << std::endl;
        break;
      case console_bridge::CONSOLE_BRIDGE_LOG_ERROR:
        std::cerr << "\033[31m" << buf << "[ERROR]: " << text << "\033[0m"
                  << std::endl;
      default:
        break;
    }
    if (level >= console_bridge::CONSOLE_BRIDGE_LOG_WARN) {
      std::cerr << "         at line " << line << " in " << filename
                << std::endl;
      std::cerr.flush();
    } else {
      std::cout.flush();
    }
  }
};

void setupLog(
    console_bridge::LogLevel level = console_bridge::CONSOLE_BRIDGE_LOG_INFO) {
  OutputHandlerColorSTD *oh = new OutputHandlerColorSTD();
  console_bridge::useOutputHandler(oh);
  console_bridge::setLogLevel(level);
}

Eigen::VectorXd sampleBasePose(vkc::VKCEnvBasic &env,
                               Eigen::Isometry3d ee_goal);

bool sampleArmPose1(vkc::VKCEnvBasic &env, Eigen::Isometry3d ee_goal,
                    Eigen::VectorXd &ik_result);

bool sampleArmPose2(
    vkc::VKCEnvBasic &env, std::string target_joint, double target_value,
    Eigen::VectorXd &ik_result,
    vkc::BaseObject::AttachLocation::ConstPtr attach_location_ptr,
    int remaining_steps);

void ik2csv(const std::vector<std::string> &joint_names,
            const std::vector<Eigen::VectorXd> &joint_states,
            const std::string &file_path);

void csv2ik(std::vector<std::string> &joint_names,
            std::vector<Eigen::VectorXd> &joint_states,
            const std::string &file_path);

std::string getCurrentTime(std::string time_format = "%T");

#endif
