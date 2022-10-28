#ifndef VKC_UTILS_H
#define VKC_UTILS_h

#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_environment/environment.h>

tesseract_planning::CompositeInstruction generateTrajectorySeed(
    tesseract_planning::CompositeInstruction& program,
    std::vector<std::string> joint_names,
    std::vector<Eigen::VectorXd> joint_states,
    tesseract_environment::Environment::Ptr env);

#endif