#include <vkc/utils.h>

using namespace tesseract_planning;

CompositeInstruction generateTrajectorySeed(
    CompositeInstruction& program, std::vector<std::string> joint_names,
    std::vector<Eigen::VectorXd> joint_states,
    tesseract_environment::Environment::Ptr env) {
  if (!program.hasStartInstruction())
    throw std::runtime_error(
        "Top most composite instruction is missing start instruction!");
  CompositeInstruction seed = program;
  seed.clear();

  const tesseract_common::ManipulatorInfo& mi = program.getManipulatorInfo();
  MoveInstructionPoly base_instruction = seed.getStartInstruction();
  tesseract_common::ManipulatorInfo base_mi =
      base_instruction.getManipulatorInfo();
  tesseract_common::ManipulatorInfo start_mi = mi.getCombined(base_mi);
  std::unordered_map<std::string, std::vector<std::string>> manip_joint_names;
  manip_joint_names[start_mi.manipulator] = joint_names;

  for (auto& i : program) {
    if (i.isCompositeInstruction()) {
      throw std::runtime_error(
          "no nested composite instruction should be provided for trajectory "
          "seed");
    }
    auto& base_instruction = i.as<MoveInstructionPoly>();
    tesseract_common::ManipulatorInfo mi =
        mi.getCombined(base_instruction.getManipulatorInfo());

    CompositeInstruction ci;
    ci.setProfile(base_instruction.getProfile());
    ci.setDescription(base_instruction.getDescription());
    ci.setManipulatorInfo(base_instruction.getManipulatorInfo());
    ci.setProfileOverrides(base_instruction.getProfileOverrides());
    for (int j = 0; j < joint_states.size(); j++) {
      const auto& js = joint_states[j];
      MoveInstructionPoly move_instruction = base_instruction.createChild();
      StateWaypointPoly swp = move_instruction.createStateWaypoint();
      swp.setNames(joint_names);
      swp.setPosition(js);
      move_instruction.assignStateWaypoint(swp);
      if (j != joint_states.size()) {
        move_instruction.setProfile(base_instruction.getPathProfile());
        move_instruction.setProfileOverrides(
            base_instruction.getPathProfileOverrides());
        move_instruction.setPathProfile(base_instruction.getPathProfile());
        move_instruction.setPathProfileOverrides(
            base_instruction.getPathProfileOverrides());
      }
      ci.appendMoveInstruction(move_instruction);
    }
    seed.appendInstruction(ci);
  }
  return seed;
}