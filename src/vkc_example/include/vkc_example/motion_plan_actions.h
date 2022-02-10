#ifndef VKC_MOTION_PLAN_ACTIONS
#define VKC_MOTION_PLAN_ACTIONS
#include "vkc/action/actions.h"

/**
 * @brief implement robot big task case 1.1, 
 *        move cup3 into reachable place and then put them into cabinet one by one 
 * @param actions output parameter, which carries the generated actions for motion planning
 * @param robot input parameter, which specifies the robot who will act the actions 
 * @return
 */
void GenerateGraspCup3WithoutToolActions(vkc::ActionSeq &actions, const std::string &robot);
  /**
   * @brief implement robot big task case 1.2, 
   *        move cup3 into reachable place and then put them into cabinet one by one 
   * @param actions output parameter, which carries the generated actions for motion planning
   * @param robot input parameter, which specifies the robot who will act the actions 
   * @return
   */
void GenerateActionsUnefficiency(vkc::ActionSeq &actions, const std::string &robot);  

/**
 * @brief implement robot big task case 1.3, 
 *        move cups and plate from the one nearest to the cabinet into the cabinet 
 * @param actions output parameter, which carries the generated actions for motion planning
 * @param robot input parameter, which specifies the robot who will act the actions 
 * @return
 */
void GenerateOderedActionsByDistance(vkc::ActionSeq &actions, const std::string &robot);
/**
 * @brief implement robot big task case 2.1, 
 *        grasp Cup_4 after moving cups and plate into the cabinet, as Cup_4 is unreachable,
 *        the robot grasp it by the plate  
 * @param actions output parameter, which carries the generated actions for motion planning
 * @param robot input parameter, which specifies the robot who will act the actions 
 * @return
 */
void GenerateActionsGraspCup4WithPlate(vkc::ActionSeq &actions, const std::string &robot);

/**
 * @brief implement robot big task case 1.2, 
 *        move cup3 into reachable place and then put them into cabinet one by one 
 * @param actions output parameter, which carries the generated actions for motion planning
 * @param robot input parameter, which specifies the robot who will act the actions 
 * @return
 */
void GenerateRobotBigTaskActions(vkc::ActionSeq &actions, const std::string &robot);
/**
 * @brief implement robot big task case 1.2, 
 *        move cup3 into reachable place and then put them into cabinet one by one 
 * @param actions output parameter, which carries the generated actions for motion planning
 * @param robot input parameter, which specifies the robot who will act the actions 
 * @return
 */
void GenerateRobotBigTaskActionsDisableBaseRotation(vkc::ActionSeq &actions, const std::string &robot);
#endif