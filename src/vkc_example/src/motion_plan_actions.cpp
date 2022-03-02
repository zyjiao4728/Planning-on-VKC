#include "vkc_example/motion_plan_actions.h"

#include <memory>
using namespace std;
using namespace vkc;

/**
 * @brief implement robot big task case 1.1,
 *        move cup3 into reachable place and then put them into Cabinet_417 one by one
 * @param actions output parameter, which carries the generated actions for motion planning
 * @param robot input parameter, which specifies the robot who will act the actions
 * @return
 */
void GenerateGraspCup3WithoutToolActions(vkc::ActionSeq &actions, const std::string &robot)
{
    Eigen::Isometry3d dst_tf;
    PlaceAction::Ptr place_action;
    const bool enable_init_traj = true;

    /** move away chair which blocks the robot to pick something **/
    // action 1: pick the Cup_3
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cup_3_front"));
        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 2: place Cup_3 onto Table_5
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        dst_tf.translation() = Eigen::Vector3d(-0.338684, 2.24349, 0.81);
        dst_tf.linear() = Eigen::Quaterniond(0.707107, 0.707107, 0.0, 0.0).matrix();

        link_objectives.emplace_back("Cup_3_link", dst_tf);
        place_action = make_shared<PlaceAction>(robot, "attach_cup_3_front", link_objectives, joint_objectives, false);
        place_action->setNewAttachObject("Table_5_link");

        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }

    /** move away chair which blocks the robot to pick something **/
    // action 3: pick the Cup_3
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cup_3_left"));
        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 4: place Cup_3 onto Table_5
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        dst_tf.translation() = Eigen::Vector3d(-0.338684, 2.24349, 0.81);
        dst_tf.linear() = Eigen::Quaterniond(0.707107, 0.707107, 0.0, 0.0).matrix();

        link_objectives.emplace_back("Cup_3_link", dst_tf);
        place_action = make_shared<PlaceAction>(robot, "attach_cup_3_left", link_objectives, joint_objectives, false);
        place_action->setNewAttachObject("Table_5_link");

        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }

    /** move away chair which blocks the robot to pick something **/
    // action 5: pick the Cup_1
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cup_3_right"));
        (*actions.rbegin())->RequireInitTraj(true);
    }
}
/**
 * @brief implement robot big task case 1.2,
 *        move cup3 into reachable place and then put them into Cabinet_417 one by one
 * @param actions output parameter, which carries the generated actions for motion planning
 * @param robot input parameter, which specifies the robot who will act the actions
 * @return
 */
void GenerateActionsUnefficiency(vkc::ActionSeq &actions, const std::string &robot)
{
    // TODO
    Eigen::Isometry3d dst_tf;
    PlaceAction::Ptr place_action;
    const bool enable_init_traj = true;

    const double x_offset = 0.0;
    const double y_offset = 0.0;
    const double z_offset = 0.0;

    /** pick up Plate_1 and use it to pick up Cup_1 **/
    // action 1: pick Plate_1
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_plate_1"));
        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 2: pick up Cup_3
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cup_3_with_tool"));
        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 3: place Cup_3 onto Table_5
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        dst_tf.translation() = Eigen::Vector3d(-0.74, 1.85, 0.81);
        dst_tf.linear() = Eigen::Quaterniond(0.707107, 0.707107, 0.0, 0.0).matrix();

        link_objectives.emplace_back("Cup_3_link", dst_tf);
        place_action = make_shared<PlaceAction>(robot, "attach_cup_3_with_tool", link_objectives, joint_objectives, false);
        place_action->setNewAttachObject("Table_5_link");
        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }

    // action 4: place Plate_1 on Table_5
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        dst_tf.translation() = Eigen::Vector3d(-0.338684, 2.15149, 0.87);
        // dst_tf.linear() = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0).matrix();   // Plate_1's bottow is upright
        dst_tf.linear() = Eigen::Quaterniond(0.707107, 0.707107, 0.0, 0.0).matrix(); // ok

        link_objectives.emplace_back("Plate_1_link", dst_tf);
        place_action = make_shared<PlaceAction>(robot, "attach_plate_1", link_objectives, joint_objectives, false);
        place_action->setNewAttachObject("Table_5_link");

        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }

    /** open Cabinet_417 and then place Cup_4 into it **/
    // action 5: pick the door of Cabinet_417
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cabinet_417_door"));
        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 6: open door of Cabinet_417
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        joint_objectives.emplace_back("Cabinet_417_link_dof_rootd_Bb001_r_joint", -2.1);
        place_action = make_shared<PlaceAction>(robot, "attach_cabinet_417_door", link_objectives, joint_objectives, false);
        place_action->setOperationObjectType(false);

        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }

    /** pick up Cup_1 and place it into Cabinet_417 **/
    // action 1: pick up Cup_1
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cup_1_body"));
        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 2: place Cup_1 into Cabinet_417
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        dst_tf.translation() = Eigen::Vector3d(1.6, 1.1, 1.15);
        dst_tf.linear() = Eigen::Quaterniond(0.5, 0.5, -0.50, -0.50).matrix();

        link_objectives.emplace_back("Cup_1_link", dst_tf);
        place_action = make_shared<PlaceAction>(robot, "attach_cup_1_body", link_objectives, joint_objectives, false);
        place_action->setNewAttachObject("Cabinet_417_link");

        place_action->RequireInitTraj(true);
        actions.emplace_back(place_action);
    }

    /** pick up Cup_2 and place it into Cabinet_417 **/
    // action 3: pick up Cup_2
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cup_2_body"));
        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 4: place Cup_2 into Cabinet_417
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        dst_tf.translation() = Eigen::Vector3d(1.7, 0.9, 1.15);
        dst_tf.linear() = Eigen::Quaterniond(0.5, 0.5, -0.50, -0.50).matrix();
        // dst_tf.linear() = Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5).matrix();

        link_objectives.emplace_back("Cup_2_link", dst_tf);
        place_action = make_shared<PlaceAction>(robot, "attach_cup_2_body", link_objectives, joint_objectives, false);
        place_action->setNewAttachObject("Cabinet_417_link");

        place_action->RequireInitTraj(true);
        actions.emplace_back(place_action);
    }

    // action 12: pick Plate_1
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_plate_1"));
        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 13: place Plate_1 into Cabinet_417
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        dst_tf.translation() = Eigen::Vector3d(2.0, 0.9, 0.65);
        dst_tf.linear() = Eigen::Quaterniond(0.5, 0.5, -0.50, -0.50).matrix(); // ok

        link_objectives.emplace_back("Plate_1_link", dst_tf);
        place_action = make_shared<PlaceAction>(robot, "attach_plate_1", link_objectives, joint_objectives, false);
        place_action->setNewAttachObject("Cabinet_417_link");

        place_action->RequireInitTraj(true);
        actions.emplace_back(place_action);
    }

    // action 2: pick up Cup_3
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cup_3_body"));
        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 3: place Cup_3 into Cabinet_417
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        dst_tf.translation() = Eigen::Vector3d(1.6, 0.7, 1.15);
        dst_tf.linear() = Eigen::Quaterniond(0.5, 0.5, -0.50, -0.50).matrix();

        link_objectives.emplace_back("Cup_3_link", dst_tf);
        place_action = make_shared<PlaceAction>(robot, "attach_cup_3_body", link_objectives, joint_objectives, false);
        place_action->setNewAttachObject("Cabinet_417_link");

        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }

    /** close Cabinet_417 after placing Plate_1 into it **/
    // action 22: pick the door of Cabinet_417
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cabinet_417_door"));
        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 23: close door of Cabinet_417
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        joint_objectives.emplace_back("Cabinet_417_link_dof_rootd_Bb001_r_joint", 0.0);
        place_action = make_shared<PlaceAction>(robot, "attach_cabinet_417_door", link_objectives, joint_objectives, false);
        place_action->RequireInitTraj(false);

        place_action->setOperationObjectType(false);
        actions.emplace_back(place_action);
    }
}

/**
 * @brief implement robot big task case 1.3,
 *        move cups and plate from the one nearest to the Cabinet_417 into the Cabinet_417
 * @param actions output parameter, which carries the generated actions for motion planning
 * @param robot input parameter, which specifies the robot who will act the actions
 * @return
 */
void GenerateOderedActionsByDistance(vkc::ActionSeq &actions, const std::string &robot)
{
    // TODO
    Eigen::Isometry3d dst_tf;
    PlaceAction::Ptr place_action;
    const bool enable_init_traj = true;

    /** open Cabinet_417 and then place Cup_4 into it **/
    // action 1: pick the door of Cabinet_417
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cabinet_417_door"));
        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 2: open door of Cabinet_417
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        joint_objectives.emplace_back("Cabinet_417_link_dof_rootd_Bb001_r_joint", -2.1);
        place_action = make_shared<PlaceAction>(robot, "attach_cabinet_417_door", link_objectives, joint_objectives, false);
        place_action->setOperationObjectType(false);

        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }

    /** pick up Cup_1 and place it into Cabinet_417 **/
    // action 3: pick up Cup_1
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cup_1"));

        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 4: place Cup_1 into Cabinet_417
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        dst_tf.translation() = Eigen::Vector3d(1.8, 1.5, 1.15);
        dst_tf.linear() = Eigen::Quaterniond(0.5, 0.5, -0.50, -0.50).matrix();

        link_objectives.emplace_back("Cup_1_link", dst_tf);
        place_action = make_shared<PlaceAction>(robot, "attach_cup_1", link_objectives, joint_objectives, false);
        place_action->setNewAttachObject("Cabinet_417_link");

        place_action->RequireInitTraj(true);
        actions.emplace_back(place_action);
    }

    /** pick up Cup_2 and place it into Cabinet_417 **/
    // action 5: pick up Cup_2
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cup_2"));

        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 6: place Cup_2 into Cabinet_417
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        dst_tf.translation() = Eigen::Vector3d(1.8, 1.3, 1.15);
        dst_tf.linear() = Eigen::Quaterniond(0.5, 0.5, -0.50, -0.50).matrix();

        link_objectives.emplace_back("Cup_2_link", dst_tf);
        place_action = make_shared<PlaceAction>(robot, "attach_cup_2", link_objectives, joint_objectives, false);
        place_action->setNewAttachObject("Cabinet_417_link");

        place_action->RequireInitTraj(true);
        actions.emplace_back(place_action);
    }

    /** pick up Plate_1 and place it into Cabinet_417 **/
    // action 7: pick Plate_1
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_plate_1"));

        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 8: place Plate_1 into Cabinet_417
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        dst_tf.translation() = Eigen::Vector3d(2.0, 1.2, 0.65);
        dst_tf.linear() = Eigen::Quaterniond(0.5, 0.5, -0.50, -0.50).matrix(); // ok

        link_objectives.emplace_back("Plate_1_link", dst_tf);
        place_action = make_shared<PlaceAction>(robot, "attach_plate_1", link_objectives, joint_objectives, false);
        place_action->setNewAttachObject("Cabinet_417_link");

        place_action->RequireInitTraj(true);
        actions.emplace_back(place_action);
    }


    /** close Cabinet_417 after placing Plate_1 into it **/
    // action 9: pick the door of Cabinet_417
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cabinet_417_door"));
        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 10: close door of Cabinet_417
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        joint_objectives.emplace_back("Cabinet_417_link_dof_rootd_Bb001_r_joint", 0.0);
        place_action = make_shared<PlaceAction>(robot, "attach_cabinet_417_door", link_objectives, joint_objectives, false);
        place_action->RequireInitTraj(false);

        place_action->setOperationObjectType(false);
        actions.emplace_back(place_action);
    }
}

/**
 * @brief implement robot big task case 2.1,
 *        grasp Cup_4 after moving cups and plate into the Cabinet_417, as Cup_4 is unreachable,
 *        the robot grasp it by the Plate_1
 * @param actions output parameter, which carries the generated actions for motion planning
 * @param robot input parameter, which specifies the robot who will act the actions
 * @return
 */
void GenerateActionsGraspCup4WithPlate(vkc::ActionSeq &actions, const std::string &robot)
{
    const bool enable_init_traj = true;
    // TODO
    Eigen::Isometry3d dst_tf;
    PlaceAction::Ptr place_action;
    const double x_offset = 0.0;
    const double y_offset = 0.0;
    const double z_offset = 0.0;

    /** pick up Plate_1 and use it to pick up Cup_1 **/
    // action 1: pick Plate_1
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_plate_1"));
        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 2: pick up Cup_3
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cup_3_with_tool"));

        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 3: place Cup_3 onto Table_5
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        dst_tf.translation() = Eigen::Vector3d(-0.74, 1.85, 0.81);
        dst_tf.linear() = Eigen::Quaterniond(0.707107, 0.707107, 0.0, 0.0).matrix();

        link_objectives.emplace_back("Cup_3_link", dst_tf);
        place_action = make_shared<PlaceAction>(robot, "attach_cup_3_with_tool", link_objectives, joint_objectives, false);
        place_action->setNewAttachObject("Table_5_link");

        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }

    // action 4: place Plate_1 on Table_5
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        dst_tf.translation() = Eigen::Vector3d(-0.338684, 2.15149, 0.87);
        // dst_tf.linear() = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0).matrix();   // Plate_1's bottow is upright
        dst_tf.linear() = Eigen::Quaterniond(0.707107, 0.707107, 0.0, 0.0).matrix(); // ok

        link_objectives.emplace_back("Plate_1_link", dst_tf);
        place_action = make_shared<PlaceAction>(robot, "attach_plate_1", link_objectives, joint_objectives, false);
        place_action->setNewAttachObject("Table_5_link");

        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }

    // action 5: pick up Cup_3
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cup_3"));
        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 6: place Cup_3 onto Plate_1
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        dst_tf.translation() = Eigen::Vector3d(-0.338684, 2.03, 0.88);
        dst_tf.linear() = Eigen::Quaterniond(0.707107, 0.707107, 0.0, 0.0).matrix();

        link_objectives.emplace_back("Cup_3_link", dst_tf);
        place_action = make_shared<PlaceAction>(robot, "attach_cup_3", link_objectives, joint_objectives, false);
        place_action->setNewAttachObject("Plate_1_link");

        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }

    /** pick up Cup_1 and place it onto Plate_1 **/
    // action 7: pick up Cup_1
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cup_1"));
        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 8: place Cup_1 onto Plate_1
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        dst_tf.translation() = Eigen::Vector3d(-0.408684, 1.86, 0.88);
        dst_tf.linear() = Eigen::Quaterniond(0.707107, 0.707107, 0.0, 0.0).matrix();

        link_objectives.emplace_back("Cup_1_link", dst_tf);
        place_action = make_shared<PlaceAction>(robot, "attach_cup_1", link_objectives, joint_objectives, false);
        place_action->setNewAttachObject("Plate_1_link");
        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }

    /** pick up Cup_2 and place it onto Plate_1 **/
    // action 9: pick up Cup_2
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cup_2"));
        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 10: place Cup_2 onto Plate_1
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        dst_tf.translation() = Eigen::Vector3d(-0.26, 1.90, 0.88);
        dst_tf.linear() = Eigen::Quaterniond(0.612, 0.612, 0.354, 0.354).matrix();
        // dst_tf.linear() = Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5).matrix();

        link_objectives.emplace_back("Cup_2_link", dst_tf);
        place_action = make_shared<PlaceAction>(robot, "attach_cup_2", link_objectives, joint_objectives, false);
        place_action->setNewAttachObject("Plate_1_link");

        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }

    /** open Cabinet_417 and then place Plate_1 into it **/
    // action 11: pick the door of Cabinet_417
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cabinet_417_door"));

        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 12: open door of Cabinet_417
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        joint_objectives.emplace_back("Cabinet_417_link_dof_rootd_Bb001_r_joint", -2.2);
        place_action = make_shared<PlaceAction>(robot, "attach_cabinet_417_door", link_objectives, joint_objectives, false);
        place_action->RequireInitTraj(true);

        place_action->setOperationObjectType(false);
        actions.emplace_back(place_action);
    }

    // action 13: pick Plate_1
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_plate_1"));
        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 14: place Plate_1 and things on it into Cabinet_417
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        dst_tf.translation() = Eigen::Vector3d(2.0, 0.9, 1.15);
        dst_tf.linear() = Eigen::Quaterniond(0.50, 0.50, -0.50, -0.50).matrix();
        // dst_tf.linear() = Eigen::Quaterniond(0.707107, 0.707107, 0.0, 0.0).matrix();   // ok

        link_objectives.emplace_back("Plate_1_link", dst_tf);
        place_action = make_shared<PlaceAction>(robot, "attach_plate_1", link_objectives, joint_objectives, false);
        place_action->setNewAttachObject("Cabinet_417_link");

        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }

    /** close Cabinet_417 after placing Plate_1 into it **/
    // action 15: pick the door handle of Cabinet_417
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cabinet_417_door"));

        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 16: close door of Cabinet_417
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        joint_objectives.emplace_back("Cabinet_417_link_dof_rootd_Bb001_r_joint", 0.0);
        place_action = make_shared<PlaceAction>(robot, "attach_cabinet_417_door", link_objectives, joint_objectives, false);
        place_action->setOperationObjectType(false);

        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }

    // // action 17: goto pick the Cup_4
    // {
    //     std::vector<LinkDesiredPose> link_objectives;
    //     std::vector<JointDesiredPose> joint_objectives;

    //     dst_tf.translation() = Eigen::Vector3d(-0.738684, 2.67349, 0.950001);
    //     dst_tf.linear() = Eigen::Quaterniond(0.707107, -0.707107, 0.0, 0.0).matrix(); // ok

    //     link_objectives.emplace_back("right_gripper_palm", dst_tf);
    //     actions.emplace_back(make_shared<GotoAction>(robot, link_objectives, joint_objectives));
    // }

    // action 17: pick up Cup_4
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cup_4_body"));

        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 17.2: place Cup_4 onto Table_5
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        dst_tf.translation() = Eigen::Vector3d(-0.738684, 2.67349, 0.810001);
        dst_tf.linear() = Eigen::Quaterniond(0.707107, 0.707107, 0.0, 0.0).matrix();

        link_objectives.emplace_back("Cup_4_link", dst_tf);
        place_action = make_shared<PlaceAction>(robot, "attach_cup_4_body", link_objectives, joint_objectives, false);
        place_action->setNewAttachObject("world");

        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }

    /** move away chair which blocks the robot to pick something **/
    // action 18: pick the door handle of Cabinet_417
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cabinet_417_door"));
        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 19: open door of Cabinet_417
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        joint_objectives.emplace_back("Cabinet_417_link_dof_rootd_Bb001_r_joint", -2.2);
        place_action = make_shared<PlaceAction>(robot, "attach_cabinet_417_door", link_objectives, joint_objectives, false);
        place_action->setOperationObjectType(false);

        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }

    /** pick up Cup_1 and place it into Cabinet_417 **/
    // action 20: pick up Cup_1
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cup_1_body"));
        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 21: place Cup_1 into Cabinet_417
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        dst_tf.translation() = Eigen::Vector3d(1.6, 1.1, 0.65);
        dst_tf.linear() = Eigen::Quaterniond(0.5, 0.5, -0.50, -0.50).matrix();

        link_objectives.emplace_back("Cup_1_link", dst_tf);
        place_action = make_shared<PlaceAction>(robot, "attach_cup_1_body", link_objectives, joint_objectives, false);
        place_action->setNewAttachObject("Cabinet_417_link");
        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }

    /** pick up Cup_2 and place it onto Cabinet_417 **/
    // action 24: pick up Cup_2
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cup_2_body_rotated"));
        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 25: place Cup_2 into Cabinet_417
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        dst_tf.translation() = Eigen::Vector3d(1.6, 0.9, 0.65);
        dst_tf.linear() = Eigen::Quaterniond(0.683, 0.683, -0.183, -0.183).matrix();

        link_objectives.emplace_back("Cup_2_link", dst_tf);
        place_action = make_shared<PlaceAction>(robot, "attach_cup_2_body_rotated", link_objectives, joint_objectives, false);
        place_action->setNewAttachObject("Cabinet_417_link");

        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }

    // action 22: pick up Cup_3
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cup_3_body"));

        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 23: place Cup_3 into Cabinet_417
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        dst_tf.translation() = Eigen::Vector3d(1.6, 0.7, 0.65);
        dst_tf.linear() = Eigen::Quaterniond(0.5, 0.5, -0.50, -0.50).matrix();

        link_objectives.emplace_back("Cup_3_link", dst_tf);
        place_action = make_shared<PlaceAction>(robot, "attach_cup_3_body", link_objectives, joint_objectives, false);
        place_action->setNewAttachObject("Cabinet_417_link");

        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }

    // action 26: pick the Plate_1
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_plate_1"));

        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 27:  goto pick the Cup_4 with plate
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cup_4_with_tool"));
        (*actions.rbegin())->RequireInitTraj(true);
    }
}

/**
 * @brief implement robot big task case 2.1,
 *        move cup3 into reachable place and then put them into Cabinet_417 one by one
 * @param actions output parameter, which carries the generated actions for motion planning
 * @param robot input parameter, which specifies the robot who will act the actions
 * @return
 */
void GenerateRobotBigTaskActions(vkc::ActionSeq &actions, const std::string &robot)
{
    const bool enable_init_traj = true;
    // TODO
    Eigen::Isometry3d dst_tf;
    PlaceAction::Ptr place_action;
    const double x_offset = 0.0;
    const double y_offset = 0.0;
    const double z_offset = 0.0;

    /** pick up Plate_1 and use it to pick up Cup_1 **/
    // action 1: pick Plate_1
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_plate_1"));

        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 2: pick up Cup_3
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cup_3_with_tool"));

        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 3: place Cup_3 onto Table_5
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        dst_tf.translation() = Eigen::Vector3d(-0.74, 1.85, 0.81);
        dst_tf.linear() = Eigen::Quaterniond(0.707107, 0.707107, 0.0, 0.0).matrix();

        link_objectives.emplace_back("Cup_3_link", dst_tf);
        place_action = make_shared<PlaceAction>(robot, "attach_cup_3_with_tool", link_objectives, joint_objectives, false);
        place_action->setNewAttachObject("Table_5_link");

        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }

    // action 4: place Plate_1 on Table_5
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        dst_tf.translation() = Eigen::Vector3d(-0.338684, 2.15149, 0.87);
        // dst_tf.linear() = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0).matrix();   // Plate_1's bottow is upright
        dst_tf.linear() = Eigen::Quaterniond(0.707107, 0.707107, 0.0, 0.0).matrix(); // ok

        link_objectives.emplace_back("Plate_1_link", dst_tf);
        place_action = make_shared<PlaceAction>(robot, "attach_plate_1", link_objectives, joint_objectives, false);
        place_action->setNewAttachObject("Table_5_link");

        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }

    // action 5: pick up Cup_3
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cup_3"));

        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 6: place Cup_3 onto Plate_1
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        dst_tf.translation() = Eigen::Vector3d(-0.338684, 2.03, 0.88);
        dst_tf.linear() = Eigen::Quaterniond(0.707107, 0.707107, 0.0, 0.0).matrix();

        link_objectives.emplace_back("Cup_3_link", dst_tf);
        place_action = make_shared<PlaceAction>(robot, "attach_cup_3", link_objectives, joint_objectives, false);
        place_action->setNewAttachObject("Plate_1_link");

        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }

    /** pick up Cup_1 and place it onto Plate_1 **/
    // action 7: pick up Cup_1
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cup_1"));
        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 8: place Cup_1 onto Plate_1
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        dst_tf.translation() = Eigen::Vector3d(-0.408684, 1.86, 0.88);
        dst_tf.linear() = Eigen::Quaterniond(0.707107, 0.707107, 0.0, 0.0).matrix();

        link_objectives.emplace_back("Cup_1_link", dst_tf);
        place_action = make_shared<PlaceAction>(robot, "attach_cup_1", link_objectives, joint_objectives, false);
        place_action->setNewAttachObject("Plate_1_link");
        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }

    /** pick up Cup_2 and place it onto Plate_1 **/
    // action 9: pick up Cup_2
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cup_2"));
        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 10: place Cup_2 onto Plate_1
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        dst_tf.translation() = Eigen::Vector3d(-0.26, 1.90, 0.88);
        dst_tf.linear() = Eigen::Quaterniond(0.612, 0.612, 0.354, 0.354).matrix();
        // dst_tf.linear() = Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5).matrix();

        link_objectives.emplace_back("Cup_2_link", dst_tf);
        place_action = make_shared<PlaceAction>(robot, "attach_cup_2", link_objectives, joint_objectives, false);
        place_action->setNewAttachObject("Plate_1_link");

        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }

    /** open Cabinet_417 and then place Plate_1 into it **/
    // action 11: pick the door of Cabinet_417
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cabinet_417_door"));

        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 12: open door of Cabinet_417
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        joint_objectives.emplace_back("Cabinet_417_link_dof_rootd_Bb001_r_joint", -2.2);
        place_action = make_shared<PlaceAction>(robot, "attach_cabinet_417_door", link_objectives, joint_objectives, false);

        place_action->RequireInitTraj(true);

        place_action->setOperationObjectType(false);
        actions.emplace_back(place_action);
    }

    // action 13: pick Plate_1
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_plate_1"));
        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 14: place Plate_1 and things on it into Cabinet_417
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        dst_tf.translation() = Eigen::Vector3d(2.0, 0.9, 1.15);
        dst_tf.linear() = Eigen::Quaterniond(0.50, 0.50, -0.50, -0.50).matrix();
        // dst_tf.linear() = Eigen::Quaterniond(0.707107, 0.707107, 0.0, 0.0).matrix();   // ok

        link_objectives.emplace_back("Plate_1_link", dst_tf);
        place_action = make_shared<PlaceAction>(robot, "attach_plate_1", link_objectives, joint_objectives, false);
        place_action->setNewAttachObject("Cabinet_417_link");

        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }

    /** close Cabinet_417 after placing Plate_1 into it **/
    // action 15: pick the door handle of Cabinet_417
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cabinet_417_door"));
        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 16: close door of Cabinet_417
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        joint_objectives.emplace_back("Cabinet_417_link_dof_rootd_Bb001_r_joint", 0.0);
        place_action = make_shared<PlaceAction>(robot, "attach_cabinet_417_door", link_objectives, joint_objectives, false);
        place_action->setOperationObjectType(false);

        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }

    // // action 17: goto pick the Chair_645
    // {
    //     std::vector<LinkDesiredPose> link_objectives;
    //     std::vector<JointDesiredPose> joint_objectives;

    //     dst_tf.translation() = Eigen::Vector3d(-0.738684, 2.67349, 0.950001);
    //     dst_tf.linear() = Eigen::Quaterniond(0.707107, -0.707107, 0.0, 0.0).matrix(); // ok

    //     link_objectives.emplace_back("right_gripper_palm", dst_tf);
    //     actions.emplace_back(make_shared<GotoAction>(robot, link_objectives, joint_objectives));
    // }

    /** move away Chair_645 to free the way for grasping Cup_4 **/
    // action 17: pick the Chair_645
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_chair_645"));
        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 18: place chair_645 to a free place
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        dst_tf.translation() = Eigen::Vector3d(-1.0, -0.5, 0.555);
        dst_tf.linear() = Eigen::Quaterniond(0.0, 0, 0.0, -1.00).matrix(); // 0.0 0.0 90
        // dst_tf.linear() = Eigen::Quaterniond(0, 0, 0.0, 1.0).matrix();

        link_objectives.emplace_back("Chair_645_link", dst_tf);
        place_action = make_shared<PlaceAction>(robot, "attach_chair_645", link_objectives, joint_objectives, false);
        place_action->setNewAttachObject("world");

        place_action->RequireInitTraj(true);
        actions.emplace_back(place_action);
    }

    /** open cabinet_417 and then place cup_4 into it **/
    // action 19: pick the door of cabinet_417
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cabinet_417_door"));
        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 20: open door of cabinet_417
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        joint_objectives.emplace_back("Cabinet_417_link_dof_rootd_Bb001_r_joint", -2.2);
        place_action = make_shared<PlaceAction>(robot, "attach_cabinet_417_door", link_objectives, joint_objectives, false);
        place_action->setOperationObjectType(false);


        place_action->RequireInitTraj(false);
        actions.emplace_back(place_action);
    }

    // action 21: pick the cup_4
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cup_4_body_rotated"));

        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 22: place cup_4 to a free place of cabinet_417
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        dst_tf.translation() = Eigen::Vector3d(1.6, 0.9, 0.65);
        dst_tf.linear() = Eigen::Quaterniond(0.653, 0.653, -0.271, -0.271).matrix();

        link_objectives.emplace_back("Cup_4_link", dst_tf);
        place_action = make_shared<PlaceAction>(robot, "attach_cup_4_body_rotated", link_objectives, joint_objectives, false);
        place_action->setNewAttachObject("Cabinet_417_link");

        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }

    /** close door of cabinet_417 after placing cup_4 into it **/
    // action 23: pick the door of cabinet_417
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cabinet_417_door"));

        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 24: close door of cabinet_417
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        joint_objectives.emplace_back("Cabinet_417_link_dof_rootd_Bb001_r_joint", 0.0);
        place_action = make_shared<PlaceAction>(robot, "attach_cabinet_417_door", link_objectives, joint_objectives, false);
        place_action->setOperationObjectType(false);

        place_action->RequireInitTraj(false);

        actions.emplace_back(place_action);
    }

    /** move chair back to its initial location **/
    // action 25: pick the chair
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_chair_645"));
        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 26: place chair_645 back
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        dst_tf.translation() = Eigen::Vector3d(-1.4, 2.3, 0.55);
        dst_tf.linear() = Eigen::Quaterniond(0.707, 0, 0.0, 0.707).matrix();

        link_objectives.emplace_back("Chair_645_link", dst_tf);
        place_action = make_shared<PlaceAction>(robot, "attach_chair_645", link_objectives, joint_objectives, false);
        place_action->setNewAttachObject("world");
        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }
}

/**
 * @brief implement robot big task case 1.2,
 *        move cup3 into reachable place and then put them into Cabinet_417 one by one
 * @param actions output parameter, which carries the generated actions for motion planning
 * @param robot input parameter, which specifies the robot who will act the actions
 * @return
 */
void GenerateRobotBigTaskActionsDisableBaseRotation(vkc::ActionSeq &actions, const std::string &robot)
{
}