#!/usr/bin/env python
import rospy
import tf
from sensor_msgs.msg import JointState
import csv
import time

JOINT_NAMES = [
    "base_y_base_x",    
    "base_theta_base_y",    
    "base_link_base_theta",
    "ur_arm_shoulder_pan_joint",
    "ur_arm_shoulder_lift_joint",
    "ur_arm_elbow_joint",
    "ur_arm_wrist_1_joint",
    "ur_arm_wrist_2_joint",
    "ur_arm_wrist_3_joint",
    "fridge_0001_dof_rootd_Aa002_r_joint", # fridge
    "dishwasher_12065_joint_0",
    "joint_1",
    "cabinet_45290_1_joint_0",
    "cabinet_45290_2_joint_0",
    "dishwasher_12065_joint_0",
    "dishwasher_12065_joint_1",
    "dishwasher_12065_joint_2",
    "dishwasher_12065_joint_3",
    "door_8966_joint_1",
    "door_8966_joint_2",
    "fridge_0001_dof_rootd_Aa001_r_joint",
    "fridge_0001_dof_rootd_Ba001_t_joint",
    "fridge_0001_dof_rootd_Ba002_t_joint",
    "fridge_0001_dof_rootd_Ba003_t_joint",
    "front_left_wheel",
    "front_right_wheel",
    "rear_left_wheel",
    "rear_right_wheel",
    "joint_0",
    "joint_2",
    "oven_101917_joint_0",
    "oven_101917_joint_1",
    "oven_101917_joint_2",
    "oven_101917_joint_3",
    "oven_101917_joint_4",
    "oven_101917_joint_5",
    "oven_101917_joint_6",
    "oven_101917_joint_7",
    "robotiq_85_left_finger_tip_joint",
    "robotiq_85_left_inner_knuckle_joint",
    "robotiq_85_left_knuckle_joint",
    "robotiq_85_right_finger_tip_joint",
    "robotiq_85_right_inner_knuckle_joint",
    "robotiq_85_right_knuckle_joint",
]   

class SceneController(object):

    def __init__(self):
        self.br_ = tf.TransformBroadcaster()
        self.joint_pub_ = rospy.Publisher("joint_states", JointState, queue_size=1)
        
        self.joint_state_ = JointState()

        for name in JOINT_NAMES:
            self.joint_state_.name.append(name)
            if name == "dishwasher_12065_joint_0":
                self.joint_state_.position.append(-0.3080)
            elif name == "dishwasher_12065_joint_1":
                self.joint_state_.position.append(-0.1359)
            elif name == "dishwasher_12065_joint_2":
                self.joint_state_.position.append(-0.6457)
            else:
                self.joint_state_.position.append(0.0)
        
        # keep poblishing joint state until other component ready
        rate = rospy.Rate(20)
        for _ in range(100):
            self.joint_state_.header.stamp = rospy.Time.now()
            self.joint_pub_.publish(self.joint_state_)
            rate.sleep()

    def open_fridge_door_1(self):
        rate = rospy.Rate(20)

        with open('/home/jiao/vkc_ws/vkc-demo-traj/open_fridge_1.csv') as f:
            reader = csv.reader(f)
            position = list(reader)
        
        time.sleep(2)
        
        for i in range(0, len(position)):
            self.joint_state_.header.stamp = rospy.Time.now()
            self.joint_state_.position[0:9] = map(float, position[i][0:9])
            if len(position[i]) > 9:
                self.joint_state_.position[9] = -float(position[i][9].replace('\U00002013', '-'))
            else:
                self.joint_state_.position[9] = 0.0
            self.joint_pub_.publish(self.joint_state_)
            rate.sleep()
    
    def open_fridge_door_2(self):
        rate = rospy.Rate(20)

        with open('/home/jiao/vkc_ws/vkc-demo-traj/open_fridge_2.csv') as f:
            reader = csv.reader(f)
            position = list(reader)
        
        time.sleep(2)
        
        for i in range(0, len(position)):
            self.joint_state_.header.stamp = rospy.Time.now()
            self.joint_state_.position[0:9] = map(float, position[i][0:9])
            if len(position[i]) > 9:
                self.joint_state_.position[9] = -float(position[i][9].replace('\U00002013', '-'))
            else:
                self.joint_state_.position[9] = 0.0
            self.joint_pub_.publish(self.joint_state_)
            rate.sleep()

    def open_fridge_door_3(self):
        rate = rospy.Rate(20)

        with open('/home/jiao/vkc_ws/vkc-demo-traj/open_fridge_3.csv') as f:
            reader = csv.reader(f)
            position = list(reader)
        
        time.sleep(2)
        
        for i in range(0, len(position)):
            self.joint_state_.header.stamp = rospy.Time.now()
            self.joint_state_.position[0:9] = map(float, position[i][0:9])
            if len(position[i]) > 9:
                self.joint_state_.position[9] = -float(position[i][9].replace('\U00002013', '-'))
            else:
                self.joint_state_.position[9] = 0.0
            self.joint_pub_.publish(self.joint_state_)
            rate.sleep()
    
    def open_cabinet_door_1(self):
        rate = rospy.Rate(20)

        with open('/home/jiao/vkc_ws/vkc-demo-traj/open_cabinet_1.csv') as f:
            reader = csv.reader(f)
            position = list(reader)
        
        time.sleep(2)
        
        for i in range(0, len(position)):
            self.joint_state_.header.stamp = rospy.Time.now()
            self.joint_state_.position[0:9] = map(float, position[i][0:9])
            if len(position[i]) > 9:
                self.joint_state_.position[11] = -float(position[i][9].replace('\U00002013', '-'))
            else:
                self.joint_state_.position[11] = 0.0
            self.joint_pub_.publish(self.joint_state_)
            rate.sleep()
    
    def open_cabinet_door_2(self):
        rate = rospy.Rate(20)

        with open('/home/jiao/vkc_ws/vkc-demo-traj/open_cabinet_2.csv') as f:
            reader = csv.reader(f)
            position = list(reader)
        
        time.sleep(2)
        
        for i in range(0, len(position)):
            self.joint_state_.header.stamp = rospy.Time.now()
            self.joint_state_.position[0:9] = map(float, position[i][0:9])
            if len(position[i]) > 9:
                self.joint_state_.position[11] = -float(position[i][9].replace('\U00002013', '-'))
            else:
                self.joint_state_.position[11] = 0.0
            self.joint_pub_.publish(self.joint_state_)
            rate.sleep()

    def open_dishwasher_1(self):
        rate = rospy.Rate(20)

        with open('/home/jiao/vkc_ws/vkc-demo-traj/open_dishwasher.csv') as f:
            reader = csv.reader(f)
            position = list(reader)
        
        time.sleep(2)
        
        for i in range(0, len(position)):
            self.joint_state_.header.stamp = rospy.Time.now()
            self.joint_state_.position[0:9] = map(float, position[i][0:9])
            self.joint_state_.position[16] = 0.9250
            if len(position[i]) > 9:
                self.joint_state_.position[10] = -float(position[i][9].replace('\U00002013', '-'))
            else:
                self.joint_state_.position[10] = -0.3080
            self.joint_pub_.publish(self.joint_state_)
            rate.sleep()

def print_menu():
    menu = ""

    menu += "0. Open fridge door 1\n"
    menu += "1. Open fridge door 2\n"
    menu += "2. Open fridge door 3\n"
    menu += "3. Open cabinet door 1\n"
    menu += "4. Open cabinet door 2\n"
    menu += "5. Open dishwasher 1\n"
    
    print(menu)


def main():
    rospy.init_node("VKC Demo")
    scene_ctrl = SceneController()

    menu_func = [
        scene_ctrl.open_fridge_door_1,
        scene_ctrl.open_fridge_door_2,
        scene_ctrl.open_fridge_door_3,
        scene_ctrl.open_cabinet_door_1,
        scene_ctrl.open_cabinet_door_2,
        scene_ctrl.open_dishwasher_1
    ]
    
    while not rospy.is_shutdown():
        print_menu()
        ret = int(input("Select action: "))
        
        menu_func[ret]()
            

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass