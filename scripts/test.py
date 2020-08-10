#!/usr/bin/env python

import rospy
import rosservice
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
import tf
from gpd_ros import msg


class gpd():
    def __init__(self):
        rospy.Subscriber("/detect_grasps/clustered_grasps", 
			            msg.GraspConfigList, 
		                self.cb, 
		                queue_size=10)
        self.flag = True
        self.moveit_setup()
        print("Init Complete")
        
    def tf_calc(self):
        # Put tf/autolab stuff here
        pass

    def cb(self, msg):
        if self.flag == True:
            pos = msg.grasps[0].position
            app = msg.grasps[0].approach
            binorm = msg.grasps[0].binormal
            axis = msg.grasps[0].axis
            self.width = np.float(msg.grasps[0].width.data)
            offset = 0.05
            self.position = np.array([pos.x - offset, pos.y , (pos.z -0.060)])
            self.grasp_pos = np.array([pos.x - offset, pos.y , pos.z])
            self.orientation = np.array([[app.x,binorm.x,axis.x,0],
					                     [app.y,binorm.y,axis.y,0],
					                     [app.z,binorm.z,axis.z,0],
                                         [0    ,0       ,0     ,1]])
            self.quat = tf.transformations.quaternion_from_matrix(self.orientation)
            print(self.position)
            print(self.quat)
            self.flag = False

    def move_robot(self, goal_pose):
        print("Moving...")
        self.group.set_pose_target(goal_pose)
        self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
    
    def close_gripper(self):
        self.group_h.set_joint_value_target([0.0, 0.0])
        self.group_h.go(wait=True)
        self.group_h.stop()
        self.group_h.clear_pose_targets()

    def open_gripper(self):
        w = self.width/2.0
        goal = min(0.04, w+0.02)
        self.group_h.set_joint_value_target([goal, goal])
        self.group_h.go(wait=True)
        self.group_h.stop()
        self.group_h.clear_pose_targets()

    def moveit_setup(self):  
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("panda_arm")
        self.group_h = moveit_commander.MoveGroupCommander("hand")
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                        moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        planning_frame = self.group.get_planning_frame() 

    def execute(self):
        print("Moving Robot to Pre Grasp")
        pre_grasp = geometry_msgs.msg.Pose()
        pre_grasp.position.x = self.position[0]
        pre_grasp.position.y = self.position[1]
        pre_grasp.position.z = self.position[2]
        pre_grasp.orientation.x = self.quat[0]
        pre_grasp.orientation.y = self.quat[1]
        pre_grasp.orientation.z = self.quat[2]
        pre_grasp.orientation.w = self.quat[3]

        self.move_robot(pre_grasp)
        
        print("Moving Robot to Grasp")
        grasp = geometry_msgs.msg.Pose()
        grasp.position.x = self.grasp_pos[0]
        grasp.position.y = self.grasp_pos[1]
        grasp.position.z = self.grasp_pos[2]
        grasp.orientation.x = self.quat[0]
        grasp.orientation.y = self.quat[1]
        grasp.orientation.z = self.quat[2]
        grasp.orientation.w = self.quat[3]

        self.move_robot(grasp)


def main(args):
    rospy.init_node('gpd_ros_execution')
    gp = gpd()
    rospy.sleep(2)
    gp.execute()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)