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
        self.moveit_setup()
        rospy.Subscriber("/detect_grasps/clustered_grasps", 
			            msg.GraspConfigList, 
		                self.cb, 
		                queue_size=10)
        self.flag = True
        print("End-Effector Link: ", self.group.get_end_effector_link())
        self.listener = tf.TransformListener()
        print("Init Complete")
        
    def tf_calc(self):
        # Put tf/autolab stuff here
        (tr_cam_base, rot_cam_base) = self.listener.lookupTransform('/panda_link0',
                                                                    '/color',
                                                                    rospy.Time(0))
        
        # transform from camera to robot base
        pose_cam_robot = geometry_msgs.msg.Pose()
        pose_cam_robot.position.x = tr_cam_base[0]
        pose_cam_robot.position.y = tr_cam_base[1]
        pose_cam_robot.position.z = tr_cam_base[2]
        pose_cam_robot.orientation.x = rot_cam_base[0]
        pose_cam_robot.orientation.y = rot_cam_base[1]
        pose_cam_robot.orientation.z = rot_cam_base[2]
        pose_cam_robot.orientation.w = rot_cam_base[3]
        
        # transform from approach grasp frame to camera
        pose_approach_cam = geometry_msgs.msg.Pose()
        pose_approach_cam.position.x = self.position[0]
        pose_approach_cam.position.y = self.position[1]
        pose_approach_cam.position.z = self.position[2]
        pose_approach_cam.orientation.x = self.quat[0]
        pose_approach_cam.orientation.y = self.quat[1]
        pose_approach_cam.orientation.z = self.quat[2]
        pose_approach_cam.orientation.w = self.quat[3]
        
        # transform from grasp frame to camera
        pose_grasp_cam = geometry_msgs.msg.Pose()
        pose_grasp_cam.position.x = self.grasp_pos[0]
        pose_grasp_cam.position.y = self.grasp_pos[1]
        pose_grasp_cam.position.z = self.grasp_pos[2]
        pose_grasp_cam.orientation.x = self.quat[0]
        pose_grasp_cam.orientation.y = self.quat[1]
        pose_grasp_cam.orientation.z = self.quat[2]
        pose_grasp_cam.orientation.w = self.quat[3]
        
        # transform from end-effector link8 to grasp frame
        pose_eef_grasp = geometry_msgs.msg.Pose()
        pose_eef_grasp.position.x = self.position[0]
        pose_eef_grasp.position.y = self.position[1]
        pose_eef_grasp.position.z = self.position[2]
        pose_eef_grasp.orientation.x = self.quat[0]
        pose_eef_grasp.orientation.y = self.quat[1]
        pose_eef_grasp.orientation.z = self.quat[2]
        pose_eef_grasp.orientation.w = self.quat[3]

    def cb(self, grasp_msg):
        if self.flag == True:
            pos = grasp_msg.grasps[0].position
            app = grasp_msg.grasps[0].approach
            binorm = grasp_msg.grasps[0].binormal
            axis = grasp_msg.grasps[0].axis
            self.width = np.float(grasp_msg.grasps[0].width.data)
            offset = 0.05
            self.position = np.array([pos.x - offset, pos.y , (pos.z -0.060)])
            self.grasp_pos = np.array([pos.x - offset, pos.y , pos.z])
            self.orientation = np.array([[app.x,binorm.x,axis.x,0],
					                     [app.y,binorm.y,axis.y,0],
					                     [app.z,binorm.z,axis.z,0],
                                         [0    ,0       ,0     ,1]])
            
            pre_orientation = self.orientation
            post_orientation = self.orientation
            
            pre_orientation[:,3] = [self.position[0], self.position[1], self.position[2], 1]
            post_orientation[:,3] = [self.grasp_pos[0], self.grasp_pos[1], self.grasp_pos[2], 1]
            
            self.pos_offset_pre_grasp = [0, -0.016, 0.18, 1]
            self.pos_offset_grasp = [0, -0.016, 0.08, 1]
            
            # NEW POSITION VALUES
            self.adjusted_pre_grasp_pos = np.inner(self.pos_offset_pre_grasp, pre_orientation)
            # print(self.adjusted_pre_grasp_pos)
            self.adjusted_pre_grasp_pos = self.adjusted_pre_grasp_pos[:3]
            self.adjusted_grasp_pos = np.inner(self.pos_offset_grasp, post_orientation)
            self.adjusted_grasp_pos = self.adjusted_grasp_pos[:3]
            
            quat_offset = tf.transformations.quaternion_from_euler(45, 90, 0)
            
            # KEEP SAME ORIENTATION VALUES
            # self.quat = tf.transformations.quaternion_from_matrix(self.orientation)
            
            quat_adjust = tf.transformations.quaternion_from_matrix(self.orientation)
            self.quat = tf.transformations.quaternion_multiply(quat_adjust, quat_offset)
            print("Pos", self.position)
            # print("Adjusted pre grasp pos", self.adjusted_pre_grasp_pos)
            # print("Adjusted grasp pos", self.adjusted_grasp_pos)
            print("Quat", self.quat)
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
        self.group.set_end_effector_link("panda_hand")
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                        moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        planning_frame = self.group.get_planning_frame() 

    def execute(self):
        print("Opening Gripper")
        self.open_gripper()
        
        print("Moving Robot to Pre Grasp")
        pre_grasp = geometry_msgs.msg.PoseStamped()
        pre_grasp.header.frame_id = 'color'
        pre_grasp.pose.position.x = self.grasp_pos[0]
        pre_grasp.pose.position.y = self.grasp_pos[1]
        pre_grasp.pose.position.z = self.grasp_pos[2]
        pre_grasp.pose.orientation.x = self.quat[0]
        pre_grasp.pose.orientation.y = self.quat[1]
        pre_grasp.pose.orientation.z = self.quat[2]
        pre_grasp.pose.orientation.w = self.quat[3]
        
        pre_grasp1 = geometry_msgs.msg.PoseStamped()
        pre_grasp1 = self.listener.transformPose('/panda_link0', pre_grasp)
        pre_grasp_adjust = self.listener.transformPose('/panda_link0', pre_grasp)
        pre_grasp_adjust.pose.position.x += self.pos_offset_pre_grasp[0]
        pre_grasp_adjust.pose.position.y += self.pos_offset_pre_grasp[1]
        pre_grasp_adjust.pose.position.z += self.pos_offset_pre_grasp[2]
        
        # print("Pre-Grasp:", pre_grasp)
        # print("Pre-Grasp1:", pre_grasp1)
        # print("Pre-Grasp Adjusted:", pre_grasp_adjust)

        self.move_robot(pre_grasp_adjust)
        
        print("Moving Robot to Grasp")
        grasp = geometry_msgs.msg.PoseStamped()
        grasp.header.frame_id = 'color'
        grasp.pose.position.x = self.grasp_pos[0]
        grasp.pose.position.y = self.grasp_pos[1]
        grasp.pose.position.z = self.grasp_pos[2]
        grasp.pose.orientation.x = self.quat[0]
        grasp.pose.orientation.y = self.quat[1]
        grasp.pose.orientation.z = self.quat[2]
        grasp.pose.orientation.w = self.quat[3]
        
        grasp1 = geometry_msgs.msg.PoseStamped()
        grasp1 = self.listener.transformPose('/panda_link0', grasp)
        grasp_adjust = self.listener.transformPose('/panda_link0', grasp)
        grasp_adjust.pose.position.x += self.pos_offset_grasp[0]
        grasp_adjust.pose.position.y += self.pos_offset_grasp[1]
        grasp_adjust.pose.position.z += self.pos_offset_grasp[2]

        self.move_robot(grasp_adjust)
        
        print("Closing Gripper")
        self.close_gripper()
        
        print("Returning to Ready Position")
        self.ready_joint_pose()
        
    def ready_joint_pose(self):
        joint1 = [0.000,0.200,-0.000,-1.5963787922313974,-0.000,1.9,0.785]
        joint_goal = self.group.get_current_joint_values()
        joint_goal[0] = joint1[0]
        joint_goal[1] = joint1[1]
        joint_goal[2] = joint1[2]
        joint_goal[3] = joint1[3]
        joint_goal[4] = joint1[4]
        joint_goal[5] = joint1[5]
        joint_goal[6] = joint1[6]
        self.group.go(joint_goal, wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        
        self.flag = True


def main(args):
    rospy.init_node('gpd_ros_execution')
    gp = gpd()
    gp.ready_joint_pose()    
    rospy.sleep(5)
    while True:
        gp.execute()
        rospy.sleep(5)
        a = raw_input("Would you like to execute the grasp program again? (y/n)")
        if a == 'Y' or a == 'y':
            continue
        else:
            break
    
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)