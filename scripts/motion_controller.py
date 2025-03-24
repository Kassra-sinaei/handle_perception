#!/usr/bin/env python3.8
# filepath: ~/alphaz_ws/src/handle_detection/scripts/handle_detection_node.py

import rospy
from geometry_msgs.msg import Point
from std_srvs.srv import SetBool
from std_srvs.srv import Empty
from dual_arm_msgs.msg import MoveJ_P
import numpy as np

class MotionControllerNode:
    def __init__(self):
        rospy.init_node('motion_controller', anonymous=True)
        self.subscriber = rospy.Subscriber('/grasp_point', Point, self.grasp_point_callback)
        rospy.loginfo("Motion Controller Node has started and is listening to /grasp_point topic.")
        
        self.handle_pose = None
        self.camera2r_base = np.array([[ 0.70738827,  0.0,          0.70682518,  0.2099817 ],
                                        [ 0.0,          1.0,          0.0,         -0.0638],
                                        [-0.70682518,  0.0,          0.70738827,  0.07291561],
                                        [ 0.0,          0.0,          0.0,          1.0        ]])
        
        self.camera2l_base = np.array([[ -0.70738827,  0.0,          0.70682518,  0.2142076 ],
                                        [ 0.0,          -1.0,          0.0,         -0.0638],
                                        [0.70682518,  0.0,          0.70738827,  0.06869],
                                        [ 0.0,          0.0,          0.0,          1.0        ]])
        
        self.r_arm_move = rospy.Publisher('/r_arm/rm_driver/MoveJ_P_Cmd', MoveJ_P, queue_size=1)
        self.l_arm_move = rospy.Publisher('/l_arm/rm_driver/MoveJ_P_Cmd', MoveJ_P, queue_size=1)

        self.arm_switch_service = ros

        self.grasp_r_service = rospy.Service('/grasp_r', SetBool, self.handle_grasp_r)

        self.active_arm = 'right'
        # TODO: write a service for changing the active arm

        rospy.loginfo("Motion Controller Node has started.")

    def grasp_point_callback(self, msg):
        self.handle_pose = np.array([msg.x, msg.y, msg.z])
        rospy.loginfo(f"Received grasp point: {msg}")

    def handle_grasp_r(self, req):
        if self.handle_pose is None:
            return False, "Grasp point not detected yet."
        else:
            # Move selected arm to grasp point  
            try:
                msg = MoveJ_P()
                msg.speed = 0.3
                msg.trajectory_connect = False
                if self.active_arm == 'right':
                    target = self.camera2r_base @ np.append(self.handle_pose, 1)        # transform to base frame of selected arm
                    target = target[:3]                                                 # remove the fourth element
                    msg.Pose = target.tolist()
                    self.r_arm_move.publish(msg)
                else:
                    target = self.camera2l_base @ np.append(self.handle_pose, 1)        # transform to base frame of selected arm
                    target = target[:3]                                                 # remove the fourth element
                    msg.Pose = target.tolist()
                    self.l_arm_move.publish(msg)
            except:
                return False, "Failed to move right arm to grasp point."
    
    def run(self):
        rospy.spin()

def main():
    node = MotionControllerNode()
    node.run()

if __name__ == '__main__':
    main()