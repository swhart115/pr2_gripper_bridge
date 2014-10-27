#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest("pr2_gripper_bridge")

from    nasa_robot_teleop.srv import *
from pr2_controllers_msgs.msg import *

from math import tan

class Pr2GripperBridge:

    def __init__(self):

        self.publishers = {}
        self.publishers['left_gripper'] =  rospy.Publisher('/l_gripper_controller/command', Pr2GripperCommand, queue_size=1)
        self.publishers['right_gripper'] = rospy.Publisher('/r_gripper_controller/command', Pr2GripperCommand, queue_size=1)

        self.service = rospy.Service('/pr2_gripper_bridge/end_effector_command', EndEffectorCommand, self.handle_end_effector_command)

    def handle_end_effector_command(self, req):
        print "Pr2GripperBridge() -- got new EndEffectorCommand: "
        print "  end-effector name: ", req.name
        print "  pose name: ", req.pose_name
        # print "  traj: ", req.goal_trajectory 
        N = len(req.goal_trajectory.points)
        g = req.goal_trajectory.points[N-1].positions
        print "  final goal: ", g

        cmd = Pr2GripperCommand()
        cmd.position = (0.15*tan(g[0]))-.02

        print "command pos: ", cmd.position 
        cmd.max_effort = 1000
        self.publishers[req.name].publish(cmd)
        return EndEffectorCommandResponse(True)

if __name__ == "__main__":

    rospy.init_node('pr2_gripper_bridge')

    pr2_gripper_bridge = Pr2GripperBridge()
    
    r = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        r.sleep()