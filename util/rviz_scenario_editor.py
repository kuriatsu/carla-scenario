#!/usr/bin/python
# -*- coding: utf-8 -*

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
import tf
import pyperclip
import math


class RvizScenarioEditor():
    # def __init__(self):

    def pointCb(self, point):
        str = "{:.1f},{:.1f},{:.1f}".format(point.point.x, -point.point.y, 1.0)
        print(str)
        pyperclip.copy(str)

    def poseCb(self, pose):
        euler = self.quatToEuler(pose.pose.orientation)
        str = "{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f}".format(pose.pose.position.x, -pose.pose.position.y, 1.0, -euler[1] * 180 / math.pi, -euler[2] * 180 / math.pi, euler[0] * 180 / math.pi)
        print(str)
        pyperclip.copy(str)

    def quatToEuler(self, orientation):
        euler = tf.transformations.euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))
        return euler

def main():
    rospy.init_node('rviz_scenario_editor_node')
    rviz_scenario_editor = RvizScenarioEditor()

    sub_point = rospy.Subscriber('/clicked_point', PointStamped, rviz_scenario_editor.pointCb)
    sub_point = rospy.Subscriber('/move_base_simple/goal', PoseStamped, rviz_scenario_editor.poseCb)
    rospy.spin()


if __name__ == '__main__':
    main()
