#!/usr/bin/env python

import rospy
import glob
import os
import sys
try:
    sys.path.append(glob.glob('**/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
    sys.path.append("./")
    sys.path.append("../carla/PythonAPI/carla/")
except IndexError:
    pass
import carla
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from tf.transformations import quaternion_from_euler
#from carla_common.transforms import carla_location_to_ros_point, carla_rotation_to_ros_quaternion
from geometry_msgs.msg import Vector3, Quaternion, Transform, Pose, Point, Twist, Accel
import math
from transforms3d.euler import euler2mat, quat2euler, euler2quat

def carla_location_to_ros_point(carla_location):
    """
    Convert a carla location to a ROS point
    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS)
    :param carla_location: the carla location
    :type carla_location: carla.Location
    :return: a ROS point
    :rtype: geometry_msgs.msg.Point
    """
    ros_point = Point()
    ros_point.x = carla_location.x
    ros_point.y = -carla_location.y
    ros_point.z = carla_location.z

    return ros_point

def carla_rotation_to_ros_quaternion(carla_rotation):
    """
    Convert a carla rotation to a ROS quaternion
    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS).
    Considers the conversion from degrees (carla) to radians (ROS).
    :param carla_rotation: the carla rotation
    :type carla_rotation: carla.Rotation
    :return: a ROS quaternion
    :rtype: geometry_msgs.msg.Quaternion
    """
    roll, pitch, yaw = carla_rotation_to_RPY(carla_rotation)
    quat = euler2quat(roll, pitch, yaw)
    ros_quaternion = Quaternion(w=quat[0], x=quat[1], y=quat[2], z=quat[3])
    return ros_quaternion


def carla_rotation_to_RPY(carla_rotation):
    """
    Convert a carla rotation to a roll, pitch, yaw tuple
    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS).
    Considers the conversion from degrees (carla) to radians (ROS).
    :param carla_rotation: the carla rotation
    :type carla_rotation: carla.Rotation
    :return: a tuple with 3 elements (roll, pitch, yaw)
    :rtype: tuple
    """
    roll = math.radians(carla_rotation.roll)
    pitch = -math.radians(carla_rotation.pitch)
    yaw = -math.radians(carla_rotation.yaw)

    return (roll, pitch, yaw)


def get_marker_from_actor(actor, marker_id):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.id = marker_id
    marker.ns = actor.attributes.get('role_name')
    marker.type = Marker.CUBE
    marker.action = Marker.ADD


    bbox = actor.bounding_box


    #marker.pose.position =  #carla_location_to_ros_point(bbox.location)
    marker.pose.position.x = actor.get_location().x
    marker.pose.position.y = -actor.get_location().y
    marker.pose.position.z = actor.get_location().z

    rotation = actor.get_transform().rotation

    print(rotation)

    marker.pose.orientation = carla_rotation_to_ros_quaternion(rotation)
    marker.scale.x = bbox.extent.x * 2
    marker.scale.y = bbox.extent.y * 2
    marker.scale.z = bbox.extent.z * 2

    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 0.8

    marker.lifetime = rospy.Duration(1)

    return marker

def main():
    rospy.init_node("carla_marker_publisher")

    marker_pub = rospy.Publisher("/carla/markers", MarkerArray, queue_size=10)

    # Connect to CARLA server
    client = carla.Client("localhost", 2000)
    client.set_timeout(2.0)
    world = client.get_world()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        marker_array = MarkerArray()

        actors = world.get_actors()
        for idx, actor in enumerate(actors):
            if actor.type_id.startswith("vehicle") or actor.type_id.startswith("walker"):
                marker_array.markers.append(get_marker_from_actor(actor, idx))

        marker_pub.publish(marker_array)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
