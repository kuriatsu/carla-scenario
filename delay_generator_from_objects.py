#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from derived_object_msgs.msg import ObjectArray



class DelayGenerator():
    def __init__(self, role_name, delay_time):
        self.objects_sub = rospy.Subscriber(
            "/carla/{}/objects".format(role_name), ObjectArray, self._objects_updated)
        self.objects_delay_pub = rospy.Publisher(
            "/carla/{}/objects/delay".format(role_name), ObjectArray, queue_size=10)
        self.markers_delay_pub = rospy.Publisher(
            "/carla/{}/markers/delay".format(role_name), MarkerArray, queue_size=10)
        
        # [second]
        self.delay_time = delay_time
        self._delay_objects_queue = []
    
    def _objects_updated(self, object_array):
        self._delay_objects_queue.append(object_array)
        delay_object_array = self.get_delay_object_array()

        delay_marker_array = MarkerArray()
        if delay_object_array.objects:
            for object in delay_object_array.objects:
                delay_marker_array.markers.append(self.get_marker_from_object(object))

        self.markers_delay_pub.publish(delay_marker_array)
        self.objects_delay_pub.publish(delay_object_array)
        

    def get_delay_object_array(self):
        current_time = rospy.Time.now().to_sec()

        for i, object_array in enumerate(self._delay_objects_queue):
            dif_time = current_time - object_array.header.stamp.to_sec()
            if dif_time >= self.delay_time:
                del self._delay_objects_queue[i]
                object_array.header.stamp = rospy.Time.now()
                return object_array
            else:
                continue

        header = Header(stamp=rospy.Time.now(), frame_id='map')
        object_array = ObjectArray(header=header, objects=[])

        return object_array

    def get_marker_from_object(self, object):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.id = object.id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose = object.pose
        marker.scale.x = object.shape.dimensions[0]
        marker.scale.y = object.shape.dimensions[1]
        marker.scale.z = object.shape.dimensions[2]

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.8
        marker.color.a = 0.0

        marker.lifetime = rospy.Duration(1)

        return marker
    
    def set_delay_time(self, delay_time):
        self.delay_time = delay_time


def main():
    rospy.init_node("carla_delay_publisher")

    role_name = 'ego_vehicle'
    delay_time = 5
    delay_gen = DelayGenerator(role_name, delay_time)

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
