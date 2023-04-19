#!/usr/bin/env python

import time
import threading
import rospy
from visualization_msgs.msg import MarkerArray


class DelayMarkerGenerator():
    def __init__(self, role_name, delay_time):
        self.markers_sub = rospy.Subscriber(
            "/carla/markers".format(role_name), MarkerArray, self._markers_updated)
        self.markers_delay_pub = rospy.Publisher(
            "/carla/{}/markers/delay".format(role_name), MarkerArray, queue_size=10)
        
        self._role_name = role_name
        # [second]
        self.delay_time = delay_time
    
    def _markers_updated(self, marker_array):
        t = threading.Thread(target=self.publish_delay_marker_array, args=(marker_array,))
        t.start()
    
    def publish_delay_marker_array(self, marker_array):
        time.sleep(self.delay_time)

        for marker in marker_array.markers:
            self._convert_marker_color(marker)
            marker.header.stamp = rospy.Time.now()

        self.markers_delay_pub.publish(marker_array)

    def _convert_marker_color(self, marker):
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.8
    
    def set_delay_time(self, delay_time):
        self.delay_time = delay_time


def main():
    rospy.init_node("carla_delay_publisher")

    role_name = 'ego_vehicle'
    delay_time = 2.5
    delay_gen = DelayMarkerGenerator(role_name, delay_time)

    rospy.spin()
    

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
