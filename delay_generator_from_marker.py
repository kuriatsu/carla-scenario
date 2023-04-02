#!/usr/bin/env python

import math
import time
import datetime
import threading
import rospy
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
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
        self._delay_markers_queue = []
    
    def _markers_updated(self, marker_array):
        t = threading.Thread(target=self.publish_delay_marker_array, args=(marker_array,))
        t.start()

        # rospy.sleep(self.delay_time)
        # self.markers_delay_pub.publish(marker_array)
        # self._delay_markers_queue.append(marker_array)
        # delay_marker_array = self.get_delay_marker_array()
        # print('array', len(delay_marker_array.markers))
        # print('queue', len(self._delay_markers_queue))

        # if delay_marker_array.markers:
        #     self.markers_delay_pub.publish(delay_marker_array)
    
    def publish_delay_marker_array(self, marker_array):

        
        time.sleep(self.delay_time)
        for marker in marker_array.markers:
            self._convert_marker_color(marker)
        print(datetime.datetime.now())
        self.markers_delay_pub.publish(marker_array)

    def get_delay_marker_array(self):
        delay_marker_array = MarkerArray(markers=[])
        current_time = rospy.Time.now()

        for i, marker_array in enumerate(self._delay_markers_queue):
            for j, marker in enumerate(marker_array.markers):
                if marker.ns == self._role_name:
                    continue
                dif_time = current_time.to_sec() - marker.header.stamp.to_sec()
                print('dif_time', dif_time)
                if dif_time >= self.delay_time:
                    marker.header.stamp = current_time
                    self._convert_marker_color(marker)
                    print(marker.pose)
                    del marker_array.markers[j]
                    delay_marker_array.markers.append(marker)

            if not marker_array.markers:
                print('delete queue')
                del self._delay_markers_queue[i]

        return delay_marker_array

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
