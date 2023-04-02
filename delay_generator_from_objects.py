#!/usr/bin/env python

import time
import threading
import rospy
from derived_object_msgs.msg import ObjectArray



class DelayObjectGenerator():
    def __init__(self, role_name, delay_time):
        self.objects_sub = rospy.Subscriber(
            "/carla/{}/objects".format(role_name), ObjectArray, self._objects_updated)
        self.objects_delay_pub = rospy.Publisher(
            "/carla/{}/objects/delay".format(role_name), ObjectArray, queue_size=10)
        
        # [second]
        self.delay_time = delay_time
    
    def _objects_updated(self, object_array):
        t = threading.Thread(target=self.publish_delay_object_array, args=(object_array,))
        t.start()
    
    def publish_delay_object_array(self, object_array):
        time.sleep(self.delay_time)
        self.objects_delay_pub.publish(object_array)
    
    def set_delay_time(self, delay_time):
        self.delay_time = delay_time


def main():
    rospy.init_node("carla_delay_publisher")

    role_name = 'ego_vehicle'
    delay_time = 5
    delay_gen = DelayObjectGenerator(role_name, delay_time)

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
