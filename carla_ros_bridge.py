#!/usr/bin/python
# -*- coding: utf-8 -*-

import glob
import os
import sys
import argparse
import math
import logging
try:
    sys.path.append(glob.glob('**/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import rospy
import tf
import carla
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Accel
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from derived_object_msgs.msg import ObjectArray
from derived_object_msgs.msg import Object
from shape_msgs.msg import SolidPrimitive

from scenario_xml import ScenarioXML

bike_blueprints = [
'vehicle.bh.crossbike',
'vehicle.diamondback.century',
]
motorcycle_blueprints = [
'vehicle.harley-davidson.low_rider',
'vehicle.kawasaki.ninja',
'vehicle.yamaha.yzf',
]
truck_blueprints = [
'vehicle.carlamotors.carlacola'
]

class CarlaBridge(object):
    def __init__(self):

        self.client = None
        self.world = None
        self.scenario_file = None
        self.scenario_xml = None

        # self.pub_pose = rospy.Publisher('/ndt_pose', PoseStamped, queue_size=1)
        # self.pub_twist = rospy.Publisher('/estimate_twist', TwistStamped, queue_size=1)
        self.pub_obj = rospy.Publisher('/my_carla_actors', ObjectArray, queue_size=5)

        self.ego_vehicle = None
        self.ego_pose = None
        self.ego_twist = None
        # following 2 list muxt have same actor with the same index
        self.ros_actors = [] # derived_object_msgs
        self.carla_actors = [] # carla.actor
        self.loop_num = 0
        self.pedestrian_cross_factor = None


    def getActors(self):
        """Call simulator and get instances of actors and store them, create list of derived_object_msgs

        """
        # self.ros_actors.clear()
        del self.carla_actors[:]
        del self.ros_actors[:]

        for actor in self.world.get_actors():
            if actor.attributes.get('role_name') in ['ego_vehicle', 'hero']:
                self.ego_vehicle = actor
                self.scenario_xml.ego_vehicle = actor

            elif actor.type_id.startswith("vehicle") or actor.type_id.startswith("walker"):
                self.carla_actors.append(actor)
                derived_obj = Object(detection_level=self.getActorProb(actor.id))

                derived_obj.id = actor.id
                derived_obj.shape.type = SolidPrimitive.BOX
                derived_obj.shape.dimensions = [
                    actor.bounding_box.extent.x*2,
                    actor.bounding_box.extent.y*2,
                    actor.bounding_box.extent.z*2
                    ]

                # if actor.type_id in bike_blueprints:
                #     derived_obj.classification = Object.CLASSIFICATION_BIKE
                # elif actor.type_id in motorcycle_blueprints:
                #     derived_obj.classification = Object.CLASSIFICATION_MOTORCYCLE
                # elif actor.type_id in truck_blueprints:
                #     derived_obj.classification = Object.CLASSIFICATION_TRUCK
                # elif actor.type_id.startswith("vehicle"):
                if actor.type_id.startswith("vehicle"):
                    derived_obj.classification = Object.CLASSIFICATION_CAR
                elif actor.type_id.startswith("walker"):
                    derived_obj.classification = Object.CLASSIFICATION_PEDESTRIAN
                else:
                    derived_obj.classification = Object.CLASSIFICATION_OTHER_VEHICLE
                self.ros_actors.append(derived_obj)

            elif actor.type_id.startswith('static'):
                self.carla_actors.append(actor)
                derived_obj = Object(detection_level=self.getActorProb(actor.id))
                derived_obj.id = actor.id
                derived_obj.shape.type = SolidPrimitive.BOX
                # print(self.scenario_xml.blueprint.find(actor.type_id).size)
                size = 1
                # size = self.scenario_xml.blueprint.find(actor.type_id).size
                derived_obj.shape.dimensions = [
                    float(size)*2,
                    float(size)*2,
                    float(size)*2
                    ]

                derived_obj.classification = Object.CLASSIFICATION_UNKNOWN
                self.ros_actors.append(derived_obj)


    def getActorProb(self, world_id):
        """find spawn actor info from scenario_xml and return probability
        if None, return 0
        ~args~
        world_id
        ~return~
        actor probability from scenario_xml
        """
        for spawn in self.scenario_xml.scenario.iter('spawn'):
            if spawn.find('world_id') is None: continue
            if int(spawn.find('world_id').text) != world_id: continue
            if spawn.find('probability') is None: return 0

            return int(spawn.find('probability').text)

        return 0


    def updateActors(self):
        """update actor dynamic info
        """
        # pose = self.scenario_xml.ego_pose
        # twist_linear = self.ego_vehicle.get_velocity()
        # twist_angular = self.ego_vehicle.get_angular_velocity()

        # self.ego_pose = Pose(
        #     position=Point(
        #         x=pose.location.x,
        #         y=pose.location.y,
        #         z=pose.location.z
        #         ),
        #     orientation=self.rotation_to_quaternion(self.scenario_xml.ego_pose.rotation)
        #     )
        #
        # self.ego_twist = Twist(
        #     linear=Vector3(
        #         x=twist_linear.x,
        #         y=twist_linear.y,
        #         z=twist_linear.z
        #     ),
        #     angular=Vector3(
        #         x=twist_angular.x,
        #         y=twist_angular.y,
        #         z=twist_angular.z
        #     )
        # )

        for i, actor in enumerate(self.carla_actors):

            transform = actor.get_transform()
            if transform.location == carla.Vector3D(0.0,0.0,0.0):
                print('ros_bridge: actor may be died. id : ', actor.id)
                del self.ros_actors[i]
                del self.carla_actors[i]
                continue

            accel = actor.get_acceleration()
            twist_angular = actor.get_angular_velocity()
            twist_linear = actor.get_velocity()

            ros_actor = self.ros_actors[i]

            ros_actor.header = Header(stamp=rospy.Time.now(), frame_id='map')
            ros_actor.pose = Pose(
                position=Point(
                    x=transform.location.x,
                    y=-transform.location.y,
                    z=transform.location.z
                    ),
                orientation=self.rotation_to_quaternion(transform.rotation)
                )

            # static object tend to sink under ground
            if ros_actor.classification == Object.CLASSIFICATION_UNKNOWN:
                ros_actor.pose.position.z += ros_actor.shape.dimensions[2] * 0.5

            ros_actor.twist = Twist(
                linear=Vector3(
                    x=twist_linear.x,
                    y=-twist_linear.y,
                    z=-twist_linear.z
                    ),
                angular=Vector3(
                    x=twist_angular.x,
                    y=-twist_angular.y,
                    z=-twist_angular.z
                    )
                )
            ros_actor.accel = Accel(
                linear=Vector3(
                    x=accel.x,
                    y=-accel.y,
                    z=-accel.z
                    ),
                angular=Vector3()
                )


    def rotation_to_quaternion(self, rotation):
        quat = tf.transformations.quaternion_from_euler(
            math.radians(rotation.roll),
            -math.radians(rotation.pitch),
            -math.radians(rotation.yaw)
            )
        return Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])


    def timerCallback(self, event):
        if self.ego_vehicle is None:
            self.getActors()
            return

        self.scenario_xml.ego_pose = self.scenario_xml.ego_vehicle.get_transform()
        flag = self.scenario_xml.checkTrigger()
        self.world.wait_for_tick()

        if flag == 3:
            print('no_scenario_file')
            exit(1)
        elif flag == 2:
            print('restart')
            self.loop_num += 1
            self.startScenario(self.scenario_file[self.loop_num % len(self.scenario_file)], self.pedestrian_cross_factor)
        elif flag == 1:
            self.getActors()
            self.updateActors()
            self.scenario_xml.controlActor()
        elif flag == 0:
            self.updateActors()
            self.scenario_xml.controlActor()

        header = Header(stamp=rospy.Time.now(), frame_id='map')
        object_array = ObjectArray(header=header, objects=self.ros_actors)
        self.pub_obj.publish(object_array)
        # ego_pose = PoseStamped(header=header, pose=self.ego_pose)
        # self.pub_pose.publish(ego_pose)
        # ego_twist = TwistStamped(header=header, twist=self.ego_twist)
        # self.pub_twist.publish(ego_twist)


    def startScenario(self, filename, pedestrian_cross_factor):
        del self.scenario_xml
        scenario_xml = ScenarioXML(self.client, self.world, filename, pedestrian_cross_factor)

        scenario_xml.blueprint = self.world.get_blueprint_library()
        scenario_xml.spawnActor(scenario_xml.scenario[0].findall('spawn'))
        scenario_xml.moveActor(scenario_xml.scenario[0].findall('move'))
        scenario_xml.poseActor(scenario_xml.scenario[0].findall('pose'))

        self.scenario_xml = scenario_xml
        self.getActors()
        print('started')


    def __del__(self):
        del self.scenario_xml


def main(args):

    try:
        rospy.init_node('carla_bridge_node', anonymous=True)
        carla_bridge = CarlaBridge()
        carla_bridge.client = carla.Client(args.host, args.port)
        carla_bridge.client.set_timeout(2.0)
        carla_bridge.world = carla_bridge.client.get_world()

        carla_bridge.scenario_file = args.scenario_file
        carla_bridge.pedestrian_cross_factor = args.pedestrian_cross_factor
        carla_bridge.startScenario(carla_bridge.scenario_file[0], carla_bridge.pedestrian_cross_factor)

        rospy.Timer(rospy.Duration(0.1), carla_bridge.timerCallback)
        rospy.spin()

    finally:
        del carla_bridge
        print('finished')


if __name__ == '__main__':

    argparser = argparse.ArgumentParser( description = __doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-s', '--scenario_file',
        metavar='S',
        nargs='+',
        default='/home/mad-carla/share/catkin_ws/src/carla_helper/scenario_test.xml',
        help='scenario file (default: scenario.xml)')
    argparser.add_argument(
        '-f', '--pedestrian_cross_factor',
        metavar='S',
        default=0.1,
        type=float,
        help='pedestrian cross rate 0.0-1.0')
    args = argparser.parse_args()

    main(args)
