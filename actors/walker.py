#!/usr/bin/env python
"""
Welcome to CARLA scenario controller.
"""
import carla
import xml.etree.ElementTree as ET
import argparse
import warnings
import random
from actors.actor import Actor

class Walker(Actor):
    def __init__(self, world, scenario_id, blueprint):
        super().__init__(world, scenario_id, blueprint)
        self.action_dict = {
                "spawn": self.spawn,
                "move" : self.move,
                "kill" : self.kill,
                "pose" : self.pose,
                }
        self.walker_posture_lib
        self.waypoints

    def getBlueprint(self, xml):
        if xml.find("blueprint").text == "random":
            blueprint = random.choice(self.blueprint.filter("walker.*"))
        else:
            try:
                blueprint = self.blueprint.find(xml.find("blueprint").text)
            except ValueError:
                warnings.warn(f"spcecified blueprint is not exist : {xml.find('blueprint').text}")
                blueprint = random.choice(self.blueprint.filter("walker.*"))

        blueprint.set_attribute('role_name', spawn.attrib.get(self.scenario_id))
        # set as not invencible
        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'false')

        return blueprint


    def spawn(self, xml):

        blueprint = self.getBlueprint(xml)

        transform = xml.find('transform').text
        transform = carla.Transform(
                carla.Location(transform[0], transform[1], transform[2]), 
                carla.Rotation(transform[3], transform[4], transform[5])
                )

        self.commands = [carla.command.SpawnActor(blueprint, transform)]
        return


    def pose(self, xml):
        control = carla.WalkerBoneControl()
        control.bone_transforms = LibPose.dict[xml.find('type').text]()
        self.actor.apply_control(control)
        self.world.wait_for_tick()
        self.actor.apply_control(control)
        return


    def move(self, xml):
        self.waypoints = xml.findall("waypoint")

        vector, speed, dist, _ = calcControl()
        if dist < 1.0:
            self.waypoints.pop(0)
        self.commands = [carla.command.ApplyWalkerControl(self.world_id, carla.WalkerControl(direction=vector, speed=speed))]
        return


    def calcControl(self):
        # some information for movng
        transform = self.world_actor.get_transform()
        if transform is None:
            print('cannot get transform: ' + self.scenario_id)
            return 0, 0, 0, 0

        waypoint = self.waypoints[0].text
        speed = float(self.waypoints[0].attrib.get('speed'))
        # calc culent motion vector and distance to the target
        vector = carla.Vector3D(
            float(waypoint[0]) - transform.location.x,
            float(waypoint[1]) - transform.location.y,
            float(waypoint[2]) - transform.location.z
            )
        dist = math.sqrt(vector.x ** 2 + vector.y ** 2)
        # normalize vector to calcurate velocity
        vector.x = vector.x / dist
        vector.y = vector.y / dist
        # debug.draw_arrow(begin=transform.location ,end=transform.location + carla.Location(vector.x, vector.y, 0.0), life_time=0.5)

        yaw = math.atan(
            (float(waypoint[1]) - transform.location.y) 
            / (float(waypoint[0]) - transform.location.x)
            )
        yaw = math.degrees(yaw)


        return vector, speed, dist, yaw


    def getResponse(self, response):
        super().getResponse(response)
        if self.waypoints:
            vector, speed, dist, _ = self.calcControl()
            self.commands = [carla.command.ApplyWalkerControl(self.world_id, carla.WalkerControl(direction=vector, speed=speed))]
            if dist < 1.0:
                self.waypoints.pop(0)

        else:
            self.commands = []

        return

        
class LibPose(object):

    def __init__(self):
        self.dict = {
        'phone_right' : self.posePhoneRight,
        'phone_left' : self.posePhoneLeft,
        'call_right' : self.poseCallRight,
        'call_left' : self.poseCallLeft,
        }

    def poseCallLeft(self):
            arm_L = ('crl_arm__L', carla.Transform(location=carla.Location(x=0.2, z=1.45), rotation=carla.Rotation(yaw=60, pitch=20)))
            forearm_L = ('crl_forearm__L', carla.Transform(location=carla.Location(x=0.31, y=0.16, z=1.54), rotation=carla.Rotation(yaw=20, pitch=158, roll=0)))
            hand_L = ('crl_hand__L', carla.Transform(location=carla.Location(x=0.13, y=0.1, z=1.62), rotation=carla.Rotation(pitch=20, yaw=90)))
            arm_R = ('crl_arm__R', carla.Transform(location=carla.Location(x=-0.17, y=0.0, z=1.48), rotation=carla.Rotation(yaw=90, roll=90, pitch=70)))
            forearm_R = ('crl_forearm__R', carla.Transform(location=carla.Location(x=-0.17, y=-0.07, z=1.29), rotation=carla.Rotation(yaw=-90, pitch=70)))
            return [arm_L, forearm_L, hand_L, arm_R, forearm_R]

    def poseCallRight(self):
            arm_R = ('crl_arm__R', carla.Transform(location=carla.Location(x=-0.2, z=1.45), rotation=carla.Rotation(yaw=-60, pitch=-20)))
            forearm_R = ('crl_forearm__R', carla.Transform(location=carla.Location(x=-0.31, y=0.16, z=1.54), rotation=carla.Rotation(yaw=-20, pitch=-158, roll=0)))
            hand_R = ('crl_hand__R', carla.Transform(location=carla.Location(x=-0.13, y=0.1, z=1.62), rotation=carla.Rotation(roll=0, pitch=-170, yaw=90)))
            arm_L = ('crl_arm__L', carla.Transform(location=carla.Location(x=0.17, y=0.0, z=1.48), rotation=carla.Rotation(yaw=-90, pitch=-70)))
            forearm_L = ('crl_forearm__L', carla.Transform(location=carla.Location(x=0.17, y=-0.07, z=1.29), rotation=carla.Rotation(yaw=90, pitch=-70, roll=180)))
            return [arm_R, forearm_R, hand_R, arm_L, forearm_L]

    def posePhoneRight(self):
            arm_R = ('crl_arm__R', carla.Transform(location=carla.Location(x=-0.16, z=1.49), rotation=carla.Rotation(yaw=-70, pitch=50)))
            forearm_R = ('crl_forearm__R', carla.Transform(location=carla.Location(x=-0.18, y=0.14, z=1.3), rotation=carla.Rotation(yaw=-140, roll=-30, pitch=-30)))
            hand_R = ('crl_hand__R', carla.Transform(location=carla.Location(x=-0.04, y=0.265, z=1.40), rotation=carla.Rotation(roll=-30, pitch=128, yaw=0)))
            arm_L = ('crl_arm__L', carla.Transform(location=carla.Location(x=0.17, y=0.0, z=1.48), rotation=carla.Rotation(yaw=-90, pitch=-70)))
            forearm_L = ('crl_forearm__L', carla.Transform(location=carla.Location(x=0.17, y=-0.07, z=1.29), rotation=carla.Rotation(yaw=90, pitch=-70, roll=180)))
            neck = ('crl_neck__c', carla.Transform(location=carla.Location(x=0, y=0.0, z=1.55), rotation=carla.Rotation(yaw=180, roll=50, pitch=0)))
            return [arm_R, forearm_R, hand_R, arm_L, forearm_L, neck]

    def posePhoneLeft(self):
            arm_L = ('crl_arm__L', carla.Transform(location=carla.Location(x=0.17, z=1.48), rotation=carla.Rotation(yaw=-110, pitch=-140, roll=0)))
            forearm_L = ('crl_forearm__L', carla.Transform(location=carla.Location(x=0.18, y=0.22, z=1.35), rotation=carla.Rotation(yaw=140, roll=180, pitch=20)))
            hand_L = ('crl_hand__L', carla.Transform(location=carla.Location(x=0.04, y=0.34, z=1.40), rotation=carla.Rotation(yaw=170, pitch=-50)))
            arm_R = ('crl_arm__R', carla.Transform(location=carla.Location(x=-0.17, y=0.0, z=1.48), rotation=carla.Rotation(yaw=90, roll=90, pitch=70)))
            forearm_R = ('crl_forearm__R', carla.Transform(location=carla.Location(x=-0.17, y=-0.07, z=1.29), rotation=carla.Rotation(yaw=-90, pitch=70)))
            neck = ('crl_neck__c', carla.Transform(location=carla.Location(x=0, y=0.0, z=1.55), rotation=carla.Rotation(yaw=180, roll=50, pitch=0)))
            return [arm_L, forearm_L, hand_L, arm_R, forearm_R, neck]

