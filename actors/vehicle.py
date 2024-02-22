#!/usr/bin/env python
"""
Welcome to CARLA scenario controller.
"""
import carla
import xml.etree.ElementTree as ET
import argparse
import warnings
import math
import random
from actors.actor import Actor

class Vehicle(Actor):
    def __init__(self, world, scenario_id, blueprint):
        super().__init__(world, scenario_id, blueprint)
        self.waypoints = None 

    def getBlueprint(self, xml):
    # def getBlueprint(self, blueprint_list, name):

        if xml.find("blueprint").text == "random":
            blueprint = random.choice(self.blueprint.filter("vehicle.*"))
        else:
            try:
                blueprint = self.blueprint.find(xml.find("blueprint").text)
            except ValueError:
                warnings.warn(f"spcecified blueprint is not exist : {xml.find('blueprint').text}")
                blueprint = random.choice(self.blueprint.filter("vehicle.*"))

        blueprint.set_attribute('role_name', self.scenario_id)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)

        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)

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

    def move(self, xml):
        self.waypoints = xml.findall("waypoint")

        vector, velocity, dist, omega = self.calcControl()
        if dist < 5.0:
            del self.waypoints[0]

        self.commands = [
            carla.command.ApplyTargetVelocity(self.world_id, velocity),
            carla.command.ApplyTargetAngularVelocity(self.world_id, carla.Vector3D(0.0, 0.0, omega))
            ]

        return

    def updateCommand(self):
        if self.waypoints and not self.commands:
            vector, velocity, dist, omega = self.calcControl()
            self.commands = [
                carla.command.ApplyTargetVelocity(self.world_id, velocity),
                carla.command.ApplyTargetAngularVelocity(self.world_id, carla.Vector3D(0.0, 0.0, omega))
                ]
            print(f"{self.scenario_id} target_vel: {velocity}")
            if dist < 5.0:
                self.waypoints.pop(0)

        elif not self.waypoints:
            self.commands = []

    def calcControl(self):
        transform = self.carla_actor.get_transform()
        if transform is None:
            print('cannot get transform: ' + self.scenario_id)
            return 0, 0, 0, 0

        waypoint = self.waypoints[0].text
        speed = float(self.waypoints[0].attrib.get('speed'))
        print(f"{self.scenario_id} current_vel: {self.carla_actor.get_velocity()}")

        ## calc current motion vector and distance from current position to waypoint
        vector = carla.Vector3D(
            float(waypoint[0]) - transform.location.x,
            float(waypoint[1]) - transform.location.y,
            float(waypoint[2]) - transform.location.z
            )
        dist = math.sqrt(vector.x ** 2 + vector.y ** 2)

        if dist < 3.0 or speed < 0.1:
            return vector, carla.Vector3D(0.0, 0.0, 0.0), dist, 0.0

        vector.x = vector.x / dist
        vector.y = vector.y / dist
        # debug = self.world.debug
        # debug.draw_arrow(begin=transform.location ,end=transform.location + carla.Location(vector.x*3, vector.y*3, 0.0), life_time=0.5)

        ## calcurate velocity
        velocity = carla.Vector3D()
        velocity.x = vector.x * speed
        velocity.y = vector.y * speed
        velocity.z = 0.0

        ## calcurate angular velocity
        objective_angle = math.atan(vector.y / vector.x)
        if vector.x < 0.0 and vector.y < 0.0:
            objective_angle -= math.pi
        elif vector.x < 0.0 and vector.y > 0.0:
            objective_angle += math.pi

        current_angle = math.radians(transform.rotation.yaw)

        ## difference between objective and current angle
        alpha = objective_angle - current_angle

        ## calcurate omega (100 is param)
        omega = 2 * 100 * speed * math.sin(alpha) / dist

        return vector, velocity, dist, omega 


