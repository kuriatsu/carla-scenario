#!/usr/bin/env python
"""
Welcome to CARLA scenario controller.
"""
import carla
import xml.etree.ElementTree as ET
import argparse
import warnings
from actors.vehicle import Vehicle
import math

class WarpVehicle(Vehicle):
    def __init__(self, world, scenario_id, blueprint):
        super().__init__(world, scenario_id, blueprint)

    def move(self, xml):
        self.waypoints = xml.findall("waypoint")
        target_transform = self.calcTargetTransform()
        self.commands = [carla.command.ApplyTransform(self.world_id, target_transform)] 
        self.waypoints.pop(0)
        return

    def updateCommand(self):
        if self.waypoints:
            target_transform = self.calcTargetTransform()
            self.commands = [carla.command.ApplyTransform(self.world_id, target_transform)] 
            self.waypoints.pop(0)
        else:
            self.commands = []

        return

    def calcTargetTransform(self):
        transform = self.carla_actor.get_transform()
        if transform is None:
            print('cannot get transform: ' + control_actor.get('id'))
            return 0, 0, 0, 0

        waypoint = self.waypoints[0].text
        ## calc current motion vector and distance from current position to waypoint
        vector = carla.Vector3D(
            float(waypoint[0]) - transform.location.x,
            float(waypoint[1]) - transform.location.y,
            float(waypoint[2]) - transform.location.z
            )

        yaw = math.atan(
            (float(waypoint[1]) - transform.location.y) 
            / (float(waypoint[0]) - transform.location.x + 0.001)
            )

        if vector.x < 0.0 and vector.y < 0.0:
            yaw -= math.pi
        elif vector.x < 0.0 and vector.y > 0.0:
            yaw += math.pi

        yaw = math.degrees(yaw)
        return carla.Transform(carla.Location(waypoint[0], waypoint[1], waypoint[2]), carla.Rotation(0.0, yaw, 0.0))

