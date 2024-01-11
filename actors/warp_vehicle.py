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

    def calcTargetTransform(self):
        transform = self.carla_actor.get_transform()
        if transform is None:
            print('cannot get transform: ' + control_actor.get('id'))
            return 0, 0, 0, 0

        point = self.waypoints[0].text
        yaw = math.atan(
            (float(point[1]) - transform.location.y) 
            / (float(point[0]) - transform.location.x)
            )
        yaw = math.degrees(yaw)
        return carla.Transform(carla.Location(point[0], point[1], point[2]), carla.Rotation(0.0, yaw, 0.0))

    def getResponse(self, response):
        super().getResponse(response)
        if self.waypoints:
            target_transform = self.calcTargetTransform()
            self.commands = [carla.command.ApplyTransform(self.world_id, target_transform)] 
            self.waypoints.pop(0)
        else:
            self.commands = []

        return
