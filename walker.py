#!/usr/bin/env python
"""
Welcome to CARLA scenario controller.
"""
import carla
import xml.etree.ElementTree as ET
import argparse
import warnings

class Walker(Actor):
    def __init__(self, world, scenario_id, blueprint):
        super().__init__(world, scenario_id, blueprint)
        self.walker_posture_lib
        self.waypoints

    def getBlueprint(self, xml):
    # def getBlueprint(self, blueprint_list, name):

        if xml.find("blueprint") == "random":
            blueprint = random.choice(self.blueprint.filter("walker.*"))
        else:
            try:
                blueprint = self.blueprint.find(name)
            except ValueError:
                warnings.warn(f"spcecified blueprint is not exist : {xml.find('blueprint').text}")
                blueprint = random.choice(self.blueprint.filter("walker.*"))

        blueprint.set_attribute('role_name', spawn.attrib.get(self.scenario_id))
        # set as not invencible
        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'false')

        return blueprint

    def spawn(self, xml):

        blueprint = self.getBlueprint(self, xml)

        transform = xml.find('transform').text
        transform = carla.Transform(
                carla.Location(transform[0], transform[1], transform[2]), 
                carla.Rotation(transform[3], transform[4], transform[5])
                )

        return carla.command.SpawnActor(blueprint, transform)

    def move(self, xml):
        self.is_controlled = True
        self.waypoints = xml.findall("waypoint")

    def calcControl(control_actor):
        # some information for movng
        transform = control_actor.get('actor').get_transform()
        if transform is None:
            print('cannot get transform: ' + control_actor.get('id'))
            return 0, 0, 0, 0

        point = control_actor.get('waypoints')[0].text
        speed = float(control_actor.get('waypoints')[0].attrib.get('speed'))
        # calc culent motion vector and distance to the target
        vector = carla.Vector3D(
            float(point[0]) - transform.location.x,
            float(point[1]) - transform.location.y,
            float(point[2]) - transform.location.z
            )
        dist = math.sqrt(vector.x ** 2 + vector.y ** 2)
        # normalize vector to calcurate velocity
        vector.x = vector.x / dist
        vector.y = vector.y / dist
        # debug.draw_arrow(begin=transform.location ,end=transform.location + carla.Location(vector.x, vector.y, 0.0), life_time=0.5)
        return vector, speed, dist, transform.rotation.yaw

    def step(self, xml):
        super().step()
        vector, speed, dist, _ = calcControl(control_actor)

        # shift to the next waypoint
        if dist < 1.0:
            del self.waypoints[0]

        return carla.command.ApplyWalkerControl(world_id, carla.WalkerControl(direction=vector, speed=speed))
