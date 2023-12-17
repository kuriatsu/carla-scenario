#!/usr/bin/env python
"""
Welcome to CARLA scenario controller.
"""
import carla
import xml.etree.ElementTree as ET
import argparse
import warnings

class Static(Actor):
    def __init__(self, world, scenario_id, blueprint):
        super().__init__(world, scenario_id, blueprint)
        self.collision_range
        self.is_invinsible

    def getBlueprint(self, xml):
    # def getBlueprint(self, blueprint_list, name):

        if xml.find("blueprint") == "random":
            blueprint = random.choice(self.blueprint.filter("static.*"))
        else:
            try:
                blueprint = self.blueprint.find(name)
            except ValueError:
                warnings.warn(f"spcecified blueprint is not exist : {xml.find('blueprint').text}")
                blueprint = random.choice(self.blueprint.filter("static.*"))


        return blueprint

    def spawn(self, xml):

        blueprint = self.getBlueprint(self, xml)

        transform = xml.find('transform').text
        transform = carla.Transform(
                carla.Location(transform[0], transform[1], transform[2]), 
                carla.Rotation(transform[3], transform[4], transform[5])
                )

        self.collision_range = float(xml.find("collision_range").text)
        self.is_invinsible = True if xml.find("invinsible").text == "true" else False

        return carla.command.SpawnActor(blueprint, transform)

    def postSpawn(self, response):
        super().postSpawn(response)
        self.is_controlled = True

    def step(self, xml):
        super().step()

        transform = control_actor.get('actor').get_transform()
        dist = math.sqrt(
            (self.ego_pose.location.x - transform.location.x) ** 2
            + (self.ego_pose.location.y - transform.location.y) ** 2
            )
        if dist < float(control_actor.get('collision_range')):
            if control_actor.get('invincible'):
                vel = self.ego_vehicle.get_velocity()
                if math.sqrt(vel.x ** 2 + vel.y ** 2) * 3.6 < 5.0:
                    self.kill()
                    return 0
            else:
                self.kill()
                return 0

