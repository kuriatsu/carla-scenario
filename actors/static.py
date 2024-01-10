#!/usr/bin/env python
"""
Welcome to CARLA scenario controller.
"""
import carla
import xml.etree.ElementTree as ET
import argparse
import warnings
from actors.actor import Actor

class Static(Actor):
    def __init__(self, world, scenario_id, blueprint):
        super().__init__(world, scenario_id, blueprint)
        self.collision_range
        self.is_invinsible
        self.ego_vehicle

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

        transform = xml.find("transform").text
        transform = carla.Transform(
                carla.Location(transform[0], transform[1], transform[2]), 
                carla.Rotation(transform[3], transform[4], transform[5])
                )

        self.collision_range = float(xml.find("collision_range").text)
        self.is_invinsible = True if xml.find("invinsible").text == "true" else False

        self.commands = [carla.command.SpawnActor(blueprint, transform)]
        return

    def getResponse(self, response):
        super().postSpawn(response)

        if self.is_invincible: return 

        if self.ego_vehicle is None:
            self.ego_vehicle = self.getEgoVehicle()

        ego_location = self.ego_vehicle.get_location()
        location = self.actor.get_location()
        dist = ((ego_location.x - location.x) ** 2
             + (ego_location.y - location.y) ** 2) ** 0.5

        if dist < float(self.collision_range):
            vel = self.ego_vehicle.get_velocity()
            if (vel.x ** 2 + vel.y ** 2) ** 0.5 * 3.6 < 5.0:
                self.kill()
                return



    def getEgoVehicle(self):
        """ get ego vehicle actor
        [return]
        self.ego_vehicle : actor of echo vehicle
        """

        for carla_actor in self.world.get_actors():
            if carla_actor.attributes.get("role_name") in ["ego_vehicle", "hero"]:
                self.ego_vehicle = carla_actor
                print("find ego_vehicle")
                return
