#!/usr/bin/env python
"""
Welcome to CARLA scenario controller.
"""
import carla
import xml.etree.ElementTree as ET
import argparse
import warnings

class Actor():
    def __init__(self, world, scenario_id, blueprint):
        self.scenario_id = scenario_id
        self.blueprint = blueprint
        self.world = world
        self.world_id
        self.carla_actor
        self.is_controlled = False

    def getBlueprint(self, xml):
    # def getBlueprint(self, blueprint_list, name):
        pass

    def spawn(self):
        pass

    def postSpawn(self, response):
        self.world_id = response.actor_id
        self.carla_actor = self.world.get_actor(self.world_id)
        print(f"spawned scenario_id: {self.scenario_id}, world_id: {self.world_id}");
        return

    def step(self, xml):
        if not actor.is_alive:
            print(f"actor {self.world_id} is dead")
            return 0

    def move(self, xml):
        pass

    def kill(self):
        return carla.command.DestroyActor(world_id)

