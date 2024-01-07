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
        self.action_dict = {
                "spawn": self.spawn,
                "move" : self.move,
                "kill" : self.kill,
                }

        self.world_id = None
        self.carla_actor = None
        self.commands = []

    def getBlueprint(self, xml):
        pass

    def action(self, xml):
        if not actor.is_alive:
            print(f"actor {self.world_id} is dead")
            self.commands = []

        if xml.tag in self.action_dict.keys():
            self.action_dict[xml.tag](xml):

        else:
            print(f"unknown action {xml.tag}")
            self.commands = []

    def getResponse(self, response):
        if self.world_id is None:
            self.world_id = response.actor_id
            self.carla_actor = self.world.get_actor(self.world_id)
            print(f"spawned scenario_id: {self.scenario_id}, world_id: {self.world_id}");
            
        return None

    def spawn(self):
        return

    def move(self, xml):
        return

    def kill(self, xml):
        self.commands = [carla.command.DestroyActor(world_id)]
        return

