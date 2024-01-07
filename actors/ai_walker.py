#!/usr/bin/env python
"""
Welcome to CARLA scenario controller.
"""
import carla
import xml.etree.ElementTree as ET
import argparse
import warnings

class AiWalker(Walker):
    def __init__(self, world, scenario_id, blueprint):
        super().__init__(world, scenario_id, blueprint)
        self.ai_walker_bp = self.blueprint.find('controller.ai.walker')
        self.ai_walker = None
        self.ai_walker_id = None

    def getBlueprint(self, xml):

        super().getBlueprint(xml)
        if spawn.find('type').text == 'ai_walker':
            ai_walkers_list.append(spawn)

        return blueprint

    def getResponse(self, response):
        super().getResponse(response)

        try:
            self.commands = [carla.command.SpawnActor(self.ai_walker_bp, carla.Transform(), attach_to=self.actor)]
            return
        except:
            print(f"failed to attach ai walker to {scenario_id}")
            return

        ## get ai walker id acter spawning ai_walker (assume self.ai_walker=None, self.carla_actor exists)
        if self.world_id is not None and self.ai_walker_id is None:
            self.ai_walker_id = response.actor_id
            self.ai_walker = self.world.get_actor(self.ai_walker_id)

    
    def move(self, xml):
        try:
            self.ai_walker.start()
            self.ai_walker.go_to_location(self.world.get_random_location_from_navigation())
            self.ai_walker.set_max_speed(float(xml.find("speed").text))
        except:
            print('cannot start AI walker. world_id: ', world_id)
            return
