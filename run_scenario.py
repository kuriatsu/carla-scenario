#!/usr/bin/env python
"""
Welcome to CARLA scenario controller.
"""
import carla
import xml.etree.ElementTree as ET
import argparse
import warnings
import logging
import time
from actors.ai_walker import AiWalker
from actors.ai_vehicle import AiVehicle
from actors.warp_vehicle import WarpVehicle
from actors.warp_walker import WarpWalker
from actors.static import Static
from actors.traffic_light import TrafficLight
from actors.vehicle import Vehicle
from actors.walker import Walker

logger = logging.getLogger(__name__)

class CarlaScenario():
    def __init__(self, world, client):

        self.client = client
        self.world = world
        self.scenario = None

        self.blueprint = self.world.get_blueprint_library()

        self.ego_vehicle = None

        self.spawned_actors = {}
        self.control_actors = {}
        self.actor_type = {
                "ai_vehicle": AiVehicle,
                "vehicle": Vehicle,
                "warp_vehicle": WarpVehicle,
                "ai_walker": AiWalker,
                "walker": Walker,
                "warp_walker": WarpWalker,
                "static": Static,
                "traffic_light": TrafficLight,
                }
        self.actor_batch = [] # id: command

    def readFile(self, filename):
        """read xml file and return root of ElementTree, elements of some lists are changed from text to float.
        [args]
        filename: path to xml file
        [return]
        root of the ElementTree
        """
        logger.info(filename)
        tree = ET.parse(filename)
        root = tree.getroot()
        decode_tags = ["transform", "location", "waypoint"] # tags of list which you want to use as float list, not text

        # convert text list to float list
        for tag in decode_tags:
            for iter in root.iter(tag):
                if iter.text is not None:
                    iter.text = [float(val) for val in iter.text.split(",")]
        return root


    def getEgoVehicle(self):
        """ get ego vehicle actor
        [return]
        self.ego_vehicle : actor of echo vehicle
        """

        for carla_actor in self.world.get_actors():
            if carla_actor.attributes.get("role_name") == "ego_vehicle":
                self.ego_vehicle = carla_actor
                print("find ego_vehicle")
                return


        print("could not find ego_vehicle")


    def removeActors(self):
        batch = []
        for actor in self.world.get_actors():
            if actor.type_id.startswith("vehicle") or actor.type_id.startswith("walker"):
                 batch.append(carla.command.DestroyActor(actor.id))
        self.client.apply_batch_sync(batch)

    def executeActorCommand(self):
        ## extract carla.command from actor instances
        batch = []
        for actor in self.actor_batch:
            if not actor.commands: continue
            batch.append(actor.commands[0])
            actor.commands.pop(0)

        ## execute command
        responses = self.client.apply_batch_sync(batch)

        ## handle response
        actor_batch_next = []
        batch_next = []

        for i, (response, actor) in enumerate(zip(responses, self.actor_batch)):
            if response.error:
                print("spawn failed: ", actor.scenario_id)
                warnings.warn(response.error)
                self.spawned_actors.pop(actor.scenario_id)
            else:
                actor.getResponse(response)
                for command in actor.commands:
                    if command not in batch_next:
                        actor_batch_next.append(actor)
                        batch_next.append(command)

        ## remove stale batches from self.actor_batch
        self.actor_batch = actor_batch_next
        self.world.wait_for_tick()

    def step(self, next_scenario, ignore_trigger):
        """check Trigger and run action
        ~return~
        0 : not on the trigger
        1 : found the trigger
        2 : end of scenario
        3 : no scenario
        """
        self.executeActorCommand()

        ## execute next scenario 
        if ignore_trigger or self.checkTrigger(next_scenario.find("location"), next_scenario.attrib.get("thres")):

            ## spawn actor
            for action in next_scenario.findall("spawn"):
                actor_id = action.attrib.get("id")
                actor = self.actor_type[action.find("type").text](self.world, actor_id, self.blueprint)
                self.spawned_actors[actor_id] = actor 
                self.spawned_actors[actor_id].action(action)
                if self.spawned_actors[actor_id].commands:
                    for _ in range(len(self.spawned_actors[actor_id].commands)):
                        self.actor_batch.append(self.spawned_actors[actor_id])

            self.executeActorCommand()

            ## control/kill actor
            for action in next_scenario:
                if action.tag in ["location", "spawn"]: continue

                actor_id = action.attrib.get("id")
                if actor_id not in self.spawned_actors:
                    print(f"{actor_id} has not been spawned")
                    continue

                self.spawned_actors[actor_id].action(action)
                if len(self.spawned_actors[actor_id].commands) > 0:
                    for _ in range(len(self.spawned_actors[actor_id].commands)):
                        self.actor_batch.append(self.spawned_actors[actor_id])

            # self.executeActorCommand()

            return 1

        else:
            return 0
                
    def checkTrigger(self, trigger_location, thres):
        if self.ego_vehicle is None:
            self.getEgoVehicle()

        ego_pose = self.ego_vehicle.get_location()

        distance = ((ego_pose.x - trigger_location.text[0]) ** 2 \
                   + (ego_pose.y - trigger_location.text[1]) ** 2) ** 0.5

        print(f"trigger distance {distance}, thres {thres}")
        return distance < float(thres) 
    

def game_loop(args):

    # try:

    client = carla.Client(args.host, args.port)
    client.set_timeout(2.0)
    world = client.get_world()
    carla_scenario = CarlaScenario(world, client)

    scenario = carla_scenario.readFile(args.scenario_file)

    trigger_index = 0 
    carla_scenario.removeActors()
    trigger_index += carla_scenario.step(scenario[trigger_index], True)
    while len(scenario) > trigger_index:
        print(f"trigger {trigger_index}")
        trigger_index += carla_scenario.step(scenario[trigger_index], False)
        world.wait_for_tick()
        time.sleep(0.1)

    # finally:
    del carla_scenario


if __name__ == '__main__':

    argparser = argparse.ArgumentParser( description = __doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-s', '--scenario_file',
        metavar='S',
        default='/home/mad-carla/share/catkin_ws/src/carla_helper/scenario_test.xml',
        help='scenario file (default: scenario.xml)')
    argparser.add_argument(
        '-f', '--pedestrian_cross_factor',
        metavar='S',
        default=1.0,
        type=float,
        help='pedestrian cross rate 0.0-1.0')
    args = argparser.parse_args()

    game_loop(args)
