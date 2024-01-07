#!/usr/bin/env python
"""
Welcome to CARLA scenario controller.
"""
import carla
import xml.etree.ElementTree as ET
import argparse
import warnings

class CarlaScenario():
    def __init__(self, world, client):

        self.client = client
        self.world = world
        self.scenario

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
        self.actor_batch = {} # id: command
        self.actors = {} 

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

    def step(self, trigger):
        """check Trigger and run action
        ~return~
        0 : not on the trigger
        1 : found the trigger
        2 : end of scenario
        3 : no scenario
        """
        if self.ego_vehicle is None:
            self.getEgoVehicle()
        else:
            self.ego_pose = self.ego_vehicle.get_transform()

        # for i in range(len(self.spawned_actors)):
        #     command = self.spawned_actors[i].step()
        #     if not command:
        #         del self.spawned_actors[i]

        #     batch.append(actor.step())
        # self.client.apply_batch(batch)

        trigger_location = trigger.find("location")
        distance = ((self.ego_pose.trigger_location.x - trigger_location.text[0]) ** 2 \
                   + (self.ego_pose.trigger_location.y - trigger_location.text[1]) ** 2) ** 0.5

        ## execute next scenario 
        if distance < float(trigger.attrib.get('thres')):
            for action in trigger:
                if action.tag == "location": continue

                actor_id = action.attrib.get("id")
                if action.tag == "spawn": 
                    self.spawned_actors[actor_id] = (self.actor_type[action.find("type")](self.world, actor_id, self.blueprint))
                else:
                    if actor_id not in self.spawned_actors:
                        warnings.warn(f"{actor_id} has not been spawned")
                        continue

                self.spawned_actors[actor_id].action(action)
                if self.spawned_actors[actor_id].commands:
                    for _ in range(len(self.spawned_actors[actor_id].commands)):
                        self.actor_batch.append(self.spawned_actors[actor_id])


            ## extract carla.command from actor instances
            batch = []
            for actor in self.actor_batch:
                batch.append(actor.commands[0])
                actor.commands.pop(0)

            ## execute command
            responses = self.client.apply_batch_sync(batch)

            ## handle response
            actor_batch_next = []
            for i, (response, actor) in enumerate(zip(responses, self.actor_batch)):
                if response.error:
                    warnings.warn(result.error)
                    self.spawned_actors.pop(actor.scenario_id)
                else:
                    actor.getResponse(response)
                    if actor.commands:
                        for _ in range(len(actor.commands)):
                            actor_batch_next.append(actor)

            ## remove stale batches from self.actor_batch
            self.actor_batch = actor_batch_next
            self.world.wait_for_tick()

            return 1

        else:
            return 0
                
    def initialize(self, scenario):
    
        self.spawned_actors = {**self.spawned_actors, **getTrafficLights(self.world)}
        self.spawnActor(step(scenario[0]))
        

def game_loop(args):

    # try:

    client = carla.Client(args.host, args.port)
    client.set_timeout(2.0)
    world = client.get_world()
    carla_scenario = CarlaScenario(world, client)

    scenario = carla_scenario.readFile(args.scenario_file)

    trigger_index = 0
    while len(scenario) > trigger_index:
        trigger_index += carla_scenario.step(scenario[trigger_index])
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
