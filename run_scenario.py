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
            for itr in root.iter(tag):
                if itr.text is not None:
                    itr.text = [float(val) for val in itr.text.split(",")]
        return root



    def getWorldActors(self):
        for carla_actor in self.world.get_actors():
            if carla_actor.type_id == 'traffic.traffic_light':
                tl = TrafficLight(carla_actor.id, carla_action.get_location())
                spawned_actors[carla_actor.id] = tl

    def spawnActor(self, actions):
        batch = []
        for spawn in actions:
            if spawn.find("type") == "ai_vehicle":
                actor = AiVehicle(self.world, spawn.attrib.get("id"), self.blueprint)
            elif spawn.find("type") == "vehicle":
                actor = Vehicle(self.world, spawn.attrib.get("id"), self.blueprint)
            elif spawn.find("type") == "warp_vehicle":
                actor = WarpVehicle(self.world, spawn.attrib.get("id"), self.blueprint)
            elif spawn.find("type") == "ai_walker":
                actor = AiWalker(self.world, spawn.attrib.get("id"), self.blueprint)
            elif spawn.find("type") == "walker":
                actor = Walker(self.world, spawn.attrib.get("id"), self.blueprint)
            elif spawn.find("type") == "warp_walker":
                actor = WarpWalker(self.world, spawn.attrib.get("id"), self.blueprint)
            elif spawn.find("type") == "static":
                actor = Static(self.world, spawn.attrib.get("id"), self.blueprint)
            else:
                pass

            batch.append(actor.spawn())
            self.spawned_actors[spawn.attrib.get("id")] = actor

        results = self.client.apply_batch_sync(batch)
        for i, result in enumerate(results):
            scenario_id = actions[i].attrib.get("id")
            if result.error:
                warnings.warn(result.error)
                del self.spawned_actors[scenario_id]
            else:
                self.spawned_actors[scenario_id].postSpawn(result)


    def moveActor(self, actions):
        batch = []
        for a in actions:
            scenario_id = a.attrib.get("id")
            if scenario_id in self.spawned_actors.keys():
                batch.append(self.spawned_actors[scenario_id].move(a))

        results = self.client.apply_batch_sync(batch)

    def killActor(self, actions):
        batch = []
        for a in actions:
            scenario_id = a.attrib.get("id")
            if scenario_id in self.spawned_actors.keys():
                batch.append(self.spawned_actors[scenario_id].kill())
                del self.spawned_actors[scenario_id]

        results = self.client.apply_batch_sync(batch)

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

        batch = []
        for i in range(len(self.spawned_actors)):
            command = self.spawned_actors[i].step()
            if not command:
                del self.spawned_actors[i]

            batch.append(actor.step())
        self.client.apply_batch(batch)

        trigger_location = trigger.find("location")
        distance = (self.ego_pose.trigger_location.x - trigger_location.text[0]) ** 2 \
                   + (self.ego_pose.trigger_location.y - trigger_location.text[1]) ** 2

        if distance < float(trigger.attrib.get('thres')) ** 2:
            batch = []
            spawn_actors = {}
            self.killActor(trigger.findall('kill'))
            self.spawnActor(trigger.findall('spawn'))
            self.moveActor(trigger.findall('move'))
            self.moveActor(trigger.findall('trafficlight'))
            self.moveActor(trigger.findall('pose'))
            self.world.wait_for_tick()

            return 1
        else:
            return 0
                

def game_loop(args):

    # try:

    client = carla.Client(args.host, args.port)
    client.set_timeout(2.0)
    world = client.get_world()
    carla_scenario = CarlaScenario(world, client)

    scenario = carla_scenario.readFile(args.scenario_file)
    carla_scenario.spawnActor(scenario[0].findall('spawn'))
    carla_scenario.moveActor(scenario[0].findall('move'))

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
