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
        self.actor_dict = {
                "ai_vehicle": AiVehicle,
                "vehicle": Vehicle,
                "warp_vehicle": WarpVehicle,
                "ai_walker": AiWalker,
                "walker": Walker,
                "warp_walker": WarpWalker,
                "static": Static,
                "traffic_light": TrafficLight,
                }
        self.batch = {} # id: command
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


    def getWorldActors(self):
        for actor in self.world.get_actors():
            if actor.type_id == 'traffic.traffic_light':
                self.actors[actor.id] = TrafficLight(actor.id, actor.get_location())


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

        for i in range(len(self.spawned_actors)):
            command = self.spawned_actors[i].step()
            if not command:
                del self.spawned_actors[i]

            batch.append(actor.step())
        self.client.apply_batch(batch)

        trigger_location = trigger.find("location")
        distance = (self.ego_pose.trigger_location.x - trigger_location.text[0]) ** 2 \
                   + (self.ego_pose.trigger_location.y - trigger_location.text[1]) ** 2

        ## execute action
        if distance < float(trigger.attrib.get('thres')) ** 2:
            for action in trigger:
                if action.tag == "location": continue

                id = action.attrib.get("id")
                if action.tag == "spawn" : 
                    self.actors[id] = (self.actor_dict[action.find("type")](self.world, id, self.blueprint))
                else:
                    if id not in self.actors:
                        warnings.warn(f"{id} has not been spawned")
                        continue

                self.actors[id].action(action)
                if self.actors[id].command is not None:
                    self.batch.append(self.actors[id])

            for actor in self.batch:
                responses = self.client.apply_batch_sync([actor.command for actor in self.batch])
            remove_batch = []
            for i, (response, actor) in enumerate(zip(responses, self.batch)):
                if response.error:
                    warnings.warn(result.error)
                    self.actors.pop(actor.scenario_id)
                    remove_batch.append(i) 
                else:
                    actor.getResponse(response)
                    remove_batch.append(i) 
                    if actor.command is not None:
                        self.batch.append(actor.command)

            ## remove stale batches from self.batch
            self.batch = [b for i, b in enumerate(self.batch) if i not in remove_batch]
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
