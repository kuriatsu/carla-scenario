#!/usr/bin/env python

import glob
import os
import sys
try:
    sys.path.append(glob.glob('**/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================

import argparse
import random
import csv
import time
import carla
import logging
import math


class SpawnActor(object):

    def __init__(self):

        self.client = None
        self.world = None
        self.ego_vehicle = None
        self.ego_pose = None
        self.blueprintVehicles = None
        self.blueprintWalkers = None
        self.blueprintWalkerController = None
        self.actor_profile_list = []
        self.scenario_list = []
        self.spawned_actor_list = []
        self.control_actor_list = []

    def readFile(self, filename):

        file = open(filename, 'r')
        reader = csv.reader(file)
        header = next(reader)

        in_list = [row for row in reader]
        return in_list

    def getEgoCar(self):

        for carla_actor in self.world.get_actors():
            if carla_actor.type_id.startswith("vehicle"):
                if carla_actor.attributes.get('role_name') == 'ego_vehicle':
                    self.ego_vehicle = carla_actor


    def checkScenario(self):

        ego_pose = self.ego_vehicle.get_transform()
        intrusion_thres = 2.0

        for scenario in self.scenario_list:
            distance = (ego_pose.location.x - float(scenario[1])) ** 2 \
                       + (ego_pose.location.y - float(scenario[2])) ** 2
            if distance < intrusion_thres:
                if (scenario[4] == 'spawn'):
                    self.spawnActor(scenario[5:])
                    print("spawned {}".format(scenario[5:]))
                elif (scenario[4] == 'start'):
                    self.startActor(scenario[5:])
                    # del self.control_actor_list[:]
                    print("control {}".format(scenario[5:]))
                elif (scenario[4] == 'destroy'):
                    self.destroyActor(scenario[5:])
                    print("destroy {}".format(scenario[5:]))

                self.scenario_list.remove(scenario)

    def spawnActor(self, actor_id_list):

        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

        # we spawn the actor
        batch = []

        for actor_id in actor_id_list:
            # check whether the required actor id exists in the profile list or not
            if int(actor_id) > len(self.actor_profile_list):
                print("actor_id {} does not registered in the actor file".format(actor_id))
                actor_id_list.remove(actor_id)
                continue

            spawned_actor = {}
            # get target object from not aligned list
            actor = self.actor_profile_list[int(actor_id)]
            # add id to the list of currently existing actor under the world
            spawned_actor['id'] = int(actor_id)
            spawned_actor['attribute'] = actor[1]

            # spawn the walker object
            if (actor[1] == 'walker' or actor[1] == 'ai_walker'):

                blueprint = random.choice(self.blueprintWalkers)

                # set as not invencible
                if blueprint.has_attribute('is_invincible'):
                    blueprint.set_attribute('is_invincible', 'false')

                # set walker's speed
                spawned_actor['walk'] = 1.0
                spawned_actor['run'] = 3.0
                spawned_actor['free'] = random.uniform(1.0, 3.0)

                transform = carla.Transform(carla.Location(float(actor[2]), float(actor[3]), float(actor[4])), carla.Rotation(float(actor[5]), float(actor[6]), float(actor[7])))
                batch.append(carla.command.SpawnActor(blueprint, transform))


            # spawn the vehicle object
            elif (actor[1] == 'vehicle' or actor[1] == 'ai_vehicle'):
                blueprint = random.choice(self.blueprintVehicles)

                if blueprint.has_attribute('color'):
                    color = random.choice(blueprint.get_attribute('color').recommended_values)
                    blueprint.set_attribute('color', color)

                if blueprint.has_attribute('driver_id'):
                    driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                    blueprint.set_attribute('driver_id', driver_id)

                spawned_actor['slow'] = 8.0
                spawned_actor['fast'] = 18.0
                spawned_actor['free'] = random.randint(1, 20)

                blueprint.set_attribute('role_name', 'driver')
                transform = carla.Transform(carla.Location(float(actor[2]), float(actor[3]), float(actor[4])), carla.Rotation(float(actor[5]), float(actor[6]), float(actor[7])))
                batch.append(carla.command.SpawnActor(blueprint, transform))

            self.spawned_actor_list.append(spawned_actor)

        # conduct spawn
        results = self.client.apply_batch_sync(batch)
        for i in range(len(results)):
            if results[i].error:
                print(results[i].error)
            else:
                self.spawned_actor_list[i]["world_id"] = results[i].actor_id
                # print("spawned", self.spawned_actor_list[i])

        # we spawn the ai walker controller
        batch = []
        ai_walker_ids = []
        for i, spawned_actor in enumerate(self.spawned_actor_list):
            if spawned_actor['attribute'] == 'ai_walker':
                ai_walker_ids.append(i)
                batch.append(carla.command.SpawnActor(self.blueprintWalkerController, carla.Transform(), spawned_actor['world_id']))
        results = self.client.apply_batch_sync(batch, True)

        # conduct spawn
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                self.spawned_actor_list[ai_walker_ids[i]]['controller'] = results[i].actor_id
        # wait for a tick to ensure client receives the last transform of the walkers we have just created
        self.world.wait_for_tick()


    def startActor(self, actor_id_list):


        for actor_id in actor_id_list:
            for spawned_actor in self.spawned_actor_list:
                if spawned_actor['id'] == int(actor_id):

                    if spawned_actor['attribute'] == 'ai_walker':
                        actor = self.world.get_actor(spawned_actor['controller'])
                        actor.start()
                        actor.go_to_location(self.world.get_random_location_from_navigation())
                        actor.set_max_speed(spawned_actor[self.actor_profile_list[int(actor_id)][8]])

                    elif spawned_actor['attribute'] == 'ai_vehicle':
                        actor = self.world.get_actor(spawned_actor['world_id'])
                        actor.set_autopilot(True)

                    elif spawned_actor['attribute'] == 'walker' or spawned_actor['attribute'] == 'vehicle':
                        control_actor = {}
                        control_actor['actor'] = self.world.get_actor(spawned_actor['world_id'])
                        control_actor['attribute'] = spawned_actor['attribute']
                        goal = self.actor_profile_list[int(actor_id)][9:12]
                        control_actor['goal'] = carla.Location(float(goal[0]), float(goal[1]), float(goal[2]))
                        control_actor['speed'] = spawned_actor[self.actor_profile_list[int(actor_id)][8]]
                        del self.actor_profile_list[int(actor_id)][8:12]
                        self.control_actor_list.append(control_actor)



    def controlActor(self):
        debug = self.world.debug
        for control_actor in self.control_actor_list:
            if control_actor['actor'].is_alive == False:
                self.control_actor_list.remove(control_actor)
                continue

            else:
                actor = control_actor['actor']
                transform = actor.get_transform()
                vector = carla.Vector3D(control_actor['goal'].x - transform.location.x,
                                               control_actor['goal'].y - transform.location.y,
                                              control_actor['goal'].z)
                dist = math.sqrt(vector.x ** 2 + vector.y ** 2)
                vector.x = vector.x / dist
                vector.y = vector.y / dist

                debug.draw_arrow(begin=transform.location ,end=transform.location + carla.Location(vector.x, vector.y, 0.0), life_time=0.5)

            if control_actor['attribute'] == 'walker':
                # print(control_actor['actor'].id, math.sqrt(control_actor['actor'].get_velocity().x**2 + control_actor['actor'].get_velocity().y**2), math.sqrt(vector.x**2 + vector.y**2),control_actor['speed'])
                if dist < 0.5:
                    control = carla.WalkerControl(direction=carla.Vector3D(0.0,0.0,0.0),speed=0.0)
                    self.control_actor_list.remove(control_actor)
                else:
                    control = carla.WalkerControl(direction=vector, speed=control_actor['speed'])
                    control_actor['actor'].apply_control(control)

            elif control_actor['attribute'] == 'vehicle':
                # calc vel and rotation
                if dist < 1.0:
                    control_actor['actor'].set_velocity(carla.Vector3D(0.0,0.0,0.0))
                    self.control_actor_list.remove(control_actor)
                else:
                    alpha = math.atan(vector.y / vector.x) - transform.rotation.yaw
                    omega = 2 * control_actor['speed'] * math.sin(alpha) / dist

                    control = carla.VehicleControl()
                    velocity = carla.Vector3D()
                    velocity.x = vector.x * control_actor['speed']
                    velocity.y = vector.y  * control_actor['speed']
                    velocity.z = 0.0
                    control_actor['actor'].set_velocity(velocity)
                    control_actor['actor'].set_angular_velocity(carla.Vector3D(0.0, 0.0, omega))


    def destroyActor(self, actor_id_list):
        batch = []
        for actor_id in actor_id_list:
            for spawned_actor in self.spawned_actor_list:
                if spawned_actor['id'] == int(actor_id):
                    if spawned_actor['attribute'] == 'ai_walker':
                        self.world.get_actor(spawned_actor['controller']).stop()
                        batch.append(carla.command.DestroyActor(spawned_actor['controller']))

                    batch.append(carla.command.DestroyActor(spawned_actor['world_id']))
                    self.spawned_actor_list.remove(spawned_actor)
        self.client.apply_batch(batch)


    def game_loop(self, args):

        try:

            self.client = carla.Client(args.host, args.port)
            self.client.set_timeout(2.0)
            self.world = self.client.get_world()
            self.blueprintVehicles = self.world.get_blueprint_library().filter(args.filterv)
            self.blueprintWalkers = self.world.get_blueprint_library().filter(args.filterw)
            self.blueprintWalkerController = self.world.get_blueprint_library().find('controller.ai.walker')
            self.actor_profile_list = self.readFile(args.actor_file)
            self.scenario_list = self.readFile(args.scenario_file)
            self.getEgoCar()
            # self.world.set_pedestrians_cross_factor(100)

            # self.actor_profile_list.insert(0, ['0'])
            # print(self.actor_profile_list)
            while True:

                self.world.wait_for_tick()
                self.checkScenario()
                self.controlActor()
                time.sleep(0.1)

        finally:
            batch = []
            for carla_actor in self.world.get_actors():
                if carla_actor.type_id.startswith("vehicle") or carla_actor.type_id.startswith("walker"):
                    if carla_actor.attributes.get('role_name') != 'ego_vehicle':
                        batch.append(carla.command.DestroyActor(carla_actor.id))

            self.client.apply_batch(batch)
            print('exit')

def main():

    argparser = argparse.ArgumentParser( description = __doc__ )
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
        '-a', '--actor_file',
        metavar='A',
        default='../actor.csv',
        help='scenario file (default: actor.csv)')
    argparser.add_argument(
        '-s', '--scenario_file',
        metavar='S',
        default='/home/mad-carla/share/catkin_ws/src/carla_helper/scenario.csv',
        help='scenario file (default: scenario.csv)')
    argparser.add_argument(
        '--filterv',
        metavar='PATTERN',
        default='vehicle.*',
        help='vehicles filter (default: "vehicle.*")')
    argparser.add_argument(
        '--filterw',
        metavar='PATTERN',
        default='walker.pedestrian.*',
        help='pedestrians filter (default: "walker.pedestrian.*")')
    args = argparser.parse_args()


    try:
        spawn_actor = SpawnActor()
        spawn_actor.game_loop(args)
    finally:
        print('EXIT')


if __name__ == '__main__':
    main()
