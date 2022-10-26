#!/usr/bin/env python
"""
Welcome to CARLA scenario controller.
"""
import glob
import os
import sys
try:
    sys.path.append(glob.glob('**/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
    sys.path.append("./")
except IndexError:
    pass

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================
import carla
import xml.etree.ElementTree as ET
import argparse
import warnings
import random
import math
import time
import datetime
from pose import PoseDefine

class ScenarioXML(object):

    def __init__(self, client, world, scenario_file, pedestrian_cross_factor):

        # args
        self.client = client
        self.world = world
        self.ego_vehicle = None
        self.ego_pose = None
        self.scenario = self.readFile(scenario_file)
        self.intrusion_thres = 3.0
        self.trigger_index = 1
        self.blueprint = None
        self.control_actor_list = []
        self.trafficlight_list = self.getTraffcLight()
        self.pose_define = PoseDefine()
        self.spawned_static_objects_id = []
        self.world.set_pedestrians_cross_factor(pedestrian_cross_factor)


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
        edit_tag_list = ['transform', 'location', 'waypoint'] # tags of list which you want to use as float list, not text

        # convert text list to float list
        for tag in edit_tag_list:
            for itr in root.iter(tag):
                if itr.text is not None:
                    itr.text = [float(val) for val in itr.text.split(',')]
        return root


    def getEgoCar(self):
        """ get ego vehicle actor
        [return]
        self.ego_vehicle : actor of echo vehicle
        """

        for carla_actor in self.world.get_actors():
            if carla_actor.attributes.get('role_name') == 'ego_vehicle':
                self.ego_vehicle = carla_actor
                print('find ego_vehicle')
                return


        print('could not find ego_vehicle')

    def checkTrigger(self):
        """check Trigger and run action
        ~return~
        0 : not on the trigger
        1 : found the trigger
        2 : end of scenario
        3 : no scenario
        """

        if self.scenario is not None and len(self.scenario) > 1:
            trigger = self.scenario[self.trigger_index] # get trigger tree from scenario tree
            distance = (self.ego_pose.location.x - trigger[0].text[0]) ** 2 \
                       + (self.ego_pose.location.y - trigger[0].text[1]) ** 2

            if distance < float(trigger.attrib.get('thres')) ** 2:
                print("trigger: {}".format(self.trigger_index))
                self.killActor(trigger.findall('kill'))
                self.spawnActor(trigger.findall('spawn'))
                self.poseActor(trigger.findall("pose"))
                self.moveActor(trigger.findall('move'))
                self.controlTrafficLight(trigger.findall("trafficlight"))
                # self.poseActor(trigger.findall("pose"))
                self.trigger_index += 1
                if self.trigger_index == len(self.scenario):
                    return 2 # scenario finished

                return 1 # next trigger
            else:
                return 0 # ordinary loop
        elif len(self.scenario) == 1:
            return 0
        else:
            return 3 # no scenario

    def getBlueprint(self, type, name):
    # def getBlueprint(self, blueprint_list, name):

        if name == 'random':
            if 'walker' in type:
                blueprint = random.choice(self.blueprint.filter('walker.*'))
            elif 'vehicle' in type:
                blueprint = random.choice(self.blueprint.filter('vehicle.*'))
            elif 'static' in type:
                blueprint = random.choice(self.blueprint.filter('static.*'))
            else:
                warnings.warn('spcecified blueprint is not exist : {}'.format(actor.find('blueprint').text))
                return

        else:
            try:
                blueprint = self.blueprint.find(name)
            except ValueError:
                warnings.warn('spcecified blueprint is not exist : {}'.format(actor.find('blueprint').text))
                return

        return blueprint

    def getTraffcLight(self):

        trafficlight_list = []

        for carla_actor in self.world.get_actors():
            if carla_actor.type_id == 'traffic.traffic_light':
                trafficlight = {}
                trafficlight['id'] = carla_actor.id
                trafficlight['location'] = carla_actor.get_location()
                trafficlight_list.append(trafficlight)

        return trafficlight_list


    def spawnActor(self, spawn_list):
        # we spawn the actor
        batch = []
        ai_walkers_list = []

        for spawn in spawn_list:
            actor_ids = {}
            blueprint = self.getBlueprint(spawn.find('type').text, spawn.find('blueprint').text)
            blueprint.set_attribute('role_name', spawn.attrib.get('id'))
            # set blueprint for walker
            if 'walker' in spawn.find('type').text:

                if spawn.find('type').text == 'ai_walker':
                    ai_walkers_list.append(spawn)

                # set as not invencible
                if blueprint.has_attribute('is_invincible'):
                    blueprint.set_attribute('is_invincible', 'false')

            # set blueprint for vehicle
            elif 'vehicle' in spawn.find('type').text:

                if blueprint.has_attribute('color'):
                    color = random.choice(blueprint.get_attribute('color').recommended_values)
                    blueprint.set_attribute('color', color)

                if blueprint.has_attribute('driver_id'):
                    driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                    blueprint.set_attribute('driver_id', driver_id)

            # append command to spawn actor to batch
            buf = spawn.find('transform').text
            transform = carla.Transform(carla.Location(buf[0], buf[1], buf[2]), carla.Rotation(buf[3], buf[4], buf[5]))

            if spawn.attrib.get('id') == "ego_vehicle" and self.ego_vehicle is not None:
                self.ego_vehicle.set_transform(transform)
                continue

            batch.append(carla.command.SpawnActor(blueprint, transform))

        # conduct spawn actor
        results = self.client.apply_batch_sync(batch)
        for i in range(len(results)):
            spawned_actor = spawn_list[i]
            if results[i].error:
                print("spawn failed: " + spawned_actor.attrib.get('id') + results[i].error)
                if spawned_actor in ai_walkers_list:
                    ai_walkers_list.remove(spawned_actor)
                for trigger in self.scenario.findall('trigger'):
                    for pose in trigger.findall('pose'):
                        if pose.attrib.get('id') == spawned_actor.attrib.get('id'):
                            trigger.remove(pose)
                    for move in trigger.findall('move'):
                        if move.attrib.get('id') == spawned_actor.attrib.get('id'):
                            trigger.remove(move)
                    for kill in trigger.findall('kill'):
                        if kill.attrib.get('id') == spawned_actor.attrib.get('id'):
                            trigger.remove(kill)

            else:
                world_id = ET.SubElement(spawned_actor, 'world_id')
                world_id.text = results[i].actor_id
                print("spawned : " + spawned_actor.attrib.get('id') + ', ' + str(world_id.text));
                logger.info('{},{}'.format(spawned_actor.attrib.get('id'), str(world_id.text)))
                if spawned_actor.find('type').text == 'static':
                    control_actor = {}
                    control_actor['actor'] = self.world.get_actor(world_id.text)
                    control_actor['type'] = 'static'
                    control_actor['collision_range'] = spawned_actor.find('collision_range').text
                    control_actor['invincible'] = True if spawned_actor.find('invincible').text == 'true' else False
                    self.control_actor_list.append(control_actor)


        # append command to spawn ai_controller to batch
        batch = []
        for ai_walker in ai_walkers_list:
            batch.append(carla.command.SpawnActor(self.blueprint.find('controller.ai.walker'), carla.Transform(), ai_walker.find('world_id').text))
        results = self.client.apply_batch_sync(batch, True)

        # conduct spawn ai controller
        for i in range(len(results)):
            if (results[i].error):
                warnings.warn(results[i].error)
            else:
                ai_controller_id = ET.SubElement(ai_walkers_list[i], 'ai_controller_id')
                ai_controller_id.text = results[i].actor_id
        # wait for a tick to ensure client receives the last transform of the walkers we have just created
        self.world.wait_for_tick()


    def poseActor(self, pose_list):

        control = carla.WalkerBoneControl()
        pose_define = PoseDefine()
        for pose in pose_list:
            for spawn in self.scenario.iter('spawn'):
                if pose.attrib.get('id') == spawn.attrib.get('id'):
                    actor = self.world.get_actor(spawn.find('world_id').text)
                    control.bone_transforms = pose_define.pose_dict.get(pose.find('form').text)()
                    actor.apply_control(control)
                    self.world.wait_for_tick()
                    # time.sleep(1)
                    actor.apply_control(control)
                    print('pose: '+ spawn.attrib.get('id'))


    def moveActor(self, move_elem_list):

        def moveAiWalker(world_id, speed):
            try:
                actor = self.world.get_actor(world_id)
                actor.start()
                actor.go_to_location(self.world.get_random_location_from_navigation())
                actor.set_max_speed(speed)
            except:
                print('cannot start AI walker. world_id: ', world_id)
                return

        def moveAiVehicle(world_id):
            try:
                actor = self.world.get_actor(world_id)
                actor.set_autopilot(True)
            except:
                print('cannot start AI vehicle. world_id: ', world_id)
                return

        def moveInnocentActor(id, world_id, type, waypoints):
            """add motion information to the stack
            world_id : actir id under the carla
            """
            control_actor = {}
            control_actor['actor'] = self.world.get_actor(world_id)
            control_actor['type'] = type
            control_actor['id'] = id
            control_actor['waypoints'] = waypoints
            self.control_actor_list.append(control_actor)


        for move_elem in move_elem_list:
            for spawn_elem in self.scenario.iter('spawn'):
                if move_elem.attrib.get('id') == spawn_elem.attrib.get('id'):
                    type = spawn_elem.find('type').text
                    if type == 'ai_walker':
                        moveAiWalker(spawn_elem.find('ai_controller_id').text, float(move_elem.find('waypoint').attrib.get('speed')))
                    elif type == 'ai_vehicle':
                        moveAiVehicle(spawn_elem.find('world_id').text)
                    elif type == 'walker' or type == 'vehicle':
                        moveInnocentActor(move_elem.attrib.get('id'), spawn_elem.find('world_id').text, type, move_elem.findall('waypoint')) # take over world info in spawn element and move info in move element
                    # elif type == 'static':
                    #     moveStaticActor(spawn_elem.find('world_id').text, type) # take over world info in spawn element and move info in move element
                    print("move: " + spawn_elem.attrib.get("id"));


    def controlActor(self):
        """control actor in loop
        [args]
        self.control_actor_list : actor dictionary for control them in loop

        """

        def calcControl(control_actor):
            # some information for movng
            transform = control_actor.get('actor').get_transform()
            if transform is None:
                print('cannot get transform: ' + control_actor.get('id'))
                return 0, 0, 0, 0

            point = control_actor.get('waypoints')[0].text
            speed = float(control_actor.get('waypoints')[0].attrib.get('speed'))
            # calc culent motion vector and distance to the target
            vector = carla.Vector3D(
                float(point[0]) - transform.location.x,
                float(point[1]) - transform.location.y,
                float(point[2]) - transform.location.z
                )
            dist = math.sqrt(vector.x ** 2 + vector.y ** 2)
            # normalize vector to calcurate velocity
            vector.x = vector.x / dist
            vector.y = vector.y / dist
            # debug.draw_arrow(begin=transform.location ,end=transform.location + carla.Location(vector.x, vector.y, 0.0), life_time=0.5)
            return vector, speed, dist, transform.rotation.yaw


        batch = []
        # debug = self.world.debug
        for control_actor in self.control_actor_list:
            if control_actor.get('actor').is_alive == False:
                self.control_actor_list.remove(control_actor)
                print("Innocent actor ", control_actor['actor'].id, "is dead")
                continue

            if control_actor['type'] == 'walker' and control_actor.get('waypoints'):
                vector, speed, dist, _ = calcControl(control_actor)

                # shift to the next waypoint
                if dist < 1.0:
                    del control_actor.get('waypoints')[0]

                batch.append(carla.command.ApplyWalkerControl(control_actor.get('actor').id, carla.WalkerControl(direction=vector, speed=speed)))

            elif control_actor['type'] == 'vehicle' and control_actor.get('waypoints'):
                # calc vel and rotation
                vector, speed, dist, yaw = calcControl(control_actor)
                if control_actor['type'] == 'vehicle' and dist < 5.0:
                    del control_actor.get('waypoints')[0]

                # calc velocity
                velocity = carla.Vector3D()
                velocity.x = vector.x * speed
                velocity.y = vector.y  * speed
                velocity.z = 0.0

                # calc angular velocity
                # define angle (radian)
                objective_angle = math.atan(vector.y / vector.x)
                objective_angle -= math.pi if vector.y < 0.0 else 0.0
                current_angle = math.radians(yaw)

                # difference between objective and current angle
                alpha = objective_angle - current_angle

                # take care the change from 180 -> -180
                if alpha > math.pi:
                    alpha = alpha - 2*math.pi
                elif alpha < -math.pi:
                    alpha = 2*math.pi - alpha

                # calcurat omega (100 is param)
                omega = 2 * 100 * speed * math.sin(alpha) / dist

                batch.append(carla.command.ApplyTargetVelocity(control_actor.get('actor').id, velocity))
                batch.append(carla.command.ApplyTargetAngularVelocity(control_actor.get('actor').id, carla.Vector3D(0.0, 0.0, omega)))

                # debug = self.world.debug
                # debug.draw_arrow(
                # begin=control_actor.get('actor').get_location() + carla.Location(z=vector.z + 3),
                # end=control_actor.get('actor').get_location() + carla.Location(x=vector.x, y=vector.y, z=vector.z + 3),
                # life_time=0.5
                # )
                # debug.draw_string(
                # location=control_actor.get('actor').get_location() + carla.Location(z=vector.z + 3),
                # text=str(omega),
                # life_time=0.1
                # )

            # update distination if move has multiple targets

            elif control_actor['type'] == 'static':
                transform = control_actor.get('actor').get_transform()
                dist = math.sqrt(
                    (self.ego_pose.location.x - transform.location.x) ** 2
                    + (self.ego_pose.location.y - transform.location.y) ** 2
                    )
                # print('static', dist)
                if dist < float(control_actor.get('collision_range')):
                    if control_actor.get('invincible'):
                        vel = self.ego_vehicle.get_velocity()
                        if math.sqrt(vel.x ** 2 + vel.y ** 2) * 3.6 < 5.0:
                            control_actor.get('actor').destroy()
                            self.control_actor_list.remove(control_actor)
                    else:
                        self.control_actor_list.remove(control_actor)
                        batch.append(carla.command.DestroyActor(control_actor.get('actor').id))

            # something without next waypoint
            else:
                print('{} reached goal and killed '.format(control_actor.get('id')))
                self.control_actor_list.remove(control_actor)
                batch.append(carla.command.DestroyActor(control_actor.get('actor').id))

        self.client.apply_batch(batch)


    def killActor(self, death_note):
        batch = []
        for death in death_note:
            for spawn in self.scenario.iter('spawn'):
                if death.attrib.get('id') == spawn.attrib.get('id'):
                    if spawn.find('type').text == 'ai_walker':
                        ai_controller_id = spawn.find('ai_controller_id').text
                        self.world.get_actor(ai_controller_id).stop()
                        batch.append(carla.command.DestroyActor(ai_controller_id))

                    print("kill: " + spawn.attrib.get("id"));
                    batch.append(carla.command.DestroyActor(spawn.find('world_id').text))
                    spawn.remove(spawn.find('world_id'))

        self.client.apply_batch(batch)


    def controlTrafficLight(self, light_list):
        for controll_light in light_list:
            for trafficlight in self.trafficlight_list:
                if ((controll_light.find('location').text[0] - trafficlight.get('location').x) ** 2 + (controll_light.find('location').text[1] - trafficlight['location'].y) ** 2 < 9.0):

                    actor = self.world.get_actor(trafficlight.get('id'))
                    print('control trafficlight')

                    if (controll_light.find('state').text == 'red'):
                        actor.set_state(carla.TrafficLightState.Red)
                        actor.set_red_time(float(controll_light.find('time').text))

                    elif (controll_light.find('state').text == 'green'):
                        actor.set_green_time(float(controll_light.find('time').text))
                        actor.set_state(carla.TrafficLightState.Green)

                    continue

    def __del__(self):
        batch = []
        for spawn in self.scenario.iter('spawn'):
            print('remove {}'.format(spawn.attrib.get('id')))

            if spawn.find('type').text == 'static' and spawn.find('world_id') is not None:
                print('remove {}'.format(spawn.attrib.get('id')))
                batch.append(carla.command.DestroyActor(spawn.find('world_id').text))

        for actor in self.world.get_actors():
            if actor.type_id.startswith("vehicle") or actor.type_id.startswith("walker") or actor.type_id.startswith("controller"):
                # if actor.attributes.get('role_name') != 'ego_vehicle':
                #    print('remove debris not in scenario')
                #    batch.append(carla.command.DestroyActor(actor.id))
                print('remove debris not in scenario')
                batch.append(carla.command.DestroyActor(actor.id))

        self.client.apply_batch(batch)



def game_loop(args):

    # try:

    client = carla.Client(args.host, args.port)
    client.set_timeout(2.0)
    world = client.get_world()
    sx = ScenarioXML(client, world, args.scenario_file, args.pedestrian_cross_factor)
    sx.blueprint = sx.world.get_blueprint_library()
    sx.getEgoCar()

    sx.spawnActor(sx.scenario[0].findall('spawn'))
    sx.moveActor(sx.scenario[0].findall('move'))
    sx.poseActor(sx.scenario[0].findall('pose'))
    sx.controlTrafficLight(sx.scenario[0].findall("trafficlight"))

    sx.getEgoCar()
    sx.ego_pose = sx.ego_vehicle.get_transform()

    while sx.checkTrigger() in [0, 1]:
        sx.world.wait_for_tick()
        if sx.ego_vehicle is None:
            sx.getEgoCar()
        else:
            sx.ego_pose = sx.ego_vehicle.get_transform()
            sx.controlActor()

        time.sleep(0.1)

    # finally:
    del sx

    # del sx



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
