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

import carla
import argparse
# import pyperclip
import time
import math
import random

def main():

    print(sys.version)
    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(2.0)
    world = client.get_world()
    debug = world.debug

    # print(start)
    # get ego car

# ##########################
# spawn actor
############################
    # blueprint = world.get_blueprint_library().find('static.prop.plasticbag')
    # transform = carla.Transform(carla.Location(-210.104354858,1.57313215733,0.51499998569), carla.Rotation(0.0,75.1999435425,0.0))
    # world.spawn_actor(blueprint, transform)
# ##########################
# traffic light
############################
    # light_right = []
    # light_left = []
    #
    # for carla_actor in world.get_actors():
    #     # print(carla_actor)
    #     # if carla_actor.type_id.startswith("vehicle"):
    #     if carla_actor.attributes.get('role_name') == 'ego_vehicle':
    #         ego_vehicle = carla_actor
    #         ego_pose = ego_vehicle.get_transform()
    #
    #     if carla_actor.type_id == 'traffic.traffic_light':
    #         traffic_light = carla_actor
    #         light_pose = traffic_light.get_location()
    #         vector = light_pose - carla.Location(39.7840614319,-13.4922046661,0.137185171247)
    #         print(vector.x ** 2 + vector.y ** 2)
    #         if (vector.x ** 2 + vector.y ** 2 < 9.0):
    #             print("found_light!!!")
    #             traffic_light.set_state(carla.TrafficLightState.Green)
    #             traffic_light.set_green_time(30)

            #
            # if (abs(ego_pose.rotation.yaw - light_pose.rotation.yaw) < 30):
            # else:
            #     traffic_light.set_green_time(5)

    ###################
    ### speed test ####
    ###################
    # for carla_actor in world.get_actors():
    #     if carla_actor.attributes.get('role_name') == 'hero':
    #         ego_vehicle = carla_actor
    #         print("found")
    # batch = []
    # client.apply_batch(batch)
    # print("accel")
    # for i in range(0,5):
    #     batch.append(carla.command.ApplyVelocity(ego_vehicle, carla.Vector3D(100.0,0.0,0.0)))
    #     # print(ego_vehicle.get_velocity())
    #     # time.sleep(0.02)
    # # time.sleep(1.0)
    # # batch.append(carla.command.ApplyVelocity(ego_vehicle, carla.Vector3D(0.0,0.0,0.0)))
    # client.apply_batch(batch)
    # print("brake")
    # for i in range(0,5):
    #     print(ego_vehicle.get_velocity())
    #     time.sleep(0.02)

    #####################
    ####teleportation####
    #####################
    for carla_actor in world.get_actors():
        if carla_actor.attributes.get('role_name') == 'hero':
            hero = carla_actor
            print("found")

    # ego_vehicle.set_transform(carla.Transform(carla.Location(206.5,95.9,1.0),carla.Rotation(-0.0,-89.5,0.0)))
    hero.set_location(carla.Location(48.0550689697,145.219467163,0.0340613611042))


    ##########################
    #### obrtain position ####
    ##########################
        # key = raw_input('ENTER to copy transorm to clip_boad')
        # print(key)
        # key = ''
        # if key == '':
            # ego_pose = ego_vehicle.get_transform()
            # str = "{},{},{},{},{},{}".format(ego_pose.location.x, ego_pose.location.y, ego_pose.location.z, ego_pose.rotation.pitch, ego_pose.rotation.yaw, ego_pose.rotation.roll)
            # print(ego_pose.location.x, ego_pose.location.y, ego_pose.location.z, ego_pose.rotation.pitch, ego_pose.rotation.roll, ego_pose.rotation.yaw)
            # pyperclip.copy(str)
            # time.sleep(0.1)
        # elif key == 'q':
        #     print('exit')
        #     break
        # else:
        #     print('wrong key')

        # control = carla.WalkerControl(carla.Vector3D(-1.0,0.0,0.0), speed=3.5)
        # ego_vehicle.apply_control(control)
        # print(ego_vehicle.get_velocity())

    ##########################
    ####control skelton#######
    ##########################
    # for carla_actor in world.get_actors():
    #     if carla_actor.attributes.get('role_name') == 'ego_vehicle':
    #         ego_vehicle = carla_actor
    #         print("found")
        ############################
        #####right hand cellphone###
        ############################
    # control = carla.WalkerBoneControl()
    # actor = self.world.get_actor(world_id)
    # control = carla.WalkerBoneControl()
    #
    # arm_R = ('crl_arm__R', carla.Transform(location=carla.Location(x=-0.2, z=1.45), rotation=carla.Rotation(yaw=-60, pitch=-20)))
    # forearm_R = ('crl_forearm__R', carla.Transform(location=carla.Location(x=-0.31, y=0.16, z=1.54), rotation=carla.Rotation(yaw=-20, pitch=-158, roll=0)))
    # hand_R = ('crl_hand__R', carla.Transform(location=carla.Location(x=-0.13, y=0.1, z=1.62), rotation=carla.Rotation(roll=0, pitch=-170, yaw=90)))
    # arm_L = ('crl_arm__L', carla.Transform(location=carla.Location(x=0.17, y=0.0, z=1.48), rotation=carla.Rotation(yaw=-90, pitch=-70)))
    # forearm_L = ('crl_forearm__L', carla.Transform(location=carla.Location(x=0.17, y=-0.07, z=1.29), rotation=carla.Rotation(yaw=90, pitch=-70, roll=180)))
    # control.bone_transforms = [arm_R, forearm_R, hand_R, arm_L, forearm_L]
    # ego_vehicle.apply_control(control)

        ############################
        #####left hand cellphone###
    #     ############################
    # control = carla.WalkerBoneControl()
    # arm_L = ('crl_arm__L', carla.Transform(location=carla.Location(x=0.2, z=1.45), rotation=carla.Rotation(yaw=60, pitch=20)))
    # forearm_L = ('crl_forearm__L', carla.Transform(location=carla.Location(x=0.31, y=0.16, z=1.54), rotation=carla.Rotation(yaw=20, pitch=158, roll=0)))
    # hand_L = ('crl_hand__L', carla.Transform(location=carla.Location(x=0.13, y=0.1, z=1.62), rotation=carla.Rotation(pitch=20, yaw=90)))
    # # pinky_L = ('crl_handPinky__L', carla.Transform(location=carla.Location(x=0.11, y=0.08, z=1.67), rotation=carla.Rotation(pitch=120)))
    #
    # # shoulder_R = ('crl_shoulder__R', carla.Transform(location=carla.Location(x=-0.2, z=1.45), rotation=carla.Rotation(yaw=90)))
    # arm_R = ('crl_arm__R', carla.Transform(location=carla.Location(x=-0.17, y=0.0, z=1.48), rotation=carla.Rotation(yaw=90, roll=90, pitch=70)))
    # forearm_R = ('crl_forearm__R', carla.Transform(location=carla.Location(x=-0.17, y=-0.07, z=1.29), rotation=carla.Rotation(yaw=-90, pitch=70)))
    # control.bone_transforms = [arm_L, forearm_L, hand_L, arm_R, forearm_R]
    # ego_vehicle.apply_control(control)

        ############################
        #####right hand call###
        ############################
    # control = carla.WalkerBoneControl()
    #
    # arm_R = ('crl_arm__R', carla.Transform(location=carla.Location(x=-0.16, z=1.49), rotation=carla.Rotation(yaw=-70, pitch=50)))
    # forearm_R = ('crl_forearm__R', carla.Transform(location=carla.Location(x=-0.18, y=0.14, z=1.3), rotation=carla.Rotation(yaw=-140, roll=-30, pitch=-30)))
    # hand_R = ('crl_hand__R', carla.Transform(location=carla.Location(x=-0.04, y=0.265, z=1.40), rotation=carla.Rotation(roll=-30, pitch=128, yaw=0)))
    # arm_L = ('crl_arm__L', carla.Transform(location=carla.Location(x=0.17, y=0.0, z=1.48), rotation=carla.Rotation(yaw=-90, pitch=-70)))
    # forearm_L = ('crl_forearm__L', carla.Transform(location=carla.Location(x=0.17, y=-0.07, z=1.29), rotation=carla.Rotation(yaw=90, pitch=-70, roll=180)))
    # neck = ('crl_neck__c', carla.Transform(location=carla.Location(x=0, y=0.0, z=1.55), rotation=carla.Rotation(yaw=180, roll=50, pitch=0)))
    #
    #
    # control.bone_transforms = [arm_R, forearm_R, hand_R, arm_L, forearm_L, neck]
    # ego_vehicle.apply_control(control)


        ############################
        #####left hand call###
        ############################
    # control = carla.WalkerBoneControl()
    #
    # arm_L = ('crl_arm__L', carla.Transform(location=carla.Location(x=0.17, z=1.48), rotation=carla.Rotation(yaw=-110, pitch=-140, roll=0)))
    # forearm_L = ('crl_forearm__L', carla.Transform(location=carla.Location(x=0.18, y=0.22, z=1.35), rotation=carla.Rotation(yaw=140, roll=180, pitch=20)))
    # hand_L = ('crl_hand__L', carla.Transform(location=carla.Location(x=0.04, y=0.34, z=1.40), rotation=carla.Rotation(yaw=170, pitch=-50)))
    # arm_R = ('crl_arm__R', carla.Transform(location=carla.Location(x=-0.17, y=0.0, z=1.48), rotation=carla.Rotation(yaw=90, roll=90, pitch=70)))
    # forearm_R = ('crl_forearm__R', carla.Transform(location=carla.Location(x=-0.17, y=-0.07, z=1.29), rotation=carla.Rotation(yaw=-90, pitch=70)))
    # neck = ('crl_neck__c', carla.Transform(location=carla.Location(x=0, y=0.0, z=1.55), rotation=carla.Rotation(yaw=180, roll=50, pitch=0)))
    #
    # control.bone_transforms = [arm_L, forearm_L, hand_L, arm_R, forearm_R, neck]
    # ego_vehicle.apply_control(control)
    # ego_vehicle.apply_control(control)

if __name__ == '__main__':

    main()
