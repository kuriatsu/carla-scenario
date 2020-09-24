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
except IndexError:
    pass

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================
import carla
import argparse
import time

class PoseDefine(object):

    def __init__(self):
        self.pose_dict = {
        'phone_right' : self.posePhoneRight,
        'phone_left' : self.posePhoneLeft,
        'call_right' : self.poseCallRight,
        'call_left' : self.poseCallLeft,
        }

    def poseCallLeft(self):
            arm_L = ('crl_arm__L', carla.Transform(location=carla.Location(x=0.2, z=1.45), rotation=carla.Rotation(yaw=60, pitch=20)))
            forearm_L = ('crl_forearm__L', carla.Transform(location=carla.Location(x=0.31, y=0.16, z=1.54), rotation=carla.Rotation(yaw=20, pitch=158, roll=0)))
            hand_L = ('crl_hand__L', carla.Transform(location=carla.Location(x=0.13, y=0.1, z=1.62), rotation=carla.Rotation(pitch=20, yaw=90)))
            arm_R = ('crl_arm__R', carla.Transform(location=carla.Location(x=-0.17, y=0.0, z=1.48), rotation=carla.Rotation(yaw=90, roll=90, pitch=70)))
            forearm_R = ('crl_forearm__R', carla.Transform(location=carla.Location(x=-0.17, y=-0.07, z=1.29), rotation=carla.Rotation(yaw=-90, pitch=70)))
            return [arm_L, forearm_L, hand_L, arm_R, forearm_R]

    def poseCallRight(self):
            arm_R = ('crl_arm__R', carla.Transform(location=carla.Location(x=-0.2, z=1.45), rotation=carla.Rotation(yaw=-60, pitch=-20)))
            forearm_R = ('crl_forearm__R', carla.Transform(location=carla.Location(x=-0.31, y=0.16, z=1.54), rotation=carla.Rotation(yaw=-20, pitch=-158, roll=0)))
            hand_R = ('crl_hand__R', carla.Transform(location=carla.Location(x=-0.13, y=0.1, z=1.62), rotation=carla.Rotation(roll=0, pitch=-170, yaw=90)))
            arm_L = ('crl_arm__L', carla.Transform(location=carla.Location(x=0.17, y=0.0, z=1.48), rotation=carla.Rotation(yaw=-90, pitch=-70)))
            forearm_L = ('crl_forearm__L', carla.Transform(location=carla.Location(x=0.17, y=-0.07, z=1.29), rotation=carla.Rotation(yaw=90, pitch=-70, roll=180)))
            return [arm_R, forearm_R, hand_R, arm_L, forearm_L]

    def posePhoneRight(self):
            arm_R = ('crl_arm__R', carla.Transform(location=carla.Location(x=-0.16, z=1.49), rotation=carla.Rotation(yaw=-70, pitch=50)))
            forearm_R = ('crl_forearm__R', carla.Transform(location=carla.Location(x=-0.18, y=0.14, z=1.3), rotation=carla.Rotation(yaw=-140, roll=-30, pitch=-30)))
            hand_R = ('crl_hand__R', carla.Transform(location=carla.Location(x=-0.04, y=0.265, z=1.40), rotation=carla.Rotation(roll=-30, pitch=128, yaw=0)))
            arm_L = ('crl_arm__L', carla.Transform(location=carla.Location(x=0.17, y=0.0, z=1.48), rotation=carla.Rotation(yaw=-90, pitch=-70)))
            forearm_L = ('crl_forearm__L', carla.Transform(location=carla.Location(x=0.17, y=-0.07, z=1.29), rotation=carla.Rotation(yaw=90, pitch=-70, roll=180)))
            neck = ('crl_neck__c', carla.Transform(location=carla.Location(x=0, y=0.0, z=1.55), rotation=carla.Rotation(yaw=180, roll=50, pitch=0)))
            return [arm_R, forearm_R, hand_R, arm_L, forearm_L, neck]

    def posePhoneLeft(self):
            arm_L = ('crl_arm__L', carla.Transform(location=carla.Location(x=0.17, z=1.48), rotation=carla.Rotation(yaw=-110, pitch=-140, roll=0)))
            forearm_L = ('crl_forearm__L', carla.Transform(location=carla.Location(x=0.18, y=0.22, z=1.35), rotation=carla.Rotation(yaw=140, roll=180, pitch=20)))
            hand_L = ('crl_hand__L', carla.Transform(location=carla.Location(x=0.04, y=0.34, z=1.40), rotation=carla.Rotation(yaw=170, pitch=-50)))
            arm_R = ('crl_arm__R', carla.Transform(location=carla.Location(x=-0.17, y=0.0, z=1.48), rotation=carla.Rotation(yaw=90, roll=90, pitch=70)))
            forearm_R = ('crl_forearm__R', carla.Transform(location=carla.Location(x=-0.17, y=-0.07, z=1.29), rotation=carla.Rotation(yaw=-90, pitch=70)))
            neck = ('crl_neck__c', carla.Transform(location=carla.Location(x=0, y=0.0, z=1.55), rotation=carla.Rotation(yaw=180, roll=50, pitch=0)))
            return [arm_L, forearm_L, hand_L, arm_R, forearm_R, neck]


def main():
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
        '-f', '--filter',
        metavar='F',
        default='walker.pedestrian.0010',
        help='pedestrian blueprint (default: walker.pedestrian.0010)')
    argparser.add_argument(
        '-r', '--rolename',
        metavar='R',
        default='hero',
        help='pedestrian rolename (default: hero)')

    args = argparser.parse_args()

    # setup carla environment
    client = carla.Client(args.host, args.port)
    client.set_timeout(2.0)
    world = client.get_world()

    for carla_actor in world.get_actors():
        if carla_actor.attributes.get('role_name') == args.rolename:
            actor = carla_actor

    # get pose instance and setup
    pose_actor = PoseDefine()
    pose_dict = pose_actor.pose_dict
    pose_len = len(pose_dict)
    pose_index = 0

    # get actor control
    control = carla.WalkerBoneControl()
    control.bone_transforms = pose_actor.poseCallLeft()

    while True:
        key = raw_input('input n(ext) or p(revious) to make actor take pose or q(uit)')
        if key == 'n':
            pose_index += 1
            control.bone_transforms = pose_dict.values()[pose_index % pose_len]()
            actor.apply_control(control)
            time.sleep(0.05)
            actor.apply_control(control)
        elif key == 'p':
            pose_index -= 1
            control.bone_transforms = pose_dict.values()[pose_index % pose_len]()
            actor.apply_control(control)
            time.sleep(0.05)
            actor.apply_control(control)
        elif key == 'q':
            break
        else:
            print('wrong key')

if __name__ == '__main__':
    main()
