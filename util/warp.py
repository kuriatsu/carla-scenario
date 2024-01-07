#!/usr/bin/python
# -*- coding: utf-8 -*-

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

client = carla.Client('127.0.0.1', 2000)
client.set_timeout(2.0)
world = client.get_world()
hero = None
ego = None

for actor in world.get_actors():

    if actor.attributes.get('role_name') == 'hero':
        hero = actor
    if actor.attributes.get('role_name') == 'ego_vehicle':
        ego = actor


hero.set_location(ego.get_location()+carla.Location(x=2.0, z=3.0))
