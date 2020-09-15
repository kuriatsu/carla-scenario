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

import carla
import argparse
import csv

color = [
carla.Color(0,134,179),
carla.Color(237,241,176),
carla.Color(249,194,112),
carla.Color(166,186,178),
carla.Color(111,186,44),
carla.Color(52,114,161),
carla.Color(82,59,67),
carla.Color(232,55,74),
carla.Color(15,216,45),
carla.Color(0,175,204),
]
def readFile(filename):

	file = open(filename, 'r')
	reader = csv.reader(file)
	header = next(reader)

	in_list = [row for row in reader]
	return in_list

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

    args = argparser.parse_args()
    print('hetl')
    print(sys.version)
    client = carla.Client(args.host, args.port)
    client.set_timeout(2.0)
    world = client.get_world()

    actor_list = readFile(args.actor_file)
    scenario_list = readFile(args.scenario_file)

    debug = world.debug

    for scenario in scenario_list:
        location = carla.Location(float(scenario[1]), float(scenario[2]), float(scenario[3]))
        debug.draw_point(location=location, life_time=30, size=0.5, color=color[int(scenario[0])%10])
        debug.draw_string(location=location+carla.Location(z=1.0), text=scenario[0]+'trigger', color=carla.Color(255,255,255), life_time=30)

    for actor in actor_list:
        location = carla.Location(float(actor[2]), float(actor[3]), float(actor[4]))

        if actor[1] == 'ai_walker' or actor[1] == 'ai_vehicle':
            debug.draw_point(location=location, life_time=30, size=0.1, color=color[int(actor[0])%10])
            debug.draw_string(location=location+carla.Location(z=1.0), text=actor[0]+'free', color=carla.Color(255,255,255), life_time=30)
        else:
            while len(actor) > 9:
                if actor[1] == 'walker':
                    thickness = 0.1
                elif actor[1] == 'vehicle':
                    thickness = 1.0
                goal = carla.Location(float(actor[9]), float(actor[10]), float(actor[11]))
                debug.draw_line(begin=location, end=goal, color=color[int(actor[0])%10], thickness=thickness, life_time=30)
                debug.draw_string(location=location+carla.Location(z=1.0), text=actor[0]+actor[8], color=carla.Color(255,255,255), life_time=30)
                debug.draw_string(location=goal+carla.Location(z=1.0), text=actor[0]+actor[8], color=carla.Color(255,255,255), life_time=30)
                del actor[8:12]
                location = goal

if __name__ == '__main__':

    main()
