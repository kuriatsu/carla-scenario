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
import xml.etree.ElementTree as ET
import argparse
import csv
import warnings

color = [
carla.Color(255,255,255),
carla.Color(255,255,0),
carla.Color(0,255,255),
carla.Color(0,255,0),
carla.Color(128,128,128),
carla.Color(128,128,0),
carla.Color(255,0,255),
carla.Color(0,128,128),
carla.Color(255,0,0),
carla.Color(0,128,0),
carla.Color(0,0,128),
carla.Color(128,0,128),
carla.Color(128,0,0),
carla.Color(0,0,255),
carla.Color(0,0,0),

]
line_thickness = {
'walker' : 0.2,
'vehicle' : 1.0
}
trafficlight_color = {
'red' : carla.Color(255, 0, 0),
'green' : carla.Color(0, 255, 0),
'yellow' : carla.Color(255, 255, 0)

}
def readFile(filename):

    tree = ET.parse(filename)
    root = tree.getroot()
    edit_tag_list = ['transform', 'location', 'waypoint']

    for tag in edit_tag_list:
        for itr in root.iter(tag):
            if itr.text is not None:
                itr.text = [float(val) for val in itr.text.split(',')]

    return root

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
    '-s', '--scenario_file',
    metavar='S',
    default='/home/mad-carla/share/catkin_ws/src/carla_helper/scenario.xml',
    help='scenario file (default: scenario.xml)')
    argparser.add_argument(
    '-l', '--lifetime',
    metavar='XX[sec]',
    default=30,
    type=int,
    help='lifetime of marker default 30[s]')

    args = argparser.parse_args()
    client = carla.Client(args.host, args.port)
    client.set_timeout(2.0)
    world = client.get_world()
    debug = world.debug

    scenario = readFile(args.scenario_file)
    actor_position = {}
    trigger_index = 0

    for trigger in scenario:
        buf = trigger[0].text
        location = carla.Location(buf[0], buf[1], buf[2])
        print(trigger.attrib.get('thres'))
        debug.draw_point(
            location=location,
            life_time=args.lifetime,
            size=0.02 * float(trigger.attrib.get('thres')),
            color=color[trigger_index % len(color)]
            )
        debug.draw_string(location=location+carla.Location(z=1.0),
            text='trigger'+trigger.attrib.get('id'),
            color=carla.Color(255,255,255), life_time=args.lifetime
            )

        for i, action in enumerate(trigger[1:]):
            print("id: ", action.attrib.get('id'), " type: ", action.tag)
            if action.tag == 'spawn':
                buf = action.find('transform').text
                location = carla.Location(buf[0], buf[1], buf[2])
                # debug.draw_point(
                #     location=location,
                #     life_time=args.lifetime,
                #     size=0.1,
                #     color=color[trigger_index % len(color)]
                #     )
                if (action.find('type').text == 'walker' or action.find('type').text == 'vehicle'):
                    debug.draw_string(
                        location=location+carla.Location(z=1.0),
                        text=action.attrib.get('id'),
                        color=color[trigger_index % len(color)],
                        life_time=args.lifetime
                        )
                else:
                    debug.draw_string(
                        location=location+carla.Location(z=1.0),
                        text=action.attrib.get('id'),
                        color=color[trigger_index % len(color)],
                        life_time=args.lifetime
                        )
                actor_position[action.attrib.get('id')] = location

            if action.tag == 'move':
                waypoints = action.findall('waypoint')
                start = actor_position.get(action.attrib.get('id'))
                if start is None:
                    warnings.warn('actor {} is not spawned but waypoint is set {}'.format(action.attrib.get('id'), trigger.attrib.get('id')))
                else:
                    waypoints.insert(0, start)
                    if len(waypoints) > 1:
                        for i in range(1, len(waypoints)):
                            buf = waypoints[i].text
                            if buf is not None:
                                waypoints[i] = carla.Location(buf[0], buf[1], buf[2])
                                debug.draw_line(
                                    begin=waypoints[i-1],
                                    end=waypoints[i],
                                    color=color[trigger_index % len(color)],
                                    thickness=0.5, life_time=args.lifetime
                                    )
                                actor_position[action.attrib.get('id')] = waypoints[i]
                            else:
                                debug.draw_string(
                                    location=actor_position[action.attrib.get('id')]+carla.Location(x=2.0, z=1.0),
                                    text='free',
                                    color=color[trigger_index % len(color)],
                                    life_time=args.lifetime
                                    )
                                # debug.draw_point(
                                #     location=start+carla.Location(x=2.0, z=1.0),
                                #     life_time=args.lifetime,
                                #     size=0.1,
                                #     color=color[trigger_index % len(color)]
                                #     )

            if action.tag == 'pose':
                # debug.draw_point(
                #     location=actor_position[action.attrib.get('id')]+carla.Location(z=3.0),
                #     life_time=args.lifetime,
                #     size=0.1,
                #     color=color[trigger_index % len(color)]
                #     )
                debug.draw_string(
                    location=actor_position[action.attrib.get('id')]+carla.Location(x=4.0, z=1.0),
                    text=action.find('form').text,
                    color=color[trigger_index % len(color)],
                    life_time=args.lifetime
                    )

            if action.tag == "kill":
                # debug.draw_point(
                #     location=actor_position[action.attrib.get('id')]+carla.Location(x=4.0),
                #     life_time=args.lifetime,
                #     size=0.1,
                #     color=color[trigger_index % len(color)]
                #     )
                debug.draw_string(
                    location=actor_position[action.attrib.get('id')]+carla.Location(x=6.0, z=1.0),
                    text='kill',
                    color=color[trigger_index % len(color)],
                    life_time=args.lifetime
                    )

            if action.tag == "trafficlight":
                buf = action.find('location').text
                location = carla.Location(buf[0], buf[1], buf[2])
                debug.draw_point(
                    location=location,
                    life_time=args.lifetime,
                    size=0.1,
                    color=color[trigger_index % len(color)]
                    )
                debug.draw_string(
                    location=location+carla.Location(x=2.0, z=1.0),
                    text=action.find("time").text,
                    color=trafficlight_color[action.find("state").text],
                    life_time=args.lifetime
                    )
                # else:
                    # if start is not None:
                        # debug.draw_string(location=start+carla.Location(z=1.0), text='start_ai', color=carla.Color(0,255,0), life_time=30)
                    # else:
                        # warnings.warn('actor {} is not spawned but ai is start {}'.format(action.attrib.get('id'), trigger.attrib.get('id')))
        trigger_index += 1

if __name__ == '__main__':
    main()
