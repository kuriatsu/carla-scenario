#!/usr/bin/env python
"""
Welcome to CARLA scenario controller.
"""
import carla
import xml.etree.ElementTree as ET
import argparse
import warnings
from actors.actor import Actor

class TrafficLight(Actor):
    def __init__(self, world):
        super().__init__(world, None, None)


    def move(self, xml):
        if (xml.find('state').text == 'red'):
            actor.set_state(carla.TrafficLightState.Red)
            actor.set_red_time(float(xml.find('time').text))

        elif (xml.find('state').text == 'green'):
            actor.set_green_time(float(xml.find('time').text))
            actor.set_state(carla.TrafficLightState.Green)

        return

def getTrafficLights(world):

    tls = {} 
    for actor in world.get_actors():
        if actor.type_id == 'traffic.traffic_light':
            tl = TrafficLight(world)
            tl.world_id = actor.id
            tl.carla_actor = actor
            tls[actor.id] = tl 

    return tls
