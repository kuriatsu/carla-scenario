#!/usr/bin/env python
"""
Welcome to CARLA scenario controller.
"""
import carla
import xml.etree.ElementTree as ET
import argparse
import warnings

class TrafficLight(Actor):
    def __init__(self, world):
        super().__init__(world, None, None)

    def getLights(self):

        tls = []

        for carla_actor in self.world.get_actors():
            if carla_actor.type_id == 'traffic.traffic_light':
                tl = {}
                tl['id'] = carla_actor.id
                tl['location'] = carla_actor.get_location()
                tls.append(tl)

        return tls

    def move(self, xml):
        if (controll_light.find('state').text == 'red'):
            actor.set_state(carla.TrafficLightState.Red)
            actor.set_red_time(float(controll_light.find('time').text))

        elif (controll_light.find('state').text == 'green'):
            actor.set_green_time(float(controll_light.find('time').text))
            actor.set_state(carla.TrafficLightState.Green)
