#!/usr/bin/env python
"""
Welcome to CARLA scenario controller.
"""
import carla
import xml.etree.ElementTree as ET
import argparse
import warnings
from actors.vehicle import Vehicle

class AiVehicle(Vehicle):
    def __init__(self, world, scenario_id, blueprint):
        super().__init__(world, scenario_id, blueprint)
        self.waypoints

    def move(self, xml):
        try:
            self.actor.set_autopilot(True)
        except:
            print('cannot start AI vehicle. world_id: ', world_id)
            return
