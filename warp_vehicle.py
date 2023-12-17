#!/usr/bin/env python
"""
Welcome to CARLA scenario controller.
"""
import carla
import xml.etree.ElementTree as ET
import argparse
import warnings

class WarpVehicle(Vehicle):
    def __init__(self, world, scenario_id, blueprint):
        super().__init__(world, scenario_id, blueprint)

