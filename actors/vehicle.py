#!/usr/bin/env python
"""
Welcome to CARLA scenario controller.
"""
import carla
import xml.etree.ElementTree as ET
import argparse
import warnings

class Vehicle(Actor):
    def __init__(self, world, scenario_id, blueprint):
        super().__init__(world, scenario_id, blueprint)

    def getBlueprint(self, xml):
    # def getBlueprint(self, blueprint_list, name):

        if xml.find("blueprint") == "random":
            blueprint = random.choice(self.blueprint.filter("vehicle.*"))
        else:
            try:
                blueprint = self.blueprint.find(name)
            except ValueError:
                warnings.warn(f"spcecified blueprint is not exist : {xml.find('blueprint').text}")
                blueprint = random.choice(self.blueprint.filter("vehicle.*"))

        blueprint.set_attribute('role_name', spawn.attrib.get(self.scenario_id))
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)

        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)

        return blueprint

    def spawn(self, xml):

        blueprint = self.getBlueprint(self, xml)

        transform = xml.find('transform').text
        transform = carla.Transform(
                carla.Location(transform[0], transform[1], transform[2]), 
                carla.Rotation(transform[3], transform[4], transform[5])
                )

        self.commands = [carla.command.SpawnActor(blueprint, transform)]
        return

    def move(self, xml):
        self.waypoints = xml.findall("waypoint")

        vector, speed, dist, omaga = self.calcControl()
        if dist < 5.0:
            del self.waypoints[0]

        self.commands = [
            carla.command.ApplyVelocity(world_id, velocity),
            carla.command.ApplyAngularVelocity(world_id, carla.Vector3D(0.0, 0.0, omega))
            ]

        return

    def calcControl():
        transform = self.actor.get_transform()
        if transform is None:
            print('cannot get transform: ' + self.scenario_id)
            return 0, 0, 0, 0

        waypoint = self.waypoints[0].text
        speed = float(self.waypoints[0].attrib.get('speed'))

        ## calc current motion vector and distance from current position to waypoint
        vector = carla.Vector3D(
            float(waypoint[0]) - transform.location.x,
            float(waypoint[1]) - transform.location.y,
            float(waypoint[2]) - transform.location.z
            )
        dist = math.sqrt(vector.x ** 2 + vector.y ** 2)
        vector.x = vector.x / dist
        vector.y = vector.y / dist
        # debug.draw_arrow(begin=transform.location ,end=transform.location + carla.Location(vector.x, vector.y, 0.0), life_time=0.5)

        ## calcurate velocity
        velocity = carla.Vector3D()
        velocity.x = vector.x * speed
        velocity.y = vector.y * speed
        velocity.z = 0.0

        ## calcurate angular velocity
        objective_angle = math.atan(vector.y / vector.x)
        if vector.x < 0.0 and vector.y < 0.0:
            objective_angle -= math.pi
        elif vector.x < 0.0 and vector.y > 0.0:
            objective_angle += math.pi
        current_angle = math.radians(yaw)

        ## difference between objective and current angle
        alpha = objective_angle - current_angle

        ## calcurate omega (100 is param)
        omega = 2 * 100 * speed * math.sin(alpha) / dist

        return vector, speed, dist, omega 

    def getResponse(self, response):
        super().getResponse(response)
        if self.waypoints:
            vector, speed, dist, omaga = self.calcControl()
            self.commands = [
                carla.command.ApplyVelocity(world_id, velocity),
                carla.command.ApplyAngularVelocity(world_id, carla.Vector3D(0.0, 0.0, omega))
                ]
            if dist < 5.0:
                self.waypoints.pop(0)

        else:
            self.commands = []

        return
