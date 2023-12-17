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

        return carla.command.SpawnActor(blueprint, transform)

    def move(self, xml):
        self.is_controlled = True
        self.waypoints = xml.findall("waypoint")

    def calcControl(control_actor):
        # some information for movng
        transform = control_actor.get('actor').get_transform()
        if transform is None:
            print('cannot get transform: ' + control_actor.get('id'))
            return 0, 0, 0, 0

        point = control_actor.get('waypoints')[0].text
        speed = float(control_actor.get('waypoints')[0].attrib.get('speed'))
        # calc culent motion vector and distance to the target
        vector = carla.Vector3D(
            float(point[0]) - transform.location.x,
            float(point[1]) - transform.location.y,
            float(point[2]) - transform.location.z
            )
        dist = math.sqrt(vector.x ** 2 + vector.y ** 2)
        # normalize vector to calcurate velocity
        vector.x = vector.x / dist
        vector.y = vector.y / dist
        # debug.draw_arrow(begin=transform.location ,end=transform.location + carla.Location(vector.x, vector.y, 0.0), life_time=0.5)
        return vector, speed, dist, transform.rotation.yaw

    def step(self, xml):
        super().step()
        # calc vel and rotation
        vector, speed, dist, yaw = calcControl(control_actor)
        if control_actor['type'] == 'vehicle' and dist < 5.0:
            del self.waypoints[0]

        # calc velocity
        velocity = carla.Vector3D()
        velocity.x = vector.x * speed
        velocity.y = vector.y  * speed
        velocity.z = 0.0

        # calc angular velocity
        # define angle (radian)
        objective_angle = math.atan(vector.y / vector.x)
        current_angle = math.radians(yaw)

        # difference between objective and current angle
        alpha = objective_angle - current_angle

        # take care the change from 180 -> -180
        if objective_angle < 0.0 and current_angle >= 0.0:
            alpha_2 = math.pi - current_angle + (math.pi + objective_angle)
            alpha = (alpha if abs(alpha) < abs(alpha_2) else alpha_2)
        if current_angle < 0.0 and objective_angle >= 0.0:
            alpha_2 = -(math.pi - objective_angle + (math.pi + current_angle))
            alpha = alpha if abs(alpha) < abs(alpha_2) else alpha_2

        # invert angle (I do not know why)
        alpha *= 1

        # calcurat omega (100 is param)
        omega = 2 * 100 * speed * math.sin(alpha) / dist

        return [
            carla.command.ApplyVelocity(world_id, velocity),
            carla.command.ApplyAngularVelocity(world_id, carla.Vector3D(0.0, 0.0, omega))
            ]
