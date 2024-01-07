#!/usr/bin/env python

# Copyright (c) 2019 Aptiv
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


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
import weakref
import argparse
import collections
import math
import logging
import datetime

try:
    import pygame
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

VIEW_FOV = 100

# data log
logging.basicConfig(filename='/home/kuriatsu/Documents/carla_driving_result/time_' + datetime.datetime.now().strftime('%y%m%d_%H%M') + '.log', level=logging.INFO)
logging.info('round,time,collision')
# ==============================================================================
# -- Cliant ----------------------------------------------------
# ==============================================================================


class Cliant(object):
    """
    Basic implementation of a synchronous client.
    """

    def __init__(self, args):
        self.client = None
        self.world = None
        self.display = None
        self.image = None
        self.capture = True
        self.hud = None
        self.camera = None
        self.collision = None


    def set_synchronous_mode(self, synchronous_mode):
        """
        Sets synchronous mode.
        """

        settings = self.world.get_settings()
        settings.synchronous_mode = synchronous_mode
        self.world.apply_settings(settings)

    def game_loop(self, args):
        """
        Main program loop.
        """

        pygame.init()

        self.client = carla.Client(args.host, args.port)
        self.client.set_timeout(2.0)
        self.world = self.client.get_world()

        self.camera = Camera(args, self.world)
        self.collision_sensor = CollisionSensor(self.world, args)
        self.hud = HUD(args, self.world)
        self.world.on_tick(self.hud.callbackWorldTick)

        self.display = pygame.display.set_mode(args.res, pygame.HWSURFACE | pygame.DOUBLEBUF)
        pygame_clock = pygame.time.Clock()

        self.set_synchronous_mode(True)

        while True:
            self.world.tick()
            pygame_clock.tick(20)

            self.camera.render(self.display)
            self.hud.tick(self.collision_sensor)
            self.hud.render(self.display)
            pygame.display.flip()
            pygame.event.pump()

        pygame.quit()
# ==============================================================================
# -- Camera --------------------------------------------------------------------
# ==============================================================================

class Camera(object):
    def __init__(self, args, world):
        self.camera = None
        self.image = None
        self.capture = False
        self.spawned_here=False

        for carla_actor in world.get_actors():
            if carla_actor.type_id == "sensor.camera.rgb":
                if carla_actor.attributes.get('role_name') == args.cameraname:
                    self.camera = carla_actor
                    print('found camera')

        if self.camera is None:
            self.camera = self.set_sensor(args, world)
            self.spawned_here = True

        weak_self = weakref.ref(self)
        self.camera.listen(lambda image: weak_self().set_image(weak_self, image))

        calibration = np.identity(3)
        calibration[0, 2] = args.res[0] / 2.0
        calibration[1, 2] = args.res[1] / 2.0
        calibration[0, 0] = calibration[1, 1] = args.res[0] / (2.0 * np.tan(VIEW_FOV * np.pi / 360.0))
        self.camera.calibration = calibration

    @staticmethod
    def set_image(weak_self, img):
        """
        Sets image coming from camera sensor.
        The self.capture flag is a mean of synchronization - once the flag is
        set, next coming image will be stored.
        """
        self = weak_self()
        if self.capture:
            self.capture = False
            self.image = img

    def render(self, display):
        """
        Transforms image from camera sensor and blits it to main pygame display.
        """
        self.capture = True

        if self.image is not None:
            array = np.frombuffer(self.image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (self.image.height, self.image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            display.blit(surface, (0, 0))

    def set_sensor(self, args, world):
        """set sensor if specified sensor is not in world

        """
        ego_vehicle = None
        for carla_actor in world.get_actors():
            if carla_actor.attributes.get('role_name') == args.egoname:
                ego_vehicle = carla_actor

        if ego_vehicle is None:
            exit()

        camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(args.res[0]))
        camera_bp.set_attribute('image_size_y', str(args.res[1]))
        camera_bp.set_attribute('role_name', args.cameraname)
        camera_transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=0.0), carla.Rotation(0.0,0.0,0.0))
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=ego_vehicle, attachment_type=carla.AttachmentType.SpringArm)

        camera.set_transform(carla.Transform(carla.Location(x=args.transform[0], y=args.transform[1], z=args.transform[2]), carla.Rotation(args.transform[3],args.transform[4],args.transform[5])))
        return camera

# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================

class CollisionSensor(object):
    def __init__(self, world, args):
        self.sensor = None
        self.history = []

        for carla_actor in world.get_actors():
            if carla_actor.type_id == "sensor.other.collision":
                self.sensor = carla_actor
                print('found collision')


        if self.sensor is None:
            self.sensor = self.set_sensor(args, world)

        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return

        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)

    def set_sensor(self, args, world):
        """set sensor if specified sensor is not in world

        """
        ego_vehicle = None
        for carla_actor in world.get_actors():
            if carla_actor.attributes.get('role_name') == args.egoname:
                ego_vehicle = carla_actor

        # if ego_vehicle is None:
        #     return

        collision_bp = world.get_blueprint_library().find('sensor.other.collision')
        collision_bp.set_attribute('role_name', 'collision')
        collision_transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=0.0), carla.Rotation(0.0,0.0,0.0))
        collision = world.spawn_actor(collision_bp, collision_transform, attach_to=ego_vehicle, attachment_type=carla.AttachmentType.SpringArm)

        return collision
# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
    def __init__(self, args, world):
        # prepare font
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self.font_large = pygame.font.Font(mono, 150)
        self.font_middle = pygame.font.Font(mono, 100)
        self.font_small = pygame.font.Font(mono, 40)

        self.cameraname = args.cameraname
        self.simulation_time = 0
        self.start_time = 0.0
        self.stop_watch = 0.0
        self.frame = None
        self._show_info = True
        self.info_text = []
        self.ego_vehicle = None
        self.start_loc = args.start
        self.goal_loc = args.goal
        self.collision_num = 0
        self.last_collision_point = None
        self.last_collision_frame = 0
        self.round = -1
        self.is_on_start_point = False
        self.is_on_goal_point = False
        self.is_started = False

        for carla_actor in world.get_actors():
            if carla_actor.attributes.get('role_name') == args.egoname:
                self.ego_vehicle = carla_actor

    def callbackWorldTick(self, timestamp):
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, collision_sensor):
        if not self._show_info:
            return

        # get collisioin histry and align it
        colhist = collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]

        # get ego vehciel pose
        ego_loc = self.ego_vehicle.get_location()
        v = self.ego_vehicle.get_velocity()

        # start timer and stop it
        if self.start_loc and self.goal_loc:

            # on start point flag
            if (ego_loc.x - self.start_loc[0])**2 + (ego_loc.y - self.start_loc[1])**2 < 4.0:
                self.is_on_start_point = True

            # not on the start point
            else:
                # just after passing the start point, start stop_watch, count up round, reset collision count
                if self.is_on_start_point:
                    self.start_time = self.simulation_time
                    self.is_started = True
                    self.collision_num = 0
                    self.round += 1

                self.is_on_start_point = False

            # on goal point flag
            if (ego_loc.x - self.goal_loc[0])**2 + (ego_loc.y - self.goal_loc[1])**2 < 4.0:
                # just after reaching on goal point, log the result
                if not self.is_on_goal_point:
                    logging.info('{},{},{}'.format(self.round, self.stop_watch, self.collision_num))
                    self.is_started = False

                self.is_on_goal_point = True

            #  not on the goal flag
            else:
                self.is_on_goal_point = False

            # if started count up stop_watch
            if self.is_started:
                self.stop_watch = self.simulation_time - self.start_time

        # judge the collision is worth counting as one collision
        if colhist[self.frame - 1] != 0: # count the first collision
            if self.last_collision_point is None:
                self.collision_num = 1
            else: # sufficient gap of frame or distance from last collision
                collision_interval_dist = math.sqrt((self.last_collision_point.x - ego_loc.x)**2 + (self.last_collision_point.y - ego_loc.y)**2)
                collision_interval_frame = self.frame - self.last_collision_frame
                if collision_interval_dist > 2.0 or collision_interval_frame > 50:
                    self.collision_num += 1
            # update log
            self.last_collision_point = ego_loc
            self.last_collision_frame = self.frame

        # prepare info text
        self.info_text = [
            '% 3.0f' % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
            'km/h',
            'Collision:',
            collision,
            str(self.collision_num),
            'time:',
            '% .2f' % self.stop_watch,
            'Round:',
            str(self.round)
            ]

    def render(self, display):
        if self._show_info:
            if self.cameraname == 'wide_front':

                # info_surface = pygame.Surface((600, 200))
                # info_surface.fill((255, 255, 255))
                # info_surface.set_alpha(50)
                # display.blit(info_surface, (2000, 850))
                # v_offset = 4
                # bar_h_offset = 100
                # bar_width = 106

                # speed
                surface = self.font_middle.render(self.info_text[0], True, (255, 255, 255))
                display.blit(surface, (1340, 880))
                #  speed unit
                surface = self.font_small.render(self.info_text[1], True, (255, 255, 255))
                display.blit(surface, (1550, 935))
                # collision:
                surface = self.font_small.render(self.info_text[2], True, (255, 255, 255))
                display.blit(surface, (1850, 840))

                # collision graph
                if len(self.info_text[3]) > 1:
                    points = [(x*1.5 + 1860, 1000 + (1.0 - y) * 30) for x, y in enumerate(self.info_text[3])]
                    pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                self.info_text[3] = None

                # collision_num
                surface = self.font_large.render(self.info_text[4], True, (255, 255, 255))
                display.blit(surface, (2080, 870))
                # time:
                surface = self.font_small.render(self.info_text[5], True, (255, 255, 255))
                display.blit(surface, (2400, 840))
                # stop_watch
                surface = self.font_large.render(self.info_text[6], True, (255, 255, 255))
                display.blit(surface, (2350, 870))

                # round
                surface = self.font_small.render(self.info_text[7], True, (255, 255, 255))
                display.blit(surface, (2850, 840))

                # round num
                surface = self.font_large.render(self.info_text[8], True, (255, 255, 255))
                display.blit(surface, (3000, 870))

            # for item in self.info_text:
            #     if v_offset + 18 > self.dim[1]:
            #         break
            #     if isinstance(item, list):
            #         if len(item) > 1:
            #             points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
            #             pygame.draw.lines(display, (255, 136, 0), False, points, 2)
            #         item = None
            #         v_offset += 18
            #     elif isinstance(item, tuple):
            #         if isinstance(item[1], bool):
            #             rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
            #             pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
            #         else:
            #             rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
            #             pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
            #             f = (item[1] - item[2]) / (item[3] - item[2])
            #             if item[2] < 0.0:
            #                 rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
            #             else:
            #                 rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
            #             pygame.draw.rect(display, (255, 255, 255), rect)
            #         item = item[0]
            #     if item:  # At this point has to be a str.
            #         surface = self._font_mono.render(item, True, (255, 255, 255))
            #         display.blit(surface, (8, v_offset))
            #     v_offset += 18


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    Initializes the client-side bounding box demo.
    """
    parser_position = lambda x: list(map(float, x.split(',')))
    parser_res = lambda x: tuple(map(int, x.split('x')))

    argparser = argparse.ArgumentParser(
        description='Carla image viewer for demo')
    argparser.add_argument(
        '--host',
        default='127.0.0.1',
        help='IP of the host server (127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-c', '--cameraname',
        metavar='NAME',
        default='wide_front',
        help='camera role name (default: "wide_front")')

    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='3840x1080',
        type=parser_res,
        help='window resolution (default: 3840x1080)')
    argparser.add_argument(
        '-s' ,'--start',
        metavar=('x', 'y', 'z', 'yaw'),
        default=(229.2, 133.4, -0.0, -90.0),
        type=float,
        nargs=4,
        help='start position where the timer starts')
    argparser.add_argument(
        '-t','--transform',
        metavar='x,y,z,roll,pitch,yaw',
        type=float,
        nargs=6,
        default=(-0.18, -0.2, 0.9, 0.0, 0.0, 0.0),
        help='relative transform from vehicle '
    )
    argparser.add_argument(
        '-g' ,'--goal',
        metavar=('x', 'y', 'z', 'yaw'),
        default=(229.2, 133.4, -0.0, -90.0),
        type=float,
        nargs=4,
        help='goal position where the timer stops')
    argparser.add_argument(
        '-e', '--egoname',
        metavar='NAME',
        default='ego_vehicle',
        help='vehicle role name (default: "ego_vehicle")')

    args, unknown = argparser.parse_known_args()
    print(args.transform)

    try:
        client = Cliant(args)
        client.game_loop(args)
    finally:
        client.set_synchronous_mode(False)
        print('destroy camera')
        client.camera.camera.destroy()

        # if self.camera.spawned_here:
        print('EXIT')


if __name__ == '__main__':
    main()
