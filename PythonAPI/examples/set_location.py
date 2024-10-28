import sys
import random
import time
import ipdb

try:
    sys.path.append('D:/CARLA_0.9.14/WindowsNoEditor/PythonAPI/carla/dist/carla-0.9.14-cp37-cp37m-win_amd64.whl')
except IndexError:
    pass
import carla

sys.path.append('/home/u/Bench2DriveZoo/carla/PythonAPI/carla/')

import os
import random
import sys

import carla
sys.path.append('/home/u/Bench2DriveZoo/carla/PythonAPI/carla/')
from agents.navigation.basic_agent import BasicAgent

# To import a behavior agent
from agents.navigation.behavior_agent import BehaviorAgent


def main():
    try:
        # ipdb.set_trace()
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        # Retrieve the world that is currently running
        world = client.get_world()

        origin_settings = world.get_settings()

        # set sync mode
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        blueprint_library = world.get_blueprint_library()

        # # read all valid spawn points
        # all_default_spawn = world.get_map().get_spawn_points()
        # # randomly choose one as the start point
        # spawn_point = random.choice(all_default_spawn) if all_default_spawn else carla.Transform()

        # set the destination spot
        spawn_points = world.get_map().get_spawn_points()

        # create the blueprint library
        # ego_vehicle_bp = blueprint_library.find('vehicle.lincoln.mkz2017')
        ego_vehicle_bp = blueprint_library.filter('model3')[0]        

        ego_vehicle_bp.set_attribute('color', '0, 0, 0')
        # spawn the vehicle
        vehicle = world.spawn_actor(ego_vehicle_bp, spawn_points[0])

        spectator = world.get_spectator()
        new_spectator_transform = carla.Transform(
    	spawn_points[0].location + carla.Location(x=2, y=2, z=10),
    	spawn_points[0].rotation)
        spectator.set_transform(new_spectator_transform)

        # we need to tick the world once to let the client update the spawn position
        # world.tick()

        # create the behavior agent
        agent = BasicAgent(vehicle, 30)
        agent = BehaviorAgent(vehicle, behavior='normal')

        print("spawn_point = ",spawn_points[0])
        
        ipdb.set_trace()
        for i in range(5):
        # set five points to let the vehicle "follow"
            print("destination = ",spawn_points[i])
            vehicle.set_location(spawn_points[i].location)
            draw_point_location = carla.Location(x = spawn_points[i].location.x , y = spawn_points[i].location.y ,
                                        z = spawn_points[i].location.z)
            world.debug.draw_point(draw_point_location, size=0.1, color=carla.Color(r=255, g=0, b=0), life_time=-1.0)
            time.sleep(2)

    finally:
        world.apply_settings(origin_settings)
        vehicle.destroy()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
