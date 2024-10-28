import math
import os
import queue
import random
import sys
import glob
import ipdb
import time


import carla

try:
    sys.path.append(glob.glob('D:/CARLA_0.9.14/WindowsNoEditor/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


def main():
    try:
        # setup client并且加载我们所需要的地图
        client = carla.Client('localhost', 2000)
        client.set_timeout(20.0)
        client.load_world('Town01')
        num_walkers = 3
        # 获取我们client所对应的world
        world = client.get_world()
        # 获得这个world中的观察者

        # 获得当前客户端的交通管理器
        traffic_manager = client.get_trafficmanager()
        # 获得观察者的方位信息


        # 获得整个的blueprint库并从中筛选出行人
        ped_blueprints = world.get_blueprint_library().filter('*pedestrian*')
        # ipdb.set_trace()
        # 获取所有waypoints

        spawn_point = carla.Transform(carla.Location(x=335.489990, y=298.809998, z=0.300000),
                    carla.Rotation(pitch=0.000000, yaw=90.000046, roll=0.000000))
        # 定义车辆的蓝图列表，这里以常用的车辆为例
        vehicle_blueprints = world.get_blueprint_library().filter('vehicle.*')

        # 从蓝图中随机选择一个车辆蓝图
        vehicle_blueprint = random.choice(vehicle_blueprints)
        vehicle = world.spawn_actor(vehicle_blueprint, spawn_point)
        camera_blueprint = world.get_blueprint_library().find('sensor.camera.rgb')

        # 设置相机的属性，例如图像宽度、高度和fov角度
        camera_blueprint.set_attribute('image_size_x', '800')
        camera_blueprint.set_attribute('image_size_y', '600')
        camera_blueprint.set_attribute('fov', '90')

        # 创建相机的初始位置，将其放置在汽车前方
        camera_transform = carla.Transform(carla.Location(x=2.0, z=2.5))  # 位置在汽车前方2米，上方2.5米

        # 在汽车上生成相机
        camera = world.spawn_actor(camera_blueprint, camera_transform, attach_to=vehicle)
        min_distance = 5

        max_distance = 10

        # 通过world获得所有可以生成行人的地点并存储
        ped_spawn_points = []
        for i in range(num_walkers):
            random_distance = random.uniform(min_distance, max_distance)
            # 计算前方生成点的相对位置
            relative_location = carla.Location(x=random_distance, y=random_distance, z=0)
            # 从原始的spawn_point位置添加相对位置得到新的行人生成点
            new_spawn_point = carla.Transform(spawn_point.location + relative_location)
            print('---------new_spawn_point----------',new_spawn_point)
            # 将新的行人生成点添加到列表中
            ped_spawn_points.append(new_spawn_point)
            print("-----",new_spawn_point)


        # 创建用来存储行人，行人速度设置和行人控制器的list
        walker_batch = []
        walker_speed = []
        walker_ai_batch = []
        walker_list = []

        # 在地图上随机生成num_walkers个行人，每个行人为行人蓝图库中的随机行人，并设定行人的移动速度
        for j in range(num_walkers):
            walker_bp = random.choice(ped_blueprints)

            # 取消行人无敌状态
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')

            # 设置行人的移动速度
            if walker_bp.has_attribute('speed'):
                walker_speed.append(0.2)
            walker = world.try_spawn_actor(walker_bp,ped_spawn_points[j])
            print('ped_spawn_points :',ped_spawn_points[j])
            walker_list.append(walker)
            walker_batch.append(walker)
        # 从蓝图库中寻找控制行人行动逻辑的控制器
            walker_ai_blueprint = world.get_blueprint_library().find('controller.ai.walker')
            walker_ai_batch.append(world.spawn_actor(walker_ai_blueprint, carla.Transform(), walker))

        # 批量启动行人控制器，并设置控制器参数

        # for i in range(len(walker_ai_batch)):
        #     if walker is not None:
        #         walker_ai = walker_ai_batch[i]
        #         walker_ai.start()
        #         random_distance = random.uniform(4, 6)
        #         walker_ai.set_max_speed(0.5)  # 假设所有行人速度相同
        #
        #         # 计算每个行人的独特目标位置
        #         des_location = carla.Location(x=spawn_point.location.x + random_distance,
        #                                       y=spawn_point.location.y + random_distance,
        #                                       z=spawn_point.location.z)
        #         print(f"des_location : {des_location}")
        #         print(f"spawn_point.location : {spawn_point.location}")
        #
        #         for walker in walker_list:
        #             while True:
        #                 # 获取行人当前位置
        #                 walker_location = walker.get_location()
        #                 print('walker_location :',walker_location)
        #                 distance_to_des = math.sqrt((walker_location.x - des_location.x) ** 2 +
        #                                             (walker_location.y - des_location.y) ** 2)
        #                 print('distance_to_des :',distance_to_des)
        #                 if distance_to_des <= 1.0:  # 如果足够接近目标位置
        #                     walker_ai.set_max_speed(0)
        #                     walker_ai.stop()  # 停止行人
        #                     print("ssssttttooooppppp")
        #                     break  # 退出循环
        #                 if distance_to_des >= 6.0:  # 如果足够接近目标位置
        #                     walker_ai.set_max_speed(0)
        #                     walker_ai.stop()  # 停止行人
        #                     print("ssssttttooooppppp")
        #                     break  # 退出循环
        #
        #                 # 同步到下一个仿真时间步长
        #                 world.wait_for_tick()

        # 假设 walker_ai_batch 和 walker_list 已经被正确初始化和填充
        # 假设 spawn_point 是已知的起始点

        for i in range(len(walker_ai_batch)):
            if walker_ai_batch[i] is not None and walker_list[i] is not None:
                walker_ai = walker_ai_batch[i]
                walker_ai.start()
                walker_ai.set_max_speed(0.5)
                # 为每个行人计算独特的目标位置
                random_distance = random.uniform(4, 6)
                des_location = carla.Location(x=spawn_point.location.x + random_distance,
                                              y=spawn_point.location.y + random_distance,
                                              z=spawn_point.location.z)
                walker_ai.go_to_location(des_location)

                walker = walker_list[i]
                        # 检查行人是否到达目标位置
                while True:
                    walker_location = walker.get_location()
                    distance_to_des = math.sqrt((walker_location.x - des_location.x) ** 2 +
                                                (walker_location.y - des_location.y) ** 2)
                    print('walker_location:', walker_location)
                    print('distance_to_des:', distance_to_des)

                    # 如果行人到达目标位置或超出范围，停止行人并退出循环
                    if distance_to_des <= 1.0 or distance_to_des >= 6.0:
                        walker_ai.set_max_speed(0)
                        # walker_ai.stop()  # 停止行人
                        print("行人停止")
                        # walker.destroy()  # 销毁行人角色
                        break  # 退出当前行人的循环

                # 同步到下一个仿真时间步长
                    world.wait_for_tick()


        # 获得当前模拟世界的设定
        setting = world.get_settings()
        # 设定为异步模式
        setting.synchronous_mode = True
        # 将时间步长设定为固定的0.03秒
        setting.fixed_delta_seconds = 0.05
        # 应用设定
        world.apply_settings(setting)

        # 将交通管理器设置为同步模式
        traffic_manager.synchronous_mode = True

        # 令摄像头读取数据并存储(异步模式)
        #camera.listen(lambda image: image.save_to_disk(output_path % image.frame))

        while True:

            # 如果为同步模式设定
            if traffic_manager.synchronous_mode:
                # 更新模拟世界
                world.tick()


            # 如果为异步模式设定
            else:
                # 更新模拟世界
                world.wait_for_tick()

    finally:
        # 停止并销毁所有controller
        for controller in world.get_actors().filter('*controller*'):
            controller.stop()
        # 销毁所有行人
        for walker in world.get_actors().filter('*walker*'):
            walker.destroy()

        # 获得当前模拟世界设定
        settings = world.get_settings()
        # 设定为异步模式
        settings.synchronous_mode = False
        # 设定为可变时间步长
        settings.fixed_delta_seconds = None
        # 应用设定
        world.apply_settings(settings)

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')