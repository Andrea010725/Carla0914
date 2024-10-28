import math
import os
import queue
import random
import sys
import ipdb
import carla



def main():
    try:
        # setup client并且加载我们所需要的地图
        client = carla.Client('localhost', 2000)
        client.set_timeout(120.0)
        client.load_world('Town01')
        num_walkers = 50

        percentage_pedestrians_running = 0.25
        # 设置横穿马路的行人比例

        # 获取我们client所对应的world
        world = client.get_world()
        # 获得这个world中的观察者
        spectator = world.get_spectator()

        # 获得当前客户端的交通管理器
        traffic_manager = client.get_trafficmanager()

        # 获得观察者的方位信息
        transform = spectator.get_transform()

        # 根据观察者默认方位设置新的方位
        location = transform.location + carla.Location(x=-30, z=20)
        rotation = carla.Rotation(pitch=-20, yaw=-20, roll=0)
        new_transform = carla.Transform(location, rotation)

        # 将观察者设置到新方位上
        spectator.set_transform(new_transform)

        # 获得整个的blueprint库并从中筛选出行人
        ped_blueprints = world.get_blueprint_library().filter('*pedestrian*')

        # 通过world获得map并获得所有可以生成车辆的地点
        vehicle_spawn_points = world.get_map().get_spawn_points()

        # 通过world获得所有可以生成行人的地点并存储
        ped_spawn_points = []
        for i in range(num_walkers):
            spawn_point = carla.Transform()
            loc = world.get_random_location_from_navigation()
            if loc is not None:
                spawn_point.location = loc
                ped_spawn_points.append(spawn_point)


        # 创建用来存储行人，行人速度设置和行人控制器的list
        # walker_batch = []
        walker_speed = []
        # walker_ai_batch = []

        # 在地图上随机生成num_walkers个行人
        for j in range(num_walkers):
            walker_bp = random.choice(ped_blueprints)
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')

            # 确保我们有一个有效的生成点
            if len(ped_spawn_points) > 0:
                ped_spawn_point = ped_spawn_points.pop(0)  # 从列表中取出一个生成点并更新列表

                # 尝试生成行人
                walker = world.try_spawn_actor(walker_bp, ped_spawn_point)
                if walker is not None:
                    # 确保行人AI控制器被创建并附加到行人实体上
                    walker_ai_blueprint = world.get_blueprint_library().find('controller.ai.walker')
                    walker_ai = world.spawn_actor(walker_ai_blueprint, carla.Transform(), walker)
                    walker_ai.start()

                    # 设置行人的目标位置
                    target_location = carla.Location(x=ped_spawn_point.location.x + random.uniform(2, 10),  # 随机目标距离
                                                     y=ped_spawn_point.location.y,
                                                     z=ped_spawn_point.location.z)
                    walker_ai.go_to_location(target_location)

                    # 设置行人的最大速度
                    walker_ai.set_max_speed(0.2)  # 假设所有行人速度相同

                    # 打印行人的初始位置
                    walker_location = walker.get_location()
                    print(f'ped_spawn_point.location : {ped_spawn_point.location}')
                    print(f"Walker location: {walker_location}")

                    # 检查行人是否到达目标位置，并在到达后停止
                    if 10.0<= (walker_location.x -ped_spawn_point.location.x)**2 + (walker_location.y -ped_spawn_point.location.y)**2  :  # 如果足够接近目标位置
                        walker_ai.set_max_speed(0.0)
                        walker_ai.stop()
                else:
                    print("Failed to spawn walker at location:", ped_spawn_point.location)
            else:
                print("No valid spawn points for walkers.")
            # # 假设 i 是一个有效的索引，表示您想要查询的行人AI控制器的位置
            # walker_actor = walker_ai_batch[i]  # 获取列表中的行人AI控制器实例

        # # 为整批行人各自生成对应的控制器，并把控制器添加到代表批量控制器的列表中
        # for walker in world.get_actors().filter('*pedestrian*'):
        #     walker_ai_batch.append(world.spawn_actor(walker_ai_blueprint, carla.Transform(), walker))
        #
        # # 批量启动行人控制器，并设置控制器参数
        # for i in range(len(walker_ai_batch)):
        #     # 启动控制器
        #     walker_ai_batch[i].start()
        #     # 通过控制器设置行人的目标点
        #     navigation = carla.Location(x=ped_spawn_points[i].location.x +2, y=ped_spawn_points[i].location.y +2,z =ped_spawn_points[i].location.z )
        #     walker_ai_batch[i].go_to_location(navigation)
        #     walker_ai_batch[i].set_max_speed(0.2)
        #     # 假设 i 是一个有效的索引，表示您想要查询的行人AI控制器的位置
        #     walker_actor = walker_ai_batch[i]  # 获取列表中的行人AI控制器实例
        #     walker_location = walker_actor.get_location()  # 获取行人的位置
        #     print(f"Walker location: {walker_location}")
        #     ipdb.set_trace()
        #     if (walker_location.x - ped_spawn_points[i].location.x)**2 +  (walker_location.y - ped_spawn_points[i].location.x) ** 2 >= 0.5 :
        #         walker_ai_batch[i].stop()
            # 通过控制器设置行人的行走速度



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
        # 通过交通管理器设置所有车辆相对于限速的差值，这里为负即为所有车辆都会i超速行驶

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