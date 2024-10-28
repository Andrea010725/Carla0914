import sys
import random
import time
import ipdb
import math

try:
    sys.path.append('D:/CARLA_0.9.14/WindowsNoEditor/PythonAPI/carla/dist/carla-0.9.14-cp37-cp37m-win_amd64.whl')
except IndexError:
    pass
import carla
import sys
import ipdb

sys.path.append('D:/CARLA_0.9.14/WindowsNoEditor/PythonAPI/carla')
from agents.navigation.behavior_agent import BehaviorAgent  # pylint: disable=import-error
from agents.navigation.basic_agent import BasicAgent
from agents.navigation.local_planner import LocalPlanner
from agents.navigation.global_route_planner import GlobalRoutePlanner

class Traj(object):
    def __init__(self, start_point, end_point,parking_lot):
        self.start_point_x = start_point[0]
        self.start_point_y = start_point[1]
        self.start_point_theta = start_point[2]
        self.end_point_x = end_point[0]
        self.end_point_y = end_point[1]
        self.end_point_theta = end_point[2]
        self.parking_lot_0x = parking_lot[0][0]
        self.parking_lot_0y = parking_lot[0][1]
        self.parking_lot_1x = parking_lot[1][0]
        self.parking_lot_1y = parking_lot[1][1]
        self.parking_lot_2x = parking_lot[2][0]
        self.parking_lot_2y = parking_lot[2][1]
        self.parking_lot_3x = parking_lot[3][0]
        self.parking_lot_3y = parking_lot[3][1]
        self.half_lot_width = 1.25
        self.vehicle_width = 1.8
        self.L_safe_1 = 0.0  # 0
        self.L_safe_2 = 0.0
        self.L_safe_3 = 0.0
        self.min_radius = 6.0
        self.vehicle_l_f = 4.0
        self.vehicle_len = 5.0
        self.road_width = 3.50     # 4.0
        self.ParkingType_num = 1 # vertical     -1 parallel


    def GenerateVehicle(self, pose_x, pose_y, pose_theta):
        p0_x = pose_x + (math.sqrt((self.vehicle_width / 2.0) * (self.vehicle_width / 2.0) + (self.vehicle_l_f) * (self.vehicle_l_f)) * math.cos(pose_theta - (math.atan(self.vehicle_width / 2.0) / (self.vehicle_l_f))))
        p0_y = pose_y + (math.sqrt((self.vehicle_width / 2.0) * (self.vehicle_width / 2.0) + (self.vehicle_l_f) * (self.vehicle_l_f)) * math.sin(pose_theta - (math.atan(self.vehicle_width / 2.0) / (self.vehicle_l_f))))
        p1_x = p0_x - self.vehicle_len * math.cos(pose_theta)
        p1_y = p0_y - self.vehicle_len * math.sin(pose_theta)
        p2_x = p1_x - self.vehicle_width * math.sin(pose_theta)
        p2_y = p1_y + self.vehicle_width * math.cos(pose_theta)
        p3_x = p0_x - self.vehicle_width * math.cos(math.pi / 2 - pose_theta)
        p3_y = p0_y + self.vehicle_width * math.sin(math.pi / 2 - pose_theta)
        p4_x = pose_x + (math.sqrt((self.vehicle_width / 2.0) * (self.vehicle_width / 2.0) + (self.vehicle_l_f) * (self.vehicle_l_f)) * math.cos(pose_theta - (math.atan(self.vehicle_width / 2.0) / (self.vehicle_l_f))))
        p4_y = pose_y + (math.sqrt((self.vehicle_width / 2.0) * (self.vehicle_width / 2.0) + (self.vehicle_l_f) * (self.vehicle_l_f)) * math.sin(pose_theta - (math.atan(self.vehicle_width / 2.0) / (self.vehicle_l_f))))
        result = []
        result.append(p0_x)   # UPPER RIGHT POINT X
        result.append(p0_y)  # UPPER RIGHT POINT Y
        result.append(p1_x)   # LOWER RIGHT POINT X
        result.append(p1_y)  # LOWER RIGHT POINT Y
        result.append(p2_x)   # LOWER LEFT POINT X
        result.append(p2_y)  # LOWER LEFT POINT Y
        result.append(p3_x)  # UPPER LEFT POINT X
        result.append(p3_y)  # UPPER LEFT POINT Y
        return result

    def first_point(self):
        L_op = self.min_radius - self.vehicle_width / 2 - self.L_safe_1
        # ipdb.set_trace()
        # 缺 acos里的参数 的范围规范
        print("self.min_radius - self.half_lot_width) / L_op)",(self.min_radius - self.half_lot_width) / L_op)
        alpha = math.acos((self.min_radius - self.half_lot_width) / L_op)
        self.first_point_theta = self.end_point_theta
        self.first_point_x = self.parking_lot_0x- self.half_lot_width * math.cos(self.end_point_theta - math.pi / 2) + L_op * math.sin(alpha) * math.sin(self.end_point_theta - math.pi / 2)
        self.first_point_y = self.parking_lot_0y - self.half_lot_width * math.sin(self.end_point_theta - math.pi / 2) - L_op * math.sin(alpha) * math.cos(self.end_point_theta - math.pi / 2)
        return self.first_point_x ,self.first_point_y,self.first_point_theta

    def second_point(self):
        L_op = self.min_radius - self.vehicle_width / 2 - self.L_safe_1
        alpha = math.acos((self.min_radius - self.half_lot_width) / L_op)
        beta1 = 0
        pose_x = self.first_point_x
        pose_y = self.first_point_y
        pose_theta = self.first_point_theta
        delta_beta1 = math.pi / 20
        points = []

        while ((0 <= beta1)):
            pose_theta = self.end_point_theta - beta1
            pose_x = self.first_point_x + self.min_radius * (math.cos(math.pi / 2 - self.end_point_theta) - math.cos(
                math.pi / 2 - self.end_point_theta + beta1))
            pose_y = self.first_point_y + self.min_radius * (-1 * math.sin(math.pi / 2 - self.end_point_theta) + math.sin(
                math.pi / 2 - self.end_point_theta + beta1))
            result = self.GenerateVehicle(pose_x, pose_y, pose_theta)
            x = result[6]
            y = result[7]
            delta_s = 0.1
            if (math.sqrt((x - self.parking_lot_0x) * (x - self.parking_lot_0x) + (y - self.parking_lot_0y) * (
                    y - self.parking_lot_0y)) >= (self.road_width - self.L_safe_3) + delta_s):
                break
            if ((math.pi / 2 <= beta1 and beta1 <= math.pi / 2 + delta_beta1)):
                break

            second_point_x = pose_x
            second_point_y = pose_y
            second_point_theta = self.end_point_theta - beta1
            points.append([second_point_x, second_point_y, second_point_theta])
            beta1 += delta_beta1

        # print("!!!!!",points)
        return second_point_x, second_point_y, second_point_theta, beta1, points

    def third_point(self, second_point_x, second_point_y, second_point_theta, beta1):
        beta2 = 0
        delta_beta2 = math.pi / 50.0
        pose_x = second_point_x
        pose_y = second_point_y
        pose_theta =self.end_point_theta - beta1 # M_PI / 2.0 - beta1;

        circle_point_x = pose_x - self.min_radius * (math.cos(math.pi / 2 - second_point_theta))
        circle_point_y = pose_y + self.min_radius * (math.sin(math.pi / 2 - second_point_theta))
        should_exit = False
        while (not(0 <= beta2 and (math.pi / 2 - delta_beta2 <= beta1 +beta2 and beta1 +beta2 <= math.pi / 2 + delta_beta2))):
            pose_theta = second_point_theta - beta2  # M_PI / 2.0 - beta2 - beta1
            pose_x = circle_point_x + self.min_radius * (math.sin(pose_theta))
            pose_y = circle_point_y - self.min_radius * (math.cos(pose_theta))
            result = self.GenerateVehicle(pose_x, pose_y, pose_theta)
            xCoord = result[2]
            yCoord = result[3]
            delta_s = 0.8;
            if (not(( xCoord - self.parking_lot_3x) * (xCoord - self.parking_lot_3x) + (yCoord - self.parking_lot_3y) * (
                    yCoord - self.parking_lot_3y) >= self.L_safe_2 * self.L_safe_2 + delta_s)):
                should_exit = True
                break

            if (should_exit):
                break  # // 跳出while循环

            beta2 += delta_beta2
        return pose_x, pose_y, pose_theta, beta2

    def CalcuCycleFirstPath(self,third_point_x, third_point_y, third_point_theta,sum_beta):
        beta3 = 0
        pose_x = third_point_x
        pose_y = third_point_y
        pose_theta = third_point_theta
        delta_beta3 = math.pi / 120.0
        fourth_points = []

        circle_point_x = third_point_x - 1 * self.min_radius * math.cos(
            math.pi / 2 + self.ParkingType_num * third_point_theta)  #// +  std::cos(sum_beta + (M_PI / 2) * ParkingType_num);
        circle_point_y = third_point_y - self.ParkingType_num * self.min_radius * math.sin(
            math.pi / 2 + self.ParkingType_num * third_point_theta) # // + std::sin(sum_beta - (M_PI / 2) * ParkingType_num);
        # std::cout << "beta3 = " << beta3 << "     target_pose_.theta  = " << target_pose_.theta << "    pose.theta  = " << pose.theta << std::endl;
        # std::cout << "circle_point.x = " << circle_point.x << "   circle_point.y = " << circle_point.y << std::endl;
        while ((0 <= beta3 and beta3 <= math.pi / 2)):
            pose_theta = third_point_theta - self.ParkingType_num * beta3   # // need to be checked
            pose_x = circle_point_x - self.ParkingType_num * self.min_radius * math.sin(pose_theta);
            pose_y = circle_point_y + self.ParkingType_num * self.min_radius * math.cos(pose_theta);
            # std::cout << "pose111.x y = " << pose.x << "      " << pose.y << "pose111.theta  = " << pose.theta << std::endl;

            result = self.GenerateVehicle(pose_x, pose_y, pose_theta)    # to be checked
            delta_s = 0.5
            if (self.ParkingType_num == 1):
                x = result[6]
                y = result[7]
                if (math.sqrt((x - self.parking_lot_0x) * (x - self.parking_lot_0x) + (y - self.parking_lot_0y) * (
                        y - self.parking_lot_0y)) >= (self.road_width - self.L_safe_3) + delta_s):
                    break

            if (self.ParkingType_num == -1):
                x1 = result[0]
                y1 = result[1]
                x2 = result[2]
                y2 = result[3]
            # std::
            #     cout << "x1 y1 =  " << x1 << "   " << y1 << "parking_lot_[0].x y =  " << parking_lot_[0].x << "   " <<
            #     parking_lot_[0].y << std::endl;
                if (math.sqrt((x1 - self.parking_lot_0x) * (x1 - self.parking_lot_0x) + (y1 - self.parking_lot_0y) * (
                        y1 - self.parking_lot_0y)) <= self.L_safe_1 + delta_s):
                    break
    #     // if ((-delta_beta3 <= std::sin(pose.theta) <= delta_beta3) )
    #     // {
    #
    # // std::cout << "---------------!!!!!!!FINISH PARKING111111!!!!!!!!!!!----------------" << std::endl;
    # // break;
    # //}
                if ((x2 >= self.parking_lot_0x and y2 >= self.parking_lot_0y + self.L_safe_1 and (-delta_beta3 <= math.sin(pose_theta) <= delta_beta3)) ):
                    break

                if (pose_y + self.min_radius * math.sin(pose_theta) >= self.parking_lot_0y + self.L_safe_1 * math.cos(self.end_point_theta) + self.vehicle_width / 2):
                # {
                # std::cout << "pose.x y = " << pose.x << "      " << pose.y << "pose.theta  = " << pose.theta << std::endl;
                # std::cout << "---------------Start Final Path-----------------------" << std::endl;
                    fourth_point_theta = pose_theta
                    beta = 0     # // sum_beta + beta3; // beta3;
                    delta_beta = math.pi / 50
                # std::cout << "pose.x y = " << pose.x << "      " << pose.y << "pose.theta  = " << pose.theta << std::endl;
                    circle_point1_x = pose_x + self.min_radius * math.sin(pose_theta)
                    circle_point1_y = pose_y - self.min_radius * math.cos(pose_theta)
                    while (not((self.end_point_theta -delta_beta <= pose_theta and pose_theta <= self.end_point_theta + delta_beta) or (
                            math.pi / 2 - delta_beta <= pose_theta and pose_theta <= math.pi / 2 + delta_beta)) ):
                    # check 一下 判断条件
                        pose_theta = fourth_point_theta - beta
                        pose_x = circle_point1_x - self.min_radius * math.sin(pose_theta)
                        pose_y = circle_point1_y + self.min_radius * math.cos(pose_theta)  # // beta3 + sum_beta - beta;

                    # std::cout << "pose222.x y = " << pose.x << "      " << pose.y << "pose222.theta  = " << pose.theta << std::endl;
                        beta += delta_beta

                    break

            beta3 += delta_beta3
            fourth_point_x = pose_x
            fourth_point_y = pose_y
            fourth_point_theta = pose_theta
            fourth_points.append([fourth_point_x, fourth_point_y, fourth_point_theta])

        y1 = result[1]
        beta3 = beta3
        fourth_point_x = pose_x
        fourth_point_y = pose_y
        fourth_point_theta = pose_theta
        # std::cout << "-------------------CYCLE FIRST STEP SUCCESSS--------------------" << std::endl;
        return fourth_point_x, fourth_point_y, fourth_point_theta, fourth_points, beta3

    def CalcuCycleSecondPath(self,fourth_point_x, fourth_point_y, fourth_point_theta, sum_beta):
        beta4 = 0
        delta_beta4 = math.pi / 200.0
        pose_x = fourth_point_x
        pose_y = fourth_point_y
        pose_theta = fourth_point_theta

        circle_point_x = fourth_point_x - self.ParkingType_num * self.min_radius * math.cos(math.pi / 2 - fourth_point_theta)
        circle_point_y = fourth_point_y + self.ParkingType_num * self.min_radius * math.sin(math.pi / 2 - fourth_point_theta)
        # std::cout << "circle_point.x =  " << circle_point.x << "circle_point.y = " << circle_point.y << std::endl;
        while (0 <= beta4):
            pose_theta = fourth_point_theta + beta4
            pose_x = circle_point_x + self.ParkingType_num * self.min_radius * math.sin(pose_theta)
            pose_y = circle_point_y - self.ParkingType_num * self.min_radius * math.cos(pose_theta)
            # std::cout << "pose333.x y = " << pose.x << "      " << pose.y << "pose333.theta  = " << pose.theta << std::endl;
            result = self.GenerateVehicle(pose_x, pose_y, pose_theta)
            xCoord1 = result[2]
            yCoord1 = result[3]
            # std::cout << "xCoord1 =" << xCoord1 << "   yCoord1 = " << yCoord1 << std::endl;
            # std::cout << "xCoord1 - parking_lot_[1].x = " << xCoord1 - parking_lot_[
            #     1].x << " yCoord1 - parking_lot_[1].y =   " << yCoord1 - parking_lot_[1].y << std::endl;
            delta_s = 0.5;

            if (self.ParkingType_num == 1):
                if (not(math.sqrt((xCoord1 - self.parking_lot_3x) * (xCoord1 - self.parking_lot_3x) + (yCoord1 - self.parking_lot_3y) * (yCoord1 - self.parking_lot_3y)) >= self.L_safe_1  + delta_s)):
            # {std::
            #     cout << "---------------!!!!!!!REACH THE ULTIMATE COLLISION POINT!!!!!!!!!!!----------------" << std::endl;
                    break
                if (math.pi / 2 - delta_beta4 <= sum_beta + beta4 and sum_beta + beta4 <= math.pi / 2 + delta_beta4):
            # {std::
            #     cout << "---------------!!!!!!!!!!SATISFY THE FINAL ANGLE !!!!!!!!!!!!!!!!!----------------" << std::endl;
                    break

            if (self.ParkingType_num == -1):
                if ((math.sqrt((xCoord1 - self.parking_lot_1x) * (xCoord1 - self.parking_lot_1x) + (yCoord1 - self.parking_lot_1y) * (yCoord1 - self.parking_lot_1y)) <= self.L_safe_1  + delta_s)):
        #     {
        # std::cout << "---------------!!!!!!!REACH THE ULTIMATE COLLISION POINT!!!!!!!!!!!----------------" << std::endl;
                    break
                if ((yCoord1 <= self.parking_lot_1y)):
                # {
                # std::cout << "---------------!!!!!!!REACH THE ULTIMATE COLLISION POINT!!!!!!!!!!!----------------" << std::endl;
                    break
                if (math.pi / 2 + self.end_point_theta - delta_beta4 <= pose_theta and pose_theta >= math.pi / 2 + self.end_point_theta + delta_beta4):
                # {
                # std::cout << "---------------!!!!!!!SATISFY THE FINAL ANGLE(PATH 2) !!!!!!!!!!!----------------" << std::endl;
                    break

            beta4 += delta_beta4

        return beta4




def main():
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(3.0)

        # Retrieve the world that is currently running
        world = client.get_world()
        client.load_world('Town04')  # Town02(没有车道线) 和 Town04 有一个停车场
        world = client.get_world()
        if world is not None:
            print('find the world !')
        origin_settings = world.get_settings()

        # set sync mode
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        blueprint_library = world.get_blueprint_library()

        spawn_point = carla.Transform(carla.Location(x=285.5, y=-201, z=2.3), carla.Rotation(pitch=0.000000, yaw=math.pi/2 , roll=0.000000))
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_synchronous_mode(True)

        # create the blueprint library
        ego_vehicle_bp = blueprint_library.filter('model3')[0]
        ego_vehicle_bp.set_attribute('role_name', "hero")
        ego_vehicle_bp.set_attribute('color', '0, 0, 0')
        # spawn the vehicle
        vehicle = world.spawn_actor(ego_vehicle_bp, spawn_point)

        spectator = world.get_spectator()
        transform = vehicle.get_transform()
        # spectator.set_transform(carla.Transform(transform.location + carla.Location(z=30),
        #                                         carla.Rotation(pitch=-90)))
        spectator.set_transform(carla.Transform(carla.Location(x=278.5, y=-201, z=23.3), carla.Rotation(pitch=-90.000000, yaw=0, roll=0.000000)))
        vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.0))

        # wait for a moment to let the vehicle spawn
        time.sleep(2.0)
        # we need to tick the world once to let the client update the spawn position
        world.tick()

        # ipdb.set_trace()
        # create the behavior agent
        agent = BasicAgent(vehicle)
        agent = BehaviorAgent(vehicle, behavior='normal')

        start_point = [285.5, -201,0]
        start_point_location = carla.Location(x=285.5, y=-201, z=0.5)
        # world.debug.draw_point(start_point_location, size=0.05, color=carla.Color(r=0, g=0, b=0), life_time=-1.0)

        end_point = [280, -204.3, math.pi / 2]
        end_point_location = carla.Location(x=280, y=-204.3, z=0.5)
        # world.debug.draw_point(end_point_location, size=0.05, color=carla.Color(r=255, g=0, b=255), life_time=-1.0)

        # 3  |       | 0
        #    |       |
        #    |   *   |
        # 2  | _____ | 1
        parking_lot = [[282, -203.00], [277.65, -203], [277.65, -205.65], [282, -205.65]]
        point0 = carla.Location(x=282, y=-203.00, z=0.5)
        # world.debug.draw_point(point0, size=0.05, color=carla.Color(r=255, g=0, b=0), life_time=-1.0)
        point1 = carla.Location(x=277.65, y=-203.00, z=0.5)
        # world.debug.draw_point(point1, size=0.05, color=carla.Color(r=0, g=0, b=255), life_time=-1.0)
        point2 = carla.Location(x=277.65, y=-205.65, z=0.5)
        # world.debug.draw_point(point2, size=0.05, color=carla.Color(r=0, g=255, b=0), life_time=-1.0)
        point3 = carla.Location(x=282, y=-205.65, z=0.5)
        # world.debug.draw_point(point3, size=0.05, color=carla.Color(r=255, g=255, b=0), life_time=-1.0)


        # world.debug.draw_line(point0, point1, thickness=0.1, color=carla.Color(r=255, g=0, b=0), life_time=-1.0)
        # world.debug.draw_line(point1, point2, thickness=0.1, color=carla.Color(r=255, g=0, b=0), life_time=-1.0)
        # world.debug.draw_line(point2, point3, thickness=0.1, color=carla.Color(r=255, g=0, b=0), life_time=-1.0)

        traj = Traj(start_point, end_point, parking_lot)
        wp =carla.Transform(carla.Location(x=282, y=-198, z=0.5), carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))


        result = traj.GenerateVehicle(end_point[0], end_point[1], 0)
        vehicle0 = carla.Location(x=result[0], y=result[1], z=0.5)
        vehicle1 = carla.Location(x=result[2], y=result[3], z=0.5)
        vehicle2 = carla.Location(x=result[4], y=result[5], z=0.5)
        vehicle3 = carla.Location(x=result[6], y=result[7], z=0.5)
        world.debug.draw_line(vehicle0, vehicle1, thickness=0.1, color=carla.Color(r=255, g=0, b=0), life_time=-1.0)
        world.debug.draw_line(vehicle1, vehicle2, thickness=0.1, color=carla.Color(r=255, g=0, b=0), life_time=-1.0)
        world.debug.draw_line(vehicle2, vehicle3, thickness=0.1, color=carla.Color(r=255, g=0, b=0), life_time=-1.0)
        world.debug.draw_line(vehicle3, vehicle0, thickness=0.1, color=carla.Color(r=255, g=0, b=0), life_time=-1.0)

        start_time = time.time()
        first_point_x, first_point_y, first_point_theta = traj.first_point()

        # ipdb.set_trace()
        # generate the route
        destination1_location = carla.Location(x=first_point_x, y=first_point_y +0.5, z=0.5)
        world.debug.draw_point(destination1_location, size=0.05, color=carla.Color(r=255, g=255, b=255), life_time=-1.0)
        print(" destination1_location_x", destination1_location.x)
        print(" destination1_location_y", destination1_location.y)

        result = traj.GenerateVehicle(first_point_x, first_point_y+0.5, 0)
        vehicle0 = carla.Location(x=result[0], y=result[1], z=0.5)
        vehicle1 = carla.Location(x=result[2], y=result[3], z=0.5)
        vehicle2 = carla.Location(x=result[4], y=result[5], z=0.5)
        vehicle3 = carla.Location(x=result[6], y=result[7], z=0.5)
        world.debug.draw_line(vehicle0, vehicle1, thickness=0.1, color=carla.Color(r=255, g=0, b=0), life_time=-1.0)
        world.debug.draw_line(vehicle1, vehicle2, thickness=0.1, color=carla.Color(r=255, g=0, b=0), life_time=-1.0)
        world.debug.draw_line(vehicle2, vehicle3, thickness=0.1, color=carla.Color(r=255, g=0, b=0), life_time=-1.0)
        world.debug.draw_line(vehicle3, vehicle0, thickness=0.1, color=carla.Color(r=255, g=0, b=0), life_time=-1.0)


        # agent.set_destination(vehicle.get_location(), destination1_location)

        # world.debug.draw_line(destination1_location, end_point_location, thickness=0.1, color=carla.Color(r=0, g=0, b=0), life_time=-1.0)
        second_points =[]
        second_point_x, second_point_y, second_point_theta, beta1,second_points = traj.second_point()

        end_time = time.time()
        print("start_time",start_time)
        print("end_time", end_time)
        print("calculation time: ", end_time - start_time)
        destination2_location = carla.Location(x=second_point_x, y=second_point_y+0.5, z=0.5)
        print("destination2_location",destination2_location)
        world.debug.draw_point(destination2_location, size=0.05, color=carla.Color(r=255, g=0, b=0), life_time=-1.0)
        i = 0
        print("second_points",second_points)
        for point in second_points:
            # 假设 point 是一个包含 x, y 坐标的元组或列表
            delta_x = [1,2.8,3.2 ,3.5,3.9]
            delta_theta = [0.2,math.pi / 5,math.pi / 5,math.pi / 5,0.9424777960769379]
            location = carla.Location(x=point[0]+ delta_x[i], y=point[1]+0.5, z=0.5)
            # 绘制点
            world.debug.draw_point(location, size=0.05, color=carla.Color(r=0, g=255, b=0), life_time=-1.0)
            result = traj.GenerateVehicle(point[0] + delta_x[i], point[1]+0.5, -(point[2] - math.pi /2 - delta_theta[i]) )    #  4
            vehicle0 = carla.Location(x=result[0], y=result[1], z=0.5)
            vehicle1 = carla.Location(x=result[2], y=result[3], z=0.5)
            vehicle2 = carla.Location(x=result[4], y=result[5], z=0.5)
            vehicle3 = carla.Location(x=result[6], y=result[7], z=0.5)
            world.debug.draw_line(vehicle0, vehicle1, thickness=0.1, color=carla.Color(r=255, g=0, b=0), life_time=-1.0)
            world.debug.draw_line(vehicle1, vehicle2, thickness=0.1, color=carla.Color(r=255, g=0, b=0), life_time=-1.0)
            world.debug.draw_line(vehicle2, vehicle3, thickness=0.1, color=carla.Color(r=255, g=0, b=0), life_time=-1.0)
            world.debug.draw_line(vehicle3, vehicle0, thickness=0.1, color=carla.Color(r=255, g=0, b=0), life_time=-1.0)
            # 等待0.5秒
            time.sleep(2)
            i += 1

        sum_beta = beta1
        delta_beta = math.pi / 20.0 # 20
        if (not(math.pi / 2 - delta_beta <= sum_beta and sum_beta <= math.pi / 2 + delta_beta)):
            third_point_x, third_point_y, third_point_theta, beta2 = traj.third_point(second_point_x, second_point_y, second_point_theta, beta1)
            sum_beta = beta1 + beta2

            while (not(math.pi / 2 - delta_beta <= sum_beta and sum_beta <= math.pi / 2 + delta_beta)):
                # ipdb.set_trace()
                fourth_point_x, fourth_point_y, fourth_point_theta,  fourth_points, beta3 = traj.CalcuCycleFirstPath(third_point_x, third_point_y, third_point_theta, sum_beta)
                print("fourth points :",fourth_points)
                i = 0
                for point in fourth_points:
                    # 假设 point 是一个包含 x, y 坐标的元组或列表
                    delta_x = [2.5,2.45,2.35,2.35,2.25]
                    delta_y = [0,-1.2,-2.3,-2.7,-3.4]
                    delta_theta = [math.pi / 5,math.pi/3.8, math.pi/3,math.pi/2.6,math.pi/2.5]
                    location = carla.Location(x=point[0] + delta_x[i], y=point[1] + delta_y[i], z=0.5)
                    # 绘制点
                    world.debug.draw_point(location, size=0.05, color=carla.Color(r=0, g=255, b=0), life_time=-1.0)
                    result = traj.GenerateVehicle(point[0] + delta_x[i], point[1] + delta_y[i],
                                                  -(point[2] - math.pi / 2 - delta_theta[i]))  # 4
                    vehicle0 = carla.Location(x=result[0], y=result[1], z=0.5)
                    vehicle1 = carla.Location(x=result[2], y=result[3], z=0.5)
                    vehicle2 = carla.Location(x=result[4], y=result[5], z=0.5)
                    vehicle3 = carla.Location(x=result[6], y=result[7], z=0.5)
                    world.debug.draw_line(vehicle0, vehicle1, thickness=0.1, color=carla.Color(r=0, g=255, b=0),
                                          life_time=-1.0)
                    world.debug.draw_line(vehicle1, vehicle2, thickness=0.1, color=carla.Color(r=0, g=255, b=0),
                                          life_time=-1.0)
                    world.debug.draw_line(vehicle2, vehicle3, thickness=0.1, color=carla.Color(r=0, g=255, b=0),
                                          life_time=-1.0)
                    world.debug.draw_line(vehicle3, vehicle0, thickness=0.1, color=carla.Color(r=0, g=255, b=0),
                                          life_time=-1.0)
                    # 等待0.5秒
                    time.sleep(2)
                    i += 1


                sum_beta += beta3

                if ((math.pi / 2 - delta_beta <= sum_beta and sum_beta <= math.pi / 2 + delta_beta)):
                    break

                beta4 = traj.CalcuCycleSecondPath(fourth_point_x, fourth_point_y, fourth_point_theta, sum_beta)
                sum_beta += beta4;

                if (math.pi / 2 - delta_beta <= sum_beta and sum_beta <= math.pi / 2 + delta_beta):
                    break

        # create the behavior agent
        agent = BehaviorAgent(vehicle, behavior='normal')

        start_location = carla.Location(x=285.5, y=-201, z=0.5)
        destination_location = carla.Location(x=280, y=-204.3, z=0.5)
        # generate the route
        # agent.set_destination(start_location, destination_location)
        _local_planner = LocalPlanner(vehicle)
        # wp = _local_planner.get_plan()
        wp = world.get_map()
        global_route = GlobalRoutePlanner( wp,1)
      

        while True:
            # agent.update_information(vehicle)
            world.tick()
            local_planner = LocalPlanner(vehicle)
            if len(local_planner._waypoints_queue) < 1:
                print('======== Success, Arrivied at Target Point!')
                break
            # if agent.done():
            #     print("The target has been reached, stopping the simulation")
            #     break
            control = agent.run_step()
            vehicle.apply_control(control)



            # if len(agent.get_local_planner().waypoints_queue) == 0:
            #     print("Target reached, mission accomplished...")
            #     break

    finally:
        world.apply_settings(origin_settings)
        vehicle.destroy()

if __name__ == '__main__':
    main()
