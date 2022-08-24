import numpy as np
import asyncio
import mavsdk
import time

from sensor_hub              import SensorHub
from data_hub                import DataHub
from lib.trajectory_tracking import TrajectoryTracker
from lib.postion_estmation   import ArUcoPosEstimator
from lib.lidar_processor     import LiDARProcessor 
from lib.search              import Search
from mavsdk.offboard         import OffboardError,\
                                    VelocityNedYaw,\
                                    PositionNedYaw,\
                                    VelocityBodyYawspeed


class Controller:

    def __init__(self, drone, datahub):

        self.drone = drone
        self.datahub = datahub
        self.delt = datahub.delt

        self.lidar_processor = LiDARProcessor(self.datahub)
        self.traj = TrajectoryTracker(self.drone,self.datahub,self.lidar_processor)
        self.searcher = Search(self.drone,self.datahub,self.lidar_processor)
        self.marker = ArUcoPosEstimator()





    async def arm(self):

        print("Action : armed")
        await self.drone.action.arm()




    async def disarm(self):

        print("Action : disarmed")
        await self.drone.action.disarm()




    async def takeoff(self):

        print("Action : takeoff ...")


        target_pos = self.datahub.waypoints + np.reshape(self.datahub.posvel_ned[:3], (3,1))


        destination = np.vstack((target_pos,np.zeros((3,1))))
        destination = np.reshape(destination, (6,))
        wp = np.array([])

        await self.traj.trajectory_tracking(destination,wp,1)

        self.datahub.state = "Hold"
        self.datahub.action = "hold"




    async def hold(self):

        print("Action : hold ...")
        while self.datahub.state == "Hold":

            # keepout_vel = self.lidar_processor.safety_check()

            # await self.drone.offboard.set_velocity_body(
            #     VelocityBodyYawspeed(keepout_vel[0], keepout_vel[1], keepout_vel[2], 0.0))

            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

            await asyncio.sleep(0.1)




    async def wp_guidance(self):

        print("Action : waypoint guidance ...")
        dest_position = np.reshape(self.datahub.waypoints[:,-1],(3,1))
        destination = np.vstack((dest_position,np.zeros((3,1))))
        destination = np.reshape(destination, (6,))

        if len(self.datahub.waypoints[0]) == 1:
            wp = np.array([])

        else:

            wp = self.datahub.waypoints[:,:-1]

        await self.traj.trajectory_tracking(destination,wp,self.datahub.v_mean)

        self.datahub.waypoints = None
        self.datahub.mission_input = None
        self.datahub.state = "Hold"
        self.datahub.action = "hold"




    async def park(self):
        print("Action : land finding marker 90")
        while True:
            resize_width = np.shape(self.datahub.img_bottom)[1]
            ids,x,y,z = self.marker.run(90,self.datahub.img_bottom,self.datahub.cam_mtx,self.datahub.dist_coeff,"DICT_5X5_1000",resize_width)
            
            #unit vectorization
            x = x/10
            y = y/10
            z = z/10

            if x < 0.5 and y< 0.5:
                if z > 7:
                    await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.8, 0.0))
                    await asyncio.sleep(2)

                elif z < 2.2:
                    if x < 0.3 and y< 0.3:
                        await self.drone.action.land()
                    else: pass

                else:
                    await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.8, 0.0))
                    await asyncio.sleep(1)

            print(f'marker detected: {x},{y},{z}')
            unit_x = x / np.linalg.norm([x,y])
            unit_y = y / np.linalg.norm([x,y])
            
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(unit_x, unit_y, 0.0, 0.0))
            await asyncio.sleep(0.2)
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))





    async def land(self):

        print("Action : land")
        await self.drone.action.land()




    async def offboard_start(self):

        print("Offboard Starting : Setting initial setpoint ...")

        await self.drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, 0.0, 0.0))
        
        await self.drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, 0.0, 0.0))

        await self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

        print("Offboard Start")

        try:
            await self.drone.offboard.start()

            return True

        except OffboardError as error:

            print(f"Starting offboard mode failed with error code: \
                {error._result.result}")

            return False




    async def search_veranda(self):

        print('Action : search veranda')

        await self.searcher.run()



            


        # while True:
        #     if self.datahub.cross_marker_detected == True:
        #         self.move_toward_marker()
        #         self.datahub.mission_input = None
        #         self.datahub.state = "Hold"
        #         self.datahub.action = "hold"
        #         break
        
        #     print('qweqweqweqeqweqweqweqweqweqwe')
        #     radius = np.linalg.norm([self.datahub.vox_n, self.datahub.vox_e])
        #     degree = (1/radius)*180/np.pi
        #     print(f"circle mode {radius}m radius")
        #     await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 1.0, 0.0, -degree))
        #     await asyncio.sleep(0.1)




    async def move_toward_marker(self):
        print('marker detected')
        print(self.datahub.cross_marker)
        
        await self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 2.0, 0.0))
        await asyncio.sleep(5)
