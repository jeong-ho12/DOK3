import numpy as np
import rospy
import asyncio
import mavsdk
import time 

from mavsdk             import System
from std_msgs.msg       import Float32MultiArray
from geometry_msgs.msg  import Point32


class DataHub:
    
    def __init__(self,  delt, traj_update_period,\
                        grid_size, threshold, max_range, expension_size,\
                        bottom_cam_mtx, bottom_dist_coeff, SITL):



        ###### Parameters ######

        ''' Controller '''

        self.delt = delt

        self.traj_update_period = traj_update_period

        self.v_mean = 1

        self.heading_wp =  0

        ''' LiDAR Processor '''

        self.grid_size = grid_size

        self.target_points = None

        self.threshold = threshold

        self.max_range = max_range

        self.expension_size = expension_size

        ''' Image Procesor '''

        self.bottom_cam_mtx = bottom_cam_mtx

        self.bottom_dist_coeff = bottom_dist_coeff



        ###### Options ######

        self.SITL = SITL



        ###### Sensor data & Mission data & State & Action ######
        
        self.home = np.zeros(4)  

        ''' global position of home :
            
            lat (deg) : Lattitude
            lon (deg) : Longitude
           
            absolute_altitude_m : Altitude AMSL (above mean sea level) in metres
            relative_altitude_m :  Altitude relative to takeoff altitude in metres
          '''



        self.offboard_home_ned = np.zeros(3)  

        ''' offboard home ned :
            
            position.north_m 
            position.east_m
            position.down_m

            this home position is setpoint of NED frame for offboard flight  
          '''



        self.offboard_home_global = np.zeros(3)  

        ''' offboard home ned :
            
            latitude_deg : double
            longitude_deg : double

            absolute_altitude_m : float
                Altitude AMSL (above mean sea level) in metres

            relative_altitude_m : float
                Altitude relative to takeoff altitude in metres

            this home position is global setpoint for offboard flight  
          '''



        self.posvel_ned = np.zeros(6)

        ''' local position about the home position :
            
            position.north_m 
            position.east_m
            position.down_m

            velocity.down_m
            velocity.down_m
            velocity.down_m
        '''


        self.pos_global = np.zeros(4)

        ''' local position about the home position :
            Position type in global coordinates.
            latitude_deg : double
                Latitude in degrees (range: -90 to +90)
            longitude_deg : double
                Longitude in degrees (range: -180 to +180)
            absolute_altitude_m : float
                Altitude AMSL (above mean sea level) in metres
            relative_altitude_m : float
                Altitude relative to takeoff altitude in metres
        '''



        self.attitude_eular = np.zeros(3)

        ''' Eular angle attitude :
            roll_deg : roll angle
            pitch_deg : pitch angle        
            yaw_deg : yaw angle                
        '''


        self.attitude_quat = np.zeros(4)

        ''' Eular angle attitude :
            w, x, y, z 
        '''



        self.is_connected = False

        '''
            Check connection with PX4
        '''


        self.waypoints = None

        ''' Waypoints :
        
            Input for Trajectory Tracking mission
            for wp = [ n, e, d ]^T
            waypoints =  [ wp_0, wp_1, wp_2, ... , wp_n ]
        
        '''
        

        self.img_bottom = None

        '''
            Compressed image of bottom camera
        
        '''


        self.marker_position = None
        
        ''' Marker Position :
        
            position of the marker relative to drone
            marker_position = [ n, e, d ]^T 
        
        '''
        

        self.state = "Disarm"

        ''' Current State
            1. Disarm
            2. Arm 
            3. Takeoff
            4. Hold
            5. Trajectory
            6. PrepareLand
            7. Land
            10. EmergencyStop
            initial state is Disarm
        '''


        self.action = None

        ''' Current Action requested
            1.disarm
            2.arm
            3.hold
            4.tracking
            5.land
            Action is None unless requested       
        '''


        self.mission_input = None 

        ''' Mission Input
            0. e ( Emergency Stop : Failsafe mode landing )
            1. Arm
            2. Disarm
            3. Takeoff : target_altitude
            4. WP : mean_velocity & number_of_waypoints & waypoints
            5. Land
            
        '''


        self.flight_mode = None

        ''' Flight Modes
        
            1. LAND
            2. POSITION
            3. OFFBOARD
            4. ALTITUDE
            5. ACRO
            ...
        
        '''


        self.armed = False

        ''' Is Armed
            armed    : True
            disarmed : False
        
        '''


        self.is_in_air = False

        ''' Is in air
            in air    : True
            on ground : False
        
        '''

        ###### Trajectory & Waypoints & Map Vizualization ######

        self.traj = np.array([])

        self.wp_viz = np.array([])

        self.jps_map = np.array([])
        
        ##### veranda mean position #####
        self.vox_n = None
        self.vox_e = None
        self.vox_d = None


        self.cross_marker = np.array([])
        self.cross_marker_detected = False 

        #for perfect circle move
        self.circle_yaw_angle          = None
        self.circle_radius             = None
        self.circle_move_degree        = None

        #move toward marker#
        self.cross_marker                   = np.array([])
        self.cross_marker_detected          = False 

        self.first_marker_detected_angle    = None
        self.last_marker_detected_angle     = None
        self.height = 10
