import numpy as np
import asyncio
import mavsdk
import time 
import threading
import rospy


from mavsdk             import System
from data_hub           import DataHub
from connector			import Connector
from sensor_hub         import SensorHub
from action_planner		import ActionPlanner
from planner			import Planner
from std_msgs.msg       import Float32MultiArray
from sensor_msgs.msg    import PointCloud
from geometry_msgs.msg  import Point32
from nav_msgs.msg       import Odometry
from client 		    import Client






class FCReader(threading.Thread):
	'''
	FC Reader
		Connector
		- MAVLINK connection by MAVSDK API server
		Sensorhub
		
		- reads telemetry data
		- subscribes waypoint topic, img topic
	'''
	def __init__(self, drone, datahub):
		super().__init__()
		self.drone = drone
		self.datahub = datahub

		self.connector = Connector(self.drone,self.datahub)
		self.sensorhub = SensorHub(self.drone,self.datahub)

	
	async def connect(self):
		await self.connector.connect() # connect to PX4


	def run(self):

		print("===== FCReader start =====")

		# Create new Event loop for FC Reader Thread
		telem_event_loop = asyncio.new_event_loop()
		asyncio.set_event_loop(telem_event_loop)
		
		# run connect function & telemetry reader tasks
		telem_event_loop.run_until_complete(\
			asyncio.gather( self.connect(),\
							self.sensorhub.telem_posvel(),\
							self.sensorhub.telem_posglobal(),\
							self.sensorhub.telem_att_eular(),\
							self.sensorhub.telem_att_quat(),\
							self.sensorhub.telem_home(),\
							self.sensorhub.telem_flightmode(),\
							self.sensorhub.telem_armed(),\
							self.sensorhub.telem_in_air()))





class FCWriter(threading.Thread):
	'''
	FC Writer
		Connector
		- MAVLINK connection by MAVSDK API server
		ActionPlanner
		
		- plans which action to do and execute it
	'''
	def __init__(self, drone, datahub):
		super().__init__()
		self.drone = drone
		self.datahub = datahub

		self.connector = Connector(self.drone, self.datahub)
		self.action_planner = ActionPlanner(self.drone, self.datahub)


	async def connect(self):
		await self.connector.connect() # connect to PX4


	def run(self):

		print("===== FCWriter start =====")
		
		# Create new Event loop for FC Writer Thread
		control_event_loop = asyncio.new_event_loop()
		asyncio.set_event_loop(control_event_loop)
		
		# run connect function & controller
		control_event_loop.run_until_complete(\
			asyncio.gather( self.connect(),\
							self.action_planner.run()))


class Comunicator(threading.Thread):

	def __init__(self, server_ip, server_port, datahub):
		super().__init__()
		self.ip = server_ip
		self.port = server_port
		self.datahub = datahub
		
	def run(self):
		self.client = Client(self.ip,self.port,self.datahub)
		self.client.start()

	


class Visualizer(threading.Thread):

	def __init__(self, datahub):
		super().__init__()
		self.datahub = datahub

		self.trajec = PointCloud()
		self.trajec.header.frame_id = "dok3"       
		self.trajec_pub = rospy.Publisher("/traj",PointCloud,queue_size=1)

		self.wp_viz = PointCloud()
		self.wp_viz.header.frame_id = "dok3"       
		self.wp_viz_pub = rospy.Publisher("/waypoints",PointCloud,queue_size=1)

		self.wp_jps_viz = PointCloud()
		self.wp_jps_viz.header.frame_id = "dok3"       
		self.wp_jps_viz_pub = rospy.Publisher("/jps_wp",PointCloud,queue_size=1)

		self.pose_viz = Odometry()
		self.pose_viz.header.frame_id = "dok3"
		self.pose_viz_pub = rospy.Publisher("/pose", Odometry, queue_size=1)
        
		self.jps_map_pub = rospy.Publisher("/map",Float32MultiArray,queue_size=1)
	
	def run(self):

		while not rospy.is_shutdown():
			
			if len(self.datahub.traj) != 0:
				
				self.trajec.points = []

				for i in range(len(self.datahub.traj[0])):

					point = Point32(self.datahub.traj[1,i],self.datahub.traj[0,i],-self.datahub.traj[2,i])

					self.trajec.points.append(point)
				



			if len(self.datahub.traj) != 0:
				
				for i in range(len(self.datahub.traj[0])):

					self.pose_viz.pose.pose.position.x =  self.datahub.posvel_ned[1]
					self.pose_viz.pose.pose.position.y =  self.datahub.posvel_ned[0]
					self.pose_viz.pose.pose.position.z = -self.datahub.posvel_ned[2]

					self.pose_viz.pose.pose.orientation.w = self.datahub.attitude_quat[0]
					self.pose_viz.pose.pose.orientation.x = self.datahub.attitude_quat[1]
					self.pose_viz.pose.pose.orientation.y = self.datahub.attitude_quat[2]
					self.pose_viz.pose.pose.orientation.z = self.datahub.attitude_quat[3]




				rostime = rospy.Time.now()
				self.trajec.header.stamp = rostime
				self.pose_viz.header.stamp = rostime
				self.pose_viz_pub.publish(self.pose_viz)
				self.trajec_pub.publish(self.trajec)

			
			# if len(self.datahub.wp_viz) != 0:

			# 	print(self.datahub.wp_viz)

			# 	self.wp_viz.points = []

			# 	if len(self.datahub.wp_viz) != 0:

			# 		for i in range(len(self.datahub.wp_viz[0])):

			# 			point = Point32(self.datahub.wp_viz[0,i],self.datahub.wp_viz[1,i],self.datahub.wp_viz[2,i])

			# 			self.wp_viz.points.append(point)
					
			# 	self.wp_viz_pub.publish(self.wp_viz)



			if len(self.datahub.jps_map) != 0:

				map_msg = Float32MultiArray()

				map_msg.data = self.datahub.jps_map.tolist()

				self.jps_map_pub.publish(map_msg)
			
			time.sleep(0.01)




class Master:

	def __init__(self, delt, traj_update_period, voxel_size, threshold, max_range, expension_size, bottom_cam_mtx, bottom_dist_coeff, ip, port, visualize=True, communication=True):
		

		self.datahub = DataHub(delt, traj_update_period, voxel_size, threshold, max_range, expension_size, bottom_cam_mtx, bottom_dist_coeff)	

		self.drone_I = System()
		self.drone_O = System()

		self.fc_writer = FCWriter(self.drone_I, self.datahub)
		self.fc_reader = FCReader(self.drone_O, self.datahub)
		self.communicator = Comunicator(ip,port,self.datahub)
		self.visualize = Visualizer(self.datahub)

		self.fc_writer.daemon = True
		self.fc_reader.daemon = True
		self.visualize.daemon = True
		self.communicator.daemon = True
		
		
		self.fc_writer.start()
		self.fc_reader.start()

		if communication:
			self.communicator.start()

		if visualize:
			self.visualize.start()

		self.planner = Planner(self.drone_I, self.datahub)	

	def run(self):	
		
		rospy.init_node("dok3_main")
		self.planner.run()





if __name__ == "__main__":

	# Controller

	delt = 0.1 				# Control Time Interval for dicrete-time dynamic system 

	traj_update_period = 1  # Period of updating trajectory 

	# LiDAR Processor

	voxel_size = 0.5		# voxel size [m]

	threshold = 1			# threshold for voxelization

	max_range = 20			# the maximum range of LiDAR 

	expension_size = 3
	
	# Image Procesor

	bottom_cam_mtx = np.array([[472.98538427,   0,         384.27642545],
							   [  0,         473.94130531, 226.72986825],
							   [  0,           0,           1          ]])

	bottom_dist_coeff = np.array([[ 0.15666287, -0.36135453, -0.00808564, -0.00128795,  0.18481056]])


	server_ip = '165.246.139.32'
	server_port = 9502

	master = Master(delt, traj_update_period,\
                    voxel_size, threshold, max_range, expension_size,\
                    bottom_cam_mtx, bottom_dist_coeff,\
					server_ip,server_port,\
					visualize=True,
					communication=False)        

	master.run()

