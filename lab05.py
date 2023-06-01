import matplotlib as mpl
import numpy as np
import matplotlib.pyplot as plt
import rosbag
import json
import rospy
from nav_msgs.msg import OccupancyGrid


class map_t:
	size_x = 0
	size_y = 0
	res_x = 0
	res_y = 0
	map_array = np.array([[]])
	def __init__ (self, _map_size_x, _map_size_y, _res_x, _res_y):
		self.size_x = _map_size_x
		self.size_y = _map_size_y
		self.res_x = _res_x
		self.res_y = _res_y
		self.map_array = np.zeros(shape=((int)(self.size_x/self.res_x), (int)(self.size_y/self.res_y)))
		for i in range(len(self.map_array)):
			for j in range(len(self.map_array[0])):
				self.map_array[i][j] = 0.5

	def updateMap (self, points, pose):
		points=filterPoints(points)
		for p in points:
			map_x, map_y = discretizePoint(p, self.size_x, self.size_y, self.res_x, self.res_y)
			poseDiscrete = discretizePoint(pose, self.size_x, self.size_y, self.res_x, self.res_y)
			if 0 <= map_x < (self.size_x // self.res_x) and 0 <= map_y < (self.size_y // self.res_y):		
				self.map_array[map_y][map_x] += 0.15 #adding 0.1 for each obstacle point
				#freePoints = findDiscretePathToPoint(pose, [map_y, map_x], self.res_x, self.res_y)
				freePoints = bresenham(poseDiscrete, [map_x, map_y])								
				for fp in freePoints:						
					self.map_array[int(fp[1])][int(fp[0])] -= 0.05
		return self.map_array	

	def display (self):
		plt.imshow(self.map_array, interpolation="nearest", cmap='Blues', extent=[0, self.size_x, 0, self.size_y])
		plt.colorbar()
		plt.xlim(0, self.size_x)  # Set x-axis limit based on map size
		plt.ylim(0, self.size_y)  # Set y-axis limit based on map size
		#plt.show()

	def treshold (self):
		for i,m in enumerate(self.map_array):
			for j,mm in enumerate(m):
				if mm>0.5:
					self.map_array[i][j]=1
				elif mm==0.5:
					pass
				elif mm<0.5:
					self.map_array[i][j]=0
				#0.5 stays as it is!

	def publish_map (self):
		# Initialize ROS node
		rospy.init_node('map_publisher', anonymous=True)

		# Create a publisher for the map topic
		map_pub = rospy.Publisher('map_topic', OccupancyGrid, queue_size=10)
		
		# Create an OccupancyGrid message
		map_msg = OccupancyGrid()
		map_msg.header.frame_id = 'map'  # Frame ID for the map
		map_msg.info.width = len(self.map_array[0])  # Width of the map in cells
		map_msg.info.height = len(self.map_array)  # Height of the map in cells
		map_msg.info.resolution = 1.0  # Resolution of each cell in meters

		# Flatten the 2D map into a 1D array (row-major order)
		flattened_map = [cell for row in self.map_array for cell in row]
		for i in range(len(flattened_map)):
			if flattened_map[i] == 1:
				flattened_map[i] = 100
			elif flattened_map[i] != 0:
				flattened_map[i] = 50	
		map_msg.data = flattened_map
		
		# Publish the map
		rate = rospy.Rate(10)  # Publish at 10 Hz
		while not rospy.is_shutdown():
			map_msg.header.stamp = rospy.Time.now()  # Update the timestamp
			map_pub.publish(map_msg)
			rate.sleep()

		
class robotPos:
	#as it's a 2d scenario, only x, y positions and z orientation are relevant.	
	#position
	x = 0
	y = 0
	#orientation

	th = 0
	def __init__(self, _x, _y, _z):
		self.x=_x
		self.y=_y
		self.th=_z/180*np.pi
	#def __init__(self, msg):
	#	self.x=msg.pose.pose.position.x		
	#	self.y=msg.pose.pose.position.y
	#	self.th=msg.pose.pose.orientation.z#/180*np.pi


def discretizePoint(_point, _size_x, _size_y, _res_x, _res_y):
	map_x = int( (_point[0] - (-0.5 * _size_x)) / _res_x )
	map_y = int( (_point[1] - (-0.5 * _size_y)) / _res_y )
	return map_x, map_y


def findLine(_p1, _p2):
	a = (_p2[1]-_p1[1]) / (_p2[0]-_p1[0])
	b = (_p1[1]-_p1[0]) * a
	return a, b


#this funciton creates a sort of subspace from map.
def pixelsBetweenTwoPoints(_p1, _p2):
	result = []
	for i in range(int(np.absolute(_p2[1]-_p1[1]))+1):
		for j in range(int(np.absolute(_p2[0]-_p1[0]))+1):
			result.append([j,i])
	return result			


def lineCuttingPixel(_p, _a, _b, _xres, _yres):
	result = False
	xmin = float(_p[0])-float(_xres)/2
	xmax = float(_p[0])+float(_xres)/2
	ymin = float(_p[1])-float(_yres)/2
	ymax = float(_p[1])+float(_yres)/2
	# Calculate the y-coordinate of the line at the pixel's x_min and x_max
	y_line_min = _a * xmin + _b
	y_line_max = _a * xmax + _b
	x_line_min = (_b-ymin) / _a
	x_line_max = (_b-ymax) / _a

	if (ymin <= y_line_min <= ymax) or (ymin <= y_line_max <= ymax) or (xmin <= x_line_min <= xmax) or (xmin <= x_line_max <= xmax):
		result = True
	print (_p,result)
	print (y_line_min, y_line_max, x_line_min, x_line_max)
	print (ymin, ymax, xmin, xmax)

	return result


def findDiscretePathToPoint(_pose, _point, _xres, _yres):
	path = []
	pointsToCheck = pixelsBetweenTwoPoints(_pose, _point)
	a,b = findLine(_pose, _point)
	print(pointsToCheck)
	for p in pointsToCheck:
		if lineCuttingPixel(p, a, b, _xres, _yres):
			path.append(p)
	return path


def buildPoseArray(bag):
	pose_list = []	
	for topic, msg, t in bag.read_messages(topics=['/PIONIER6/RosAria/pose']):
		pose_list.append(msg)
	return pose_list


def buildLaserscanArray(bag):
	laserscan_list = []	
	for topic, msg, t in bag.read_messages(topics=['/PIONIER6/scan']):
		laserscan_list.append(msg.ranges)
	return laserscan_list


def localToGlobalCoord(_robot_pos, _local_distance, _local_angle):
	x = _robot_pos.x + _local_distance * np.cos(_robot_pos.th + _local_angle)
	y = _robot_pos.y + _local_distance * np.sin(_robot_pos.th + _local_angle)
	return x,y


def scanToPoints(_scan, _robot_pos, _range_min, _range_max):
	obstacles = []
	x = np.arange(0,512)
	step_size = (_range_max-_range_min)/512
	scan_angles = (step_size)*x + _range_min
	for i in x:
		if np.isnan(_scan[i]) or np.isinf(_scan[i]):
			continue
		point = localToGlobalCoord(_robot_pos, _scan[i], scan_angles[i])
		#result[i] = point
		obstacles.append(point)
	return obstacles


def filterPoints(_points):
	result = []
	for p in _points:
		if np.isinf(p[0]) or np.isinf(p[1]) or np.isnan(p[0]) or np.isinf(p[1]):
			pass		
		elif p[0] == 0 and p[1] == 0 and p[2] == 0: 
			pass
		else:
			result.append(p)
	result.pop(0)	
	return result


def bresenham(_p1, _p2):
	result = []
	x1, y1 = _p1
	x2, y2 = _p2
	dx = abs(x2 - x1)
	dy = abs(y2 - y1)
	sx = 1 if x1 < x2 else -1
	sy = 1 if y1 < y2 else -1
	err = dx - dy

	while x1 != x2 or y1 != y2:
		result.append([x1, y1])
		e2 = 2 * err
		if e2 > -dy:
			err -= dy
			x1 += sx
		if e2 < dx:
			err += dx
			y1 += sy

	result.append([x2, y2])  # Include the end point
	return result


def main():
	json_file = open("map_round.json")
	recordedArray = json.load(json_file)
	size =20
	resolution = 0.01
	map = map_t(size, size, resolution, resolution)

	for i in range(len(recordedArray)):
		pose = recordedArray[i]['pose']
		laser = recordedArray[i]['scan']
		rp = robotPos(pose[0], pose[1], pose[2])
		scanPoints = scanToPoints(laser,rp,-np.pi/2,np.pi/2)
		map.updateMap(scanPoints, [pose[1],pose[0]])	
	#bag = rosbag.Bag('2021-12-21-10-57-39.bag')
	#poses = buildPoseArray(bag)
	#laser = buildLaserscanArray(bag)
	#bag.close()
	#for i in range(len(poses)-2):
	#	p=poses[i]
	#	rp = robotPos(p)
	#	scan = laser[i]
		#scanPoints = scanToPoints(scan,rp,-np.pi/2,np.pi/2)
		#map.updateMap(scanPoints)
		#print(i)
	map.treshold()
	map.display()
	plt.plot()
	x = [0]#[robotPos(p).x+size/2 for p in poses]
	y = [0]#[robotPos(p).y+size/2 for p in poses]
	plt.scatter(x, y, s=0.3, color='red')
	print("publishing!")
	map.publish_map()
	plt.show()
	


if __name__ == "__main__":
	main()
	#points = (findDiscretePathToPoint([1,0], [0,8], 1, 1))
	#print("lalal", points)	
	#print(bresenham([1,0], [0,8]))
