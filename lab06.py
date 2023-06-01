import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

OBSTACLECONST = 9999
UNINITCONST = -1

#this is a function returning mockup map before wavefront init. 1 means obstacle, 0 means free space
def testMap1():
	map = [[0,0,0,0,0,0,0,0,0,0,0,0], \
		[0,1,0,1,1,1,1,1,0,0,0,0], \
		[0,0,1,0,0,0,0,0,0,1,1,1], \
		[0,1,0,0,1,1,0,0,0,0,1,0], \
		[0,0,0,1,1,1,0,0,0,0,1,0], \
		[0,1,0,1,1,1,1,1,1,0,1,0], \
		[0,1,0,0,1,0,0,1,1,0,1,0], \
		[0,0,0,1,1,1,0,0,1,0,1,1], \
		[0,1,0,0,0,0,0,0,1,0,0,0], \
		[0,1,0,1,1,1,1,0,1,0,1,1], \
		[0,0,0,1,0,0,0,0,1,0,1,0], \
		[0,1,1,1,1,1,0,0,1,0,0,0], \
		[0,1,0,0,0,0,0,0,1,0,0,0], \
		[0,1,0,0,0,0,0,0,1,0,0,0], \
		[0,1,0,1,0,1,1,0,1,1,1,0], \
		[0,1,0,1,0,1,0,1,0,0,0,0], \
		[0,1,0,1,0,0,0,0,0,1,0,0], \
		[0,1,1,0,0,1,0,1,0,0,1,1], \
		[0,0,0,1,0,0,0,1,0,1,0,0]]	
	return map


def wavefrontInit(_map, _goal):
	result = _map
	if result[_goal[0]][_goal[1]] == 0:
		#point is free, not an obstacle in asked goal pixel
		for i in range(len(result)):
			for j in range(len(result[0])):
				if result[i][j] == 0:
					result[i][j] = UNINITCONST #uninitialised point
				if result[i][j] == 1:
					result[i][j] = OBSTACLECONST #obstacle
		result[_goal[0]][_goal[1]]=0
	else:
		print("Unallowed goal, map cannot be initialized.")
		abort
	return result


#checks if any element didn't get chenged during previous iteration. THat would mean there are not accesible pockets in the map.
def mapNotChanged(_map_old, _map_new):
	result = True
	for i in range(len(_map_new)):
		for j in range(len(_map_new[0])):
			if _map_new[i][j] != _map_old[i][j]:
				result = False
	return result			


#returns True if not initialised cells still exist (algorithm is not over yet)
def unitialisedCellsExist(_map):
	result = False
	for i in range(len(_map)):
		for j in range(len(_map[0])):
			if _map[i][j] == UNINITCONST:
				result = True
	return result			

def findHighestNonObstacleValue(_map):
	biggestValue = 0
	for i in range(len(_map)):
		for j in range(len(_map[0])):
			if _map[i][j] > biggestValue and _map[i][j] != OBSTACLECONST:
				biggestValue = _map[i][j]
	return biggestValue

def getPointsOfValue(_map, _value):
	result = []
	for i in range(len(_map)):
		for j in range(len(_map[0])):
			if _map[i][j] == _value:
				result.append([i,j])
	return result

def removeDuplicates(points):
    seen = []
    result = []
    for p in points:
        if p not in seen:
            result.append(p)
            seen.append(p)
    return result

def getPointsNonInitialisedNeighbours(_map, _points):
	result = []
	for p in _points:
		#select possible point neighbours
		neighbours=[ [p[0]+1,p[1]], [p[0]-1,p[1]], [p[0],p[1]+1], [p[0],p[1]-1] ]
		for n in neighbours:
			#check if point is within the map boudaries
			if 0 <= n[0] < len(_map) and 0 <= n[1] < len(_map[0]):
				#point within a map!
				if _map[n[0]][n[1]] == -1:
					#that point is unintialised so add it to results
					result.append([n[0],n[1]])
	result = removeDuplicates(result) #this needs to be done, as some newly selected points may be neighbouring two of the previously selected points	
	return result

def updateMapPoints(_map, _points, _new_value):
	for p in _points:
		_map[p[0]][p[1]] = _new_value
	return _map

def wavefrontStep(_map):
	highestValue = findHighestNonObstacleValue(_map)
	activePoints = getPointsOfValue(_map, highestValue)
	newPoints = getPointsNonInitialisedNeighbours(_map, activePoints)
	return updateMapPoints(_map, newPoints, highestValue+1)
 
#copy map1 to map2	
def copyMap(_map1, _map2):
	return np.copy(_map1)
#	for row in _map1:
#		_map2.append(list(row))
#	return _map2
	
def wavefrontPlanner(_map, _goal):
	argMap = []
	newMap = []
	oldMap = wavefrontInit(_map, _goal)
	while unitialisedCellsExist:
		argMap = copyMap(oldMap, argMap) #to avoid modification of  the oldMap for later comparison
		newMap = wavefrontStep(argMap) 
		#print(newMap)
		if mapNotChanged(oldMap, newMap):
			break
		oldMap = copyMap(newMap, oldMap)
	return newMap

def findNextStep(_map, _point):
	currentVal = _map[_point[0]][_point[1]]
	xs = _point[0] #just to simpify the notation
	ys = _point[1]
	neighbours=[ [xs+1,ys], [xs-1,ys], [xs,ys+1], [xs,ys-1] ]
	for n in neighbours:
		#check if point is within the map boudaries
		if 0 <= n[0] < len(_map) and 0 <= n[1] < len(_map[0]):
			#print(len(_map), len(_map[0]))
			#print(_map[n[0]][n[1]])
			if _map[n[0]][n[1]] < currentVal:
				return n		

def findShortestPath(_map, _start):
	currentVal = _map[_start[0]][_start[1]]
	if currentVal == OBSTACLECONST :
		print("Not a legal starting point.")	
	else:	
		path = []
		currentPoint = _start
		path.append(_start)
		while currentVal != 0:
			currentPoint=findNextStep(_map, currentPoint)
			path.append(currentPoint)
		#	print(nextStep)
			currentVal = _map[currentPoint[0]][currentPoint[1]]	
		#	print(currentVal)
		return path


def plot(_map, _path):
	fig, ax = plt.subplots()
	#cmap = plt.cm.get_cmap('cool')
	cmap = plt.cm.binary
	# Plot the floormap
	im = ax.imshow(_map, cmap=cmap, interpolation='nearest')
	# Plot the path as a line with a different color and linewidth
	path_x = [point[1] for point in _path]
	path_y = [point[0] for point in _path]
	ax.plot(path_x, path_y, color='red', linewidth=1.5)
	# Show the plot
	plt.show()


def main():
	# Convert floormap to a NumPy ar ray
	map = np.array(testMap1())			
	#map = np.array(testMap2())
	map = wavefrontPlanner(map, [0,0])
	print(map)
	#path=findShortestPath(map, [10, 4])
	path=findShortestPath(map,[16,2])	
	print(path)
	plot(map, path)


if __name__ == "__main__":
	main()

