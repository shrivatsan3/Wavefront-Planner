# -*- coding: utf-8 -*-
"""
ASEN 5519
ALGORITHMIC MOTION PLANNING
WAVE FRONT PLANNER

Program to find a path to goal from start using the wavefront planner

input: start, goal, obstacles
output: visual display of path

@author: shrivatsan
"""

import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon
from shapely.geometry import Point
from shapely.geometry import LineString
from matplotlib.colors import ListedColormap
from shapely.geometry import box
from shapely.ops import nearest_points


class Obstacle: #defining all obstacles as objects of this class
        
    def __init__(self, n ):
       
        self.no_of_vertices = n   # user specified number of vertices for an obstacle
        self.vertices = []     # Create a list to store all vertices
       
    
    def create_obstacle(self):   # method to create the obstacle from its vertices
        
        print("\n\nenter coordinates of vertices in the following format : if vertex is (2,3)")
        print("then enter 2,3 i.e type 2 followed by a comma followed 3")
        for i in range(self.no_of_vertices):
            self.vertices.append(tuple(float(x) for x in input('enter {} vertex: '.format(i+1)).split(','))) #create a list which stores all vertices as tuples
        self.obstacle = Polygon(self.vertices) #create a Polygon object using shapely module
        
    def display_obstacle(self):
        
        obstacle_x, obstacle_y = self.obstacle.exterior.xy  #this method is defined in class Polygon
        plt.fill(obstacle_x, obstacle_y)
        
    def is_in_obstacle(self, line):
        return line.intersects(self.obstacle)   # binary predicate defined in shapely module

  
def make_pixel_grid(goal, start):
        "Model the workspace as a grid"
    x_centre = np.arange(-6.75,40,0.25)
    y_centre = np.arange(-6.75,40,0.25)
    
    x_grid,y_grid= np.meshgrid(x_centre,y_centre) #create a grid from centre values
    pixel_value = np.zeros(x_grid.shape, dtype = int)
    visited_pixels = np.zeros(x_grid.shape, dtype = int)
    
   
    for i,x_ref in enumerate(x_centre):
        for j,y_ref in enumerate(y_centre):
            
            pixel = box(x_ref-0.125,y_ref-0.125,x_ref+ 0.125,y_ref+ 0.125) #Pixel is a box sides of length 0.25
            
            if pixel.contains(goal):    # define goal pixel value as 2
                pixel_value[j,i] = 2  
                goal_pixel_x = i
                goal_pixel_y = j
                continue
                
            if pixel.contains(start):   # Find the pixel indices for start point
                start_pixel_x = i
                start_pixel_y = j
                continue
                
            for o in obstacles: # define pixels in obstacle region as 1
                if(not pixel.disjoint(o.obstacle)):
                    pixel_value[j,i] = 1
                    break
                
    return goal_pixel_y,goal_pixel_x, start_pixel_y, start_pixel_x, pixel_value, visited_pixels, x_centre, y_centre


def findNeighbors(grid, x, y):
    
    neighbors = []
    
    if 0 < x < len(grid) - 1:
        xi = (0, -1, 1)   # this isn't first or last row, so we can look above and below
    elif x > 0:
        xi = (0, -1)      # this is the last row, so we can only look above
    else:
        xi = (0, 1)       # this is the first row, so we can only look below
    
    yi = (0, -1, 1) if 0 < y < len(grid[0]) - 1 else ((0, -1) if y > 0 else (0, 1))
    for a in xi:
        for b in yi:
            if a == b == 0:  
                continue
            if (a*b) != 0:  
                continue
            neighbors.append((x + a, y + b))
            
            
def wavefront_plan(pixel_value, visited_pixels):
        "populate the grid according to the wavefront planner using the breadth-first search algorithm"

    visited_pixels[goal_pixel_y,goal_pixel_x] = 1   # Start at goal.
    queue = []
    queue.append((goal_pixel_x,goal_pixel_y)) # add goal pixel to the queue
    
    while queue:
        
        (queue_x, queue_y) = queue[0]
        if queue_x == start_pixel_x and queue_y == start_pixel_y :  #start found, no need to populate the remaining grid
            break
        
        neighbors = findNeighbors(pixel_value, queue[0])     #Find the nieghbors for the current pixel
        
        queue.pop(0) 
        
        
        for (x_n, y_n) in neighbors: 
                if pixel_value[y_n, x_n] == 1:
                    continue
                if visited_pixels[y_n, x_n] == 0:    # Assign a value 1 greater than its neighbor 
                    queue.append((x_n, y_n))
                    pixel_value[y_n, x_n] = pixel_value[queue_y, queue_x] + 1
                    visited_pixels[y_n,x_n] = 1
    
    return pixel_value                   



No_of_obstacles = int(input("\nenter number of obstacles: "))
obstacles = []
for i in range(No_of_obstacles):
    n = int(input('\nenter number of vertices in {} obstacle: '.format(i+1)))
    obstacle_element = Obstacle(n)
    obstacle_element.create_obstacle()
    obstacles.append(obstacle_element)


goal = Point(35, 0) 
start = Point(0, 0)

goal_pixel_y,goal_pixel_x, start_pixel_y, start_pixel_x, pixel_value, visited_pixels,x_centre, y_centre = make_pixel_grid(goal, start)
pixel_value = wavefront_plan(pixel_value, visited_pixels)


robot_pos_x = start_pixel_x
robot_pos_y = start_pixel_y

robot_pos = []
robot_pos.append(( start.x,start.y))
def path_finder(robot_pos_y, robot_pos_x, pixel_value, x_centre, y_centre):
                "Find the path from start to goal"

            while(not (pixel_value[robot_pos_y, robot_pos_x] == 2)): # Stop when you reach the goal pixel
                
                neighbors = findNeighbors(pixel_value, (robot_pos_x, robot_pos_y))
                neighbor_pixel_value = pixel_value[robot_pos_y, robot_pos_x]
                
                for n in neighbors:
                    (n_x, n_y) = n
                    
                    if pixel_value[n_y, n_x] == 1:  # pixel in obstacle. Dont add it to path
                        continue
                    elif pixel_value[n_y, n_x] == 0:    #pixel in free space which hasn't been sampled. Dont add it to path
                        continue
                    elif neighbor_pixel_value > pixel_value[n_y, n_x]:#Find the shortest path by adding the neighbor with the shortest pixel value
                        neighbor_pixel_value = pixel_value[n_y, n_x]
                        robot_pos_x, robot_pos_y = (n_x, n_y)
                        robot_pos.append(( x_centre[robot_pos_x],y_centre[robot_pos_y]))
            return robot_pos_x, robot_pos_y, robot_pos

robot_pos_x, robot_pos_y, robot_pos =   path_finder(robot_pos_y, robot_pos_x, pixel_value, x_centre, y_centre)
robot_pos.pop() 
robot_pos.append(( goal.x,goal.y))

path_length = 0.25*len(robot_pos)
print("\nLength of path traversed by robot is : ", path_length)
robot_pos_x = []
robot_pos_y = []
for point in robot_pos:
    (robot_x, robot_y) = point
    robot_pos_x.append(robot_x)
    robot_pos_y.append(robot_y)

plt.figure(figsize =(10,10))
for o in obstacles:
    o.display_obstacle() 
plt.plot(start.x, start.y, "go")
plt.plot(goal.x, goal.y,"ro")
plt.plot(robot_pos_x, robot_pos_y)