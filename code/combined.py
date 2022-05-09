
########################################################################################################################
#                      Layered Planner (RRT + APF)                                                                     #
########################################################################################################################



"""
@brief: This file contains the code for the combined model.(RRT + APF)

Global Planner - Rapidly-exploring Random Tree (RRT)
Local Planner  - Artificial Potential Field (APF)

@file  : combined.py

Two test cases are included in this file:
 - Test 1: Narrow Passage 
 - Test 2: Wide Passage along with formation of robots

"""



# Importing the libraries

from ast import Constant
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import norm
import time
from math import *
import random
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.patches import Polygon
from APF import *
from rrt import *
from utils import *


def drawMap(obstacles):
    """
    @brief: This function draws the map with the obstacles.
    
    Arguments:
        obstacles {list} -- list of obstacles
        
    Returns:
        None
    
    """
    cols = [-2.5,2.5]
    rows = [-2.5,2.5]
    
    ax = plt.gca() # get current axes
    ax.set_xlim(cols)
    ax.set_ylim(rows)
    
    for obs in range(len(obstacles)):
        ax.add_patch(Polygon(obstacles[obs], facecolor='red',zorder = 1))
        
        
def dynamic_obstacles(obstacles,constants):
    
    """
    @breif: This function creates the dynamic obstacles.
    
    Args: 
        obstacles {list} -- list of obstacles
        constants {list} -- list of constants
        
    Returns:
        None
    """
    
    obstacles[-3] += np.array([0.02, 0.0]) * Constants.velocity_factor # obstacle 1 movement
    obstacles[-2] += np.array([-0.005, 0.005]) * Constants.velocity_factor
    obstacles[-1] += np.array([0.0, 0.01]) * Constants.velocity_factor
    obstacles[0] += np.array([0.603, -0.64]) * Constants.velocity_factor
    return obstacles
    
    
    
class Constants:
    def __init__(self):
        self.animation = True # True for animation
        self.visualize = 1 # show robots movement
        self.postprocessing = 1 # process and visualize the simulated experiment data after the simulation
        self.maxiters = 5000 # max number of samples to build the RRT
        self.goal_prob = 0.05 # with probability goal_prob, sample the goal
        self.threshold = 0.25 # [m], min distance os samples from goal to add goal node to the RRT
        self.step_size = 0.4 # [m], step_size parameter: this controls how far the RRT extends in each step.
        self.world_bounds_x = [-2.5, 2.5] # [m], map size in X-direction
        self.world_bounds_y = [-2.5, 2.5] # [m], map size in Y-direction
        self.velocity_factor = 4.0 # [m/s]
        self.ViconRate = 100 # [Hz]
        self.influence_radius = 1.22 # [m] potential fields radius, defining repulsive area size near the obstacle
        self.goal_tolerance = 0.05 # [m], maximum distance threshold to reach the goal
        self.num_robots = 6 # number of robots in the formation
        self.max_sp_dist = 0.3 * self.velocity_factor# * np.sqrt(self.num_robots) # [m], maximum distance between current robot's pose and the start from global planner
        self.moving_obstacles = 0 # 1 for moving obstacles, 0 for static obstacles
class Robot:
    def __init__(self):
        self.id = id
        self.start = [0.0, 0.0]
        self.start_global = [0.0, 0.0]
        self.path = []
        self.APF_potential = []
        self.leader = False # True if the robot is the leader
        self.velocity_array = []

    def local_planner(self, obstacles, Constants):
        """
       @brief: This function implements the local planner.
       
       Arguments:
            obstacles {list} -- list of obstacles
            Constants {class} -- class containing the constants
            
         Returns:
            It returns the new pose of the robot along with the new velocity.

        """
        # Initialize the variables
        obs_grid = grid_map(obstacles)
        self.APF_potential= combined_potential(obs_grid,self.start_global,Constants.influence_radius)
        self.start,self.velocity = gradient_planner_next(self.start,self.APF_potential,Constants)
        self.velocity_array.append(np.linalg.norm(self.velocity))
        self.path = np.vstack((self.path, self.start))

def visualize2D():
        plt.plot(centroid[0], centroid[1], '*', color='blue', markersize=7)
        draw_map(obstacles)
        if Constants.num_robots == 1:
            draw_gradient(robots[0].APF_potential)
        else:
            draw_gradient(robots[1].APF_potential)
        for robot in robots[1:]: plt.plot(robot.start[0], robot.start[1], '^', color='blue', markersize=10, zorder=15) # robots poses
        plt.plot(robot1.start[0], robot1.start[1], '^', color='green', markersize=10, zorder=15) # robots poses
        plt.plot(robot1.path[:,0], robot1.path[:,1], linewidth=2, color='green', label="Robot's path, corrected with local planner", zorder=10)
        # for robot in robots[1:]: plt.plot(robot.path[:,0], robot.path[:,1], '--', linewidth=2, color='green', zorder=10)
        plt.plot(P[:,0], P[:,1], linewidth=3, color='orange', label='Global planner path')
        for robot in robots[:1]: plt.plot(robot.start_global[0], robot.start_global[1], 'ro', color='green', markersize=7, label='Global planner setpoint')
        plt.plot(xy_start[0],xy_start[1],'bo',color='red', markersize=20, label='start')
        plt.plot(xy_goal[0], xy_goal[1],'bo',color='green', markersize=20, label='goal')
        plt.legend()

# Initialization

Constants = Constants()
xy_start = np.array([1.4, 0.9])
xy_goal =  np.array([1.5, -1.5])

# Uncomment the first obstacles array for visualizing the first configuration space and the second for visualizing the second configuration space
# obstacles = [
#               # bugtrap
#               np.array([[0.5, 0], [2.5, 0.], [2.5, 0.3], [0.5, 0.3]]),
#               np.array([[0.5, 0.3], [0.8, 0.3], [0.8, 1.5], [0.5, 1.5]]),
#               np.array([[0.5, 1.5], [1.5, 1.5], [1.5, 1.8], [0.5, 1.8]]), # Failed case
#               # angle
#               np.array([[-2, -2], [-0.5, -2], [-0.5, -1.8], [-2, -1.8]]),
#               np.array([[-0.7, -1.8], [-0.5, -1.8], [-0.5, -0.8], [-0.7, -0.8]]),
#               # walls
#               np.array([[-2.5, -2.5], [2.5, -2.5], [2.5, -2.49], [-2.5, -2.49]]),
#               np.array([[-2.5, 2.49], [2.5, 2.49], [2.5, 2.5], [-2.5, 2.5]]),
#               np.array([[-2.5, -2.49], [-2.49, -2.49], [-2.49, 2.49], [-2.5, 2.49]]),
#               np.array([[2.49, -2.49], [2.5, -2.49], [2.5, 2.49], [2.49, 2.49]]),

#               np.array([[-1.0, 2.0], [0.5, 2.0], [0.5, 2.5], [-1.0, 2.5]]), # my table
#               np.array([[-1.0, 2.0], [0.5, 2.0], [0.5, 2.5], [-1.0, 2.5]]) + np.array([2.0, 0]), # Evgeny's table
#               np.array([[-2.0, -0.5], [-2.0, 1.0], [-2.5, 1.0], [-2.5, -0.5]]), # Roman's table
#               np.array([[-1.2, -1.2], [-1.2, -2.5], [-2.5, -2.5], [-2.5, -1.2]]), # mats
#               np.array([[2.0, 0.8], [2.0, -0.8], [2.5, -0.8], [2.5, 0.8]]), # Mocap table


#               # moving obstacle
#               np.array([[-2.3, 2.0], [-2.2, 2.0], [-2.2, 2.1], [-2.3, 2.1]]),
#               np.array([[2.3, -2.3], [2.4, -2.3], [2.4, -2.2], [2.3, -2.2]]),
#               np.array([[0.0, -2.3], [0.1, -2.3], [0.1, -2.2], [0.0, -2.2]]),
#               np.array([[-2.3, -0.5], [-0.25, -0.5], [-0.125 ,0.5], [-2.3, 0.1]]), # Failed case 
#             ]

passage_width = 0.55
passage_location = 0.0
obstacles = [
            # narrow passage
              np.array([[-2.5, -0.5], [-passage_location-passage_width/2., -0.5], [-passage_location-passage_width/2., 0.5], [-2.5, 0.5]]),
              np.array([[-passage_location+passage_width/2., -0.5], [2.5, -0.5], [2.5, 0.5], [-passage_location+passage_width/2., 0.5]]),
                # np.array([[-2.0, -0.5], [-2.0, 1.0], [-2.5, 1.0], [-2.5, -0.5]]), 
              np.array([[-1.2, -1.2], [-1.2, -2.5], [-2.5, -2.5], [-2.5, -1.2]]), 
            #    np.array([[2.0, 0.8], [2.0, -0.8], [2.5, -0.8], [2.5, 0.8]]),
                             np.array([[-2.5, -2.5], [2.5, -2.5], [2.5, -2.49], [-2.5, -2.49]]),
              np.array([[-2.5, 2.49], [2.5, 2.49], [2.5, 2.5], [-2.5, 2.5]]),
              np.array([[-2.5, -2.49], [-2.49, -2.49], [-2.49, 2.49], [-2.5, 2.49]]),
               np.array([[0.5, 0], [2.5, 0.], [2.5, 0.3], [0.5, 0.3]]),
               np.array([[0.5, 0.3], [0.8, 0.3], [0.8, 1.5], [0.5, 1.5]]),
            #    np.array([[-1.0, 2.0], [0.5, 2.0], [0.5, 2.5], [-1.0, 2.5]]),
               np.array([[-2.3, -0.5], [-0.25, -0.5], [-0.125 ,0.5], [-2.3, 0.1]]),
               np.array([[-1.0, 2.0], [0.5, 2.0], [0.5, 2.5], [-1.0, 2.5]]) + np.array([2.0, 0]),
            #    np.array([[-2.3, -0.5], [-0.25, -0.5], [-0.125 ,0.5], [-2.3, 0.1]]),
            #    np.array([[0.5, 1.5], [1.5, 1.5], [1.5, 1.8], [0.5, 1.8]])
            ]


robots = []
for i in range(Constants.num_robots):
    robots.append(Robot())
robot1 = robots[0]; robot1.leader=True



# Layered Motion Planning: RRT (global) + Potential Field (local)
if __name__ == '__main__':
    fig2D = plt.figure(figsize=(10,10))
    draw_map(obstacles)
    plt.plot(xy_start[0],xy_start[1],'bo',color='red', markersize=20, label='start')
    plt.plot(xy_goal[0], xy_goal[1],'bo',color='green', markersize=20, label='goal')

    P_long = rrtpath( xy_start, xy_goal,obstacles, Constants)
    P = ShortenPath(P_long, obstacles, smoothiters=30) 

    traj_global = waypts2setpts(P, Constants)
    P = np.vstack([P, xy_start])
    plt.plot(P[:,0], P[:,1], linewidth=3, color='orange', label='Global planner path')
    plt.pause(0.5)

    sp_ind = 0
    robot1.path = np.array([traj_global[0,:]])
    robot1.start = robot1.path[-1,:]
    followers_sp = formation(Constants.num_robots, leader_des=robot1.start, v=np.array([0,-1]), l=0.3)
    for i in range(len(followers_sp)):
        robots[i+1].start = followers_sp[i]
        robots[i+1].path = np.array([followers_sp[i]])


    while True: # main loop for the formation control
  
        dist_to_goal = norm(robot1.start - xy_goal)
        if dist_to_goal < Constants.goal_tolerance: # goal reached 
            print('Goal is reached')
            break
        if Constants.moving_obstacles: obstacles = dynamic_obstacles(obstacles, Constants) # change poses of some obstacles on the map

        # leader's setpoint from global planner
        robot1.start_global = traj_global[sp_ind,:]
        # correct leader's pose with local planner
        robot1.local_planner(obstacles, Constants)

        """ adding following robots in the swarm """
        # formation poses from global planner
        followers_sp_global = formation(Constants.num_robots, robot1.start_global, v=normalize(robot1.start_global-robot1.start), l=0.3)
        for i in range(len(followers_sp_global)): robots[i+1].start_global = followers_sp_global[i]
        for p in range(len(followers_sp)): # formation poses correction with local planner
            # robots repel from each other inside the formation
            robots_obstacles_sp = [x for i,x in enumerate(followers_sp + [robot1.start]) if i!=p] # all poses except the robot[p]
            robots_obstacles = poses2polygons( robots_obstacles_sp ) # each drone is defined as a small cube for inter-robots collision avoidance
            obstacles1 = np.array(obstacles + robots_obstacles) # combine exisiting obstacles on the map with other robots[for each i: i!=p] in formation
            # follower robot's position correction with local planner
            robots[p+1].local_planner(obstacles1, Constants)
            followers_sp[p] = robots[p+1].start

        # centroid pose:
        centroid = 0
        for robot in robots: centroid += robot.start / len(robots)
        # dists to robots from the centroid:
        dists = []
        for robot in robots:
            dists.append( norm(centroid-robot.start) )
        # Formation size estimation


        # visualization
        if Constants.visualize:
            plt.cla()
            plt.plot(centroid[0], centroid[1], 'bo', color='red', markersize=20, label='centroid')
            visualize2D()        

            plt.draw()
            plt.pause(0.01)

        # update loop variable
        if sp_ind < traj_global.shape[0]-1 and norm(robot1.start_global - centroid) < Constants.max_sp_dist: sp_ind += 1




   
# close windows if Enter-button is pressed
plt.draw()
plt.pause(0.1)
input('Hit Enter to close')
plt.close('all')