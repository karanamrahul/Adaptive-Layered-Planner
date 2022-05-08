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
    
    obstacles[-1] += np.array([0.015,0.005]) * constants.velocity_factor
    # obstacles[-2] += np.array([-0.005,0.005]) * constants.velocity_factor/2
    # obstacles[-3] += np.array([-0.01,0.008]) * constants.velocity_factor/2
    # obstacles[-4] += np.array([0.015,0.005]) * constants.velocity_factor/3
    
    return obstacles
    
    
    
class Constants:
    def __init__(self):
        self.animation = True # True for animation
        self.visualize = 1 # show robots movement
        self.postprocessing = 1 # process and visualize the simulated experiment data after the simulation
        # self.savedata = 0 # save postprocessing metrics to the XLS-file
        self.maxiters = 500 # max number of samples to build the RRT
        self.goal_prob = 0.05 # with probability goal_prob, sample the goal
        self.threshold = 0.25 # [m], min distance os samples from goal to add goal node to the RRT
        self.step_size = 0.8 # [m], step_size parameter: this controls how far the RRT extends in each step.
        self.world_bounds_x = [-2.5, 2.5] # [m], map size in X-direction
        self.world_bounds_y = [-2.5, 2.5] # [m], map size in Y-direction
        self.velocity_factor = 4.0 # [m/s]
        self.ViconRate = 100 # [Hz]
        self.influence_radius = 0.15 # [m] potential fields radius, defining repulsive area size near the obstacle
        self.goal_tolerance = 0.05 # [m], maximum distance threshold to reach the goal
        self.num_robots = 4 # number of robots in the formation
        self.interrobots_dist = 0.3 # [m], distance between robots in default formation
        self.max_sp_dist = 0.2 * self.velocity_factor# * np.sqrt(self.num_robots) # [m], maximum distance between current robot's pose and the start from global planner

class Robot:
    def __init__(self, id):
        self.id = id
        self.start= np.array([0.0, 0.0]) # [m]
        self.start_global = np.array([0.0, 0.0]) # [m]
        self.path = np.array([self.start]) # [m]
        self.velocity = [] # [m/s]
        self.attractive_potential = [] # [m]
        self.repulsive_potential = [] # [m]
        self.APF_potential = [] # [m]
        self.leader = False # True if the robot is the leader

    def local_planner(self, obstacles, Constants):
        """
       @brief: This function implements the local planner.
       
       Arguments:
            obstacles {list} -- list of obstacles
            Constants {class} -- class containing the constants
            
         Returns:
            It returns the new pose of the robot along with the new velocity.

        """
        obstacles_grid = grid_map(obstacles)
        self.APF_potential, self.attractive_potential, self.repulsive_potential = combined_potential(obstacles_grid, self.start_global, Constants.influence_radius)
        [gy, gx] = np.gradient(-self.APF_potential)
        iy, ix = np.array( m2grid(self.start), dtype=int )
        w = 20 
        ax = np.mean(gx[ix-int(w/2) : ix+int(w/2), iy-int(w/2) : iy+int(w/2)])
        ay = np.mean(gy[ix-int(w/2) : ix+int(w/2), iy-int(w/2) : iy+int(w/2)])
        self.V = Constants.velocity_factor * np.array([ax, ay])
        self.velocity.append(norm(self.V))
        dt = 0.01 * Constants.velocity_factor / norm([ax, ay]) if norm([ax, ay])!=0 else 0.01
        self.start += dt*np.array( [ax, ay] ) 
        self.path = np.vstack( [self.path, self.start] )

def visualize2D():
    draw_map(obstacles)
    draw_gradient(robots[1].APF_potential) if Constants.num_robots>1 else draw_gradient(robots[0].APF_potential)
    for robot in robots: plt.plot(robot.start[0], robot.start[1], '^', color='blue', markersize=10, zorder=15) # robots poses
    robots_poses = []
    for robot in robots: robots_poses.append(robot.start)
    robots_poses.sort(key=lambda p: atan2(p[1]-centroid[1],p[0]-centroid[0]))
    plt.gca().add_patch( Polygon(robots_poses, color='yellow') )
    plt.plot(centroid[0], centroid[1], '*', color='b', markersize=10, label='Centroid position')
    plt.plot(robot1.route[:,0], robot1.route[:,1], linewidth=2, color='green', label="Leader's path", zorder=10)
    plt.plot(P[:,0], P[:,1], linewidth=3, color='orange', label='Global planner path')
    plt.plot(traj_global[sp_ind,0], traj_global[sp_ind,1], 'ro', color='blue', markersize=7, label='Global planner setpoint')
    plt.plot(xy_start[0],xy_start[1],'bo',color='red', markersize=20, label='start')
    plt.plot(xy_goal[0], xy_goal[1],'bo',color='green', markersize=20, label='goal')
    plt.legend()

# Initialization
init_fonts(small=12, medium=16, big=26)
Constants = Constants()
xy_start = np.array([1.2, 1.0])
# xy_goal =  np.array([1.5, -1.4])
xy_goal =  np.array([1.3, -1.0])

# Obstacles map construction
# obstacles = [
#               # bugtrap
#               np.array([[0.5, 0], [2.5, 0.], [2.5, 0.3], [0.5, 0.3]]),
#               np.array([[0.5, 0.3], [0.8, 0.3], [0.8, 1.5], [0.5, 1.5]]),
#               # np.array([[0.5, 1.5], [1.5, 1.5], [1.5, 1.8], [0.5, 1.8]]),
#               # angle
#               np.array([[-2, -2], [-0.5, -2], [-0.5, -1.8], [-2, -1.8]]),
#               np.array([[-0.7, -1.8], [-0.5, -1.8], [-0.5, -0.8], [-0.7, -0.8]]),
#               # walls
#               np.array([[-2.5, -2.5], [2.5, -2.5], [2.5, -2.47], [-2.5, -2.47]]), # comment this for better 3D visualization
#               np.array([[-2.5, 2.47], [2.5, 2.47], [2.5, 2.5], [-2.5, 2.5]]),
#               np.array([[-2.5, -2.47], [-2.47, -2.47], [-2.47, 2.47], [-2.5, 2.47]]),
#               np.array([[2.47, -2.47], [2.5, -2.47], [2.5, 2.47], [2.47, 2.47]]), # comment this for better 3D visualization

#               # moving obstacle
#               np.array([[-2.3, 2.0], [-2.2, 2.0], [-2.2, 2.1], [-2.3, 2.1]]),
#               np.array([[2.3, -2.3], [2.4, -2.3], [2.4, -2.2], [2.3, -2.2]]),
#               np.array([[0.0, -2.3], [0.1, -2.3], [0.1, -2.2], [0.0, -2.2]]),
#             ]
"""" Narrow passage """
passage_width = 0.3
passage_location = 0.0
obstacles = [
            # narrow passage
              np.array([[-2.5, -0.5], [-passage_location-passage_width/2., -0.5], [-passage_location-passage_width/2., 0.5], [-2.5, 0.5]]),
              np.array([[-passage_location+passage_width/2., -0.5], [2.5, -0.5], [2.5, 0.5], [-passage_location+passage_width/2., 0.5]]),
            ]
# obstacles = []

robots = []
for i in range(Constants.num_robots):
    robots.append(Robot(i+1))
robot1 = robots[0]; robot1.leader=True


# Metrics to measure (for postprocessing)
class Metrics:
    def __init__(self):
        self.mean_dists_array = []
        self.max_dists_array = []
        self.centroid_path = [np.array([0,0])]
        self.centroid_path_length = 0
        self.robots = []
        self.vels_mean = []
        self.vels_max = []
        self.area_array = []
        self.cpu_usage_array = [] # [%]
        self.memory_usage_array = [] # [MiB]

        self.folder_to_save = 'results/'

metrics = Metrics()

# Layered Motion Planning: RRT (global) + Potential Field (local)
if __name__ == '__main__':
    fig2D = plt.figure(figsize=(10,10))
    draw_map(obstacles)
    plt.plot(xy_start[0],xy_start[1],'bo',color='red', markersize=20, label='start')
    plt.plot(xy_goal[0], xy_goal[1],'bo',color='green', markersize=20, label='goal')

    P_long = rrtpath( xy_start, xy_goal,obstacles, Constants)
    print('Path Shortenning...')
    P = ShortenPath(P_long, obstacles, smoothiters=50) # P = [[xN, yN], ..., [x1, y1], [x0, y0]]

    traj_global = waypts2setpts(P, Constants)
    P = np.vstack([P, xy_start])
    plt.plot(P[:,0], P[:,1], linewidth=3, color='orange', label='Global planner path')
    plt.pause(0.5)

    sp_ind = 0
    robot1.route = np.array([traj_global[0,:]])
    robot1.start = robot1.route[-1,:]

    followers_sp = formation(Constants.num_robots, leader_des=robot1.start, v=np.array([0,-1]), l=Constants.interrobots_dist)
    for i in range(len(followers_sp)):
        robots[i+1].start = followers_sp[i]
        robots[i+1].route = np.array([followers_sp[i]])
    print('Start movement...')
    t0 = time.time(); t_array = []

    while True: # loop through all the setpoint from global planner trajectory, traj_global
        t_array.append( time.time() - t0 )
        # print("Current time [sec]: ", time.time() - t0)
        dist_to_goal = norm(robot1.start - xy_goal)
        if dist_to_goal < Constants.goal_tolerance: # [m]
            print('Goal is reached')
            break
        if len(obstacles)>2: obstacles = dynamic_obstacles(obstacles, Constants) # change poses of some obstacles on the map

        # leader's setpoint from global planner
        robot1.start_global = traj_global[sp_ind,:]
        # correct leader's pose with local planner
        robot1.local_planner(obstacles, Constants)

        """ adding following robots in the swarm """
        # formation poses from global planner
        followers_sp_global = formation(Constants.num_robots, robot1.start_global, v=normalize(robot1.start_global-robot1.start), l=Constants.interrobots_dist)
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
        metrics.centroid_path = np.vstack([metrics.centroid_path, centroid])
        # dists to robots from the centroid:
        dists = []
        for robot in robots:
            dists.append( norm(centroid-robot.start) )
        # Formation size estimation
        metrics.mean_dists_array.append(np.mean(dists)) # Formation mean Radius
        metrics.max_dists_array.append(np.max(dists)) # Formation max Radius

        # Algorithm performance (CPU and memory usage)
        # metrics.cpu_usage_array.append( cpu_usage() )
        # metrics.memory_usage_array.append( memory_usage() )
        # print("CPU: ", cpu_usage())
        # print("Memory: ", memory_usage())

        # visualization
        if Constants.visualize:
            plt.cla()
            visualize2D()        

            plt.draw()
            plt.pause(0.01)

        # update loop variable
        if sp_ind < traj_global.shape[0]-1 and norm(robot1.start_global - centroid) < Constants.max_sp_dist: sp_ind += 1




   
# close windows if Enter-button is pressed
plt.draw()
plt.pause(0.1)
raw_input('Hit Enter to close')
plt.close('all')