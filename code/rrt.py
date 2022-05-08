# Implement a RRT* algorithm for a 2D environment


# Importing libraries
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
from random import random
from matplotlib.patches import Polygon
import math
import time
import sys
import os


class Node:
    def __init__(self):
        self.p = [0,0] # Position
        self.i = 0 # Index
        self.iPrev = 0 # Index of previous node
        
        
def isCollisionFreeVertex(p, obstacles):
    
    """
    @param p: Position
    @param obstacles: List of obstacles
    
    @return: True if p is collision free, False otherwise
    """
    colFree = True
    
    for obs in obstacles:
        hull = Path(obs)
        colFree =  not hull.contains_point(p)
        if hull.contains_point(p):
            return colFree
    return colFree
    
def isCollisionFreeEdge(p1, p2, obstacles):
    """
    @param p1: Position
    @param p2: Position
    @param obstacles: List of obstacles
    
    @return: True if p1 and p2 are collision free, False otherwise  
        
    """
    p1 = np.array(p1)
    p2 = np.array(p2)
    obstacles = np.array(obstacles)
    colFree = True 
    dist = np.linalg.norm(p1-p2) # Distance between p1 and p2
    map_res = 0.01 # Map resolution (m)
    m = int(dist/map_res) # Number of points to be checked
    c = np.linspace(0,1,m) # List of points to be checked
    for i in range(1,m-1):
        p=(1- c[i])*p1 + c[i]*p2 # Point to be checked
        colFree = isCollisionFreeVertex(p, obstacles)
        if not colFree:
            return colFree
    return colFree


def closest_Node(p, nodes):
    """
    @param p: Position
    @param nodes: List of nodes
    
    @return: Index of closest node to p
    
    
    """
    
    dist = []
    
    for node in nodes:
        dist.append(np.sqrt((node.p[0]-p[0])**2 + (node.p[1]-p[1])**2))
    dist = np.array(dist)
    
    dist_min = np.min(dist)
    dist_min_idx = np.argmin(dist)
    closestNode = nodes[dist_min_idx]
    
    return closestNode


def rrtpath(start, goal, obstacles,const):
    """
    @param start: Start position
    @param goal: Goal position
    @param obstacles: List of obstacles
    @param const: const class containing the parameters of the algorithm
    
    @return: Path from start to goal
    
    
    """
    path = [] # Path
    start_node = Node() # Start node
    start_node.p = start    # Start position
    start_node.i = 0 # Index
    start_node.iPrev = 0 # Index of previous node
    path.append(start_node)    # Append start node to path
    
    reachedGoal = False # Goal reached
    goal_threshold = const.threshold # Goal threshold
    step_size = const.step_size # Step size    
    
    
    start_time = time.time()
    iterations = 0
    
    print("Started RRT* algorithm...")
    
    while not reachedGoal:
        p_random = random() # Random position
        
        if p_random < const.goal_prob:
            p = goal
        else :
            
            p_random =  np.array([random()*2*const.world_bounds_x[1] -const.world_bounds_x[1],random()*2*const.world_bounds_y[1]-const.world_bounds_y[1]])

            p = p_random
        colFree = isCollisionFreeVertex(p, obstacles)
        if not colFree:
            iterations += 1
            continue
        closestNode = closest_Node(p, path) # Closest node to p
        
        newNode = Node() # New node
        newNode.p = closestNode.p + step_size*(p-closestNode.p)/np.linalg.norm(p-closestNode.p) # New position
        newNode.i = len(path) # Index
        newNode.iPrev = closestNode.i # Index of previous node
        
        
        colFree = isCollisionFreeEdge(newNode.p, closestNode.p, obstacles)
        if not colFree:
            iterations += 1
            continue
        
        if const.animation:
            plt.plot(newNode.p[0], newNode.p[1], 'ro') # Plot new node
            plt.plot([closestNode.p[0], newNode.p[0]], [closestNode.p[1], newNode.p[1]], color ='blue') # Plot edge between closest node and new node
            plt.draw()
            plt.pause(0.01)

        
        path.append(newNode) # Append new node to path
        
        if np.linalg.norm(newNode.p-goal) < goal_threshold:
           
            goalNode = Node() # Goal node
            goalNode.p = goal # Goal position
            goalNode.i  = len(path) # Index
            goalNode.iPrev = newNode.i # Index of previous node
            
            if isCollisionFreeEdge(goalNode.p, newNode.p, obstacles):
                path.append(goalNode)
                goal_pts = [goalNode.p]
            else:
                goal_pts = []
            reachedGoal = True  
            print("Goal reached in {} iterations".format(iterations))
            print("Goal reached!")
            print("Time taken: {} seconds".format(time.time()-start_time))
        iterations+=1
            
    # Plotting the path
    print("Plotting the path...")
    i = len(path)-1
    while True:
        i = path[i].iPrev
        goal_pts.append(path[i].p)
        if i == 0:
            break
    goal_pts = np.array(goal_pts)
    # plt.plot(goal_pts[:,0], goal_pts[:,1], 'g-', linewidth=2)
    
    # # Plotting the obstacles
    # print("Plotting the obstacles...")
    # for obs in obstacles:
    #     hull = Path(obs)
    #     x,y = hull.vertices.T
    #     plt.plot(x,y, 'k-', linewidth=2, markersize=5, label='Obstacles')
    # plt.legend()
    # plt.show()
    
    
    return goal_pts # Return path

# def shortest_path(path,obstacles):
#     """
#     @param start: Start position
#     @param goal: Goal position
#     @param obstacles: List of obstacles
#     @param const: const class containing the parameters of the algorithm
    
    
#     @return: Smooth path from start to goal
#     """
#     smooth_iters = 10 # Number of iterations of the smoothing algorithm
#     m = path.shape[0] # Number of nodes
#     l = np.zeros(m) # List of lengths
#     for i in range(1,m):
#         l[i] = np.linalg.norm(path[i,:] - path[i-1,:]) + l[i-1] # Length of path from node i to node i-1
        
#     iterations = 0
#     while iterations < smooth_iters:
#         s1 = random()*l[-1] # Random length
#         s2 = random()*l[-1] # Random length
#         if s2 < s1:
#             s1, s2 = s2, s1
#         for i in range(1,m):
#             if l[i] > s1:
#                 i = i-1
     
def ShortenPath(P, obstacles, smoothiters=10):
    # INPUTS
    #   P - path to get smoothed (after RRT algorithm)
    #   obstacles - says where the obstacles are
    #   smoothiters - maximum number of smoothing iterations
    #
    # OUTPUTS
    #   P_smoothed - a path, same format as before:  
    #    P_smoothed = [q1 q2 q3 ... qM]
    #               where q1=qstart and qM=qgoal; in other words, the sequence
    #               of straight-line paths from q1 to q2, q2 to q3, etc., takes
    #               the robot from start to goal without collision
    P = np.array(P)
    m = P.shape[0]
    l = np.zeros(m)
    for k in range(1, m):
        l[k] = np.linalg.norm(P[k,:]-P[k-1,:]) + l[k-1] # find all of the straight-line distances
    iters = 0
    while iters < smoothiters:
        s1 = random()*l[m-1] 
        s2 = random()*l[m-1]
        if s2 < s1:
            temps = s1
            s1 = s2
            s2 = temps
        for k in range(1, m):
            if s1 < l[k]:
                i = k - 1
                break
        for k in range(i, m):
            if s2 < l[k]:
                j = k - 1
                break
        if (j <= i):
            iters = iters + 1
            continue
        t1 = (s1 - l[i]) / (l[i+1]-l[i])
        gamma1 = (1 - t1)*P[i,:] + t1*P[i+1,:]
        t2 = (s2 - l[j]) / (l[j+1]-l[j])
        gamma2 = (1 - t2)*P[j,:] + t2*P[j+1,:]
        
        collisionFree = isCollisionFreeEdge(gamma1, gamma2, obstacles)
        if collisionFree == 0:
            iters = iters + 1
            continue

        P = np.vstack([P[:(i+1),:], gamma1, gamma2, P[(j+1):,:]])
        m = P.shape[0]
        l = np.zeros(m)
        for k in range(1, m):
            l[k] = np.linalg.norm( P[k,:] - P[k-1,:] ) + l[k-1]
        iters = iters + 1
    P_short = P 
    
    return P_short
        
class const:
    def __init__(self):
        self.goal_prob = 0.05 # Probability of goal
        self.threshold = 0.25 # Goal threshold
        self.step_size = 0.3 # Step size
        self.animation = True # Animation
        self.world_bounds_x =[-2.5,2.5] # World bounds x axis
        self.world_bounds_y = [-2.5,2.5] # World bounds (y-axis)
    
        
# class RRTstar:
#     def __init__(self,start,goal,obstacles,max_iterations,max_length,goal_radius):
#         self.start = start  # start node
#         self.goal = goal  # goal node
#         self.obstacles = obstacles  # obstacles
        
        



def draw_map(obstacles):
    # Obstacles. An obstacle is represented as a polygon of a number of points. 
    # First row is x, second is y (position of vertices)

    # Bounds on world
    world_bounds_x = [-2.5, 2.5]
    world_bounds_y = [-2.5, 2.5]

    # Draw obstacles
    ax = plt.gca()
    ax.set_xlim(world_bounds_x)
    ax.set_ylim(world_bounds_y)
    for k in range(len(obstacles)):
        ax.add_patch( Polygon(obstacles[k], color='k', zorder=10) )

if __name__ == "__main__":
    
    start = np.array([1.2, 1.0])
    goal =  np.array([2.1, -2.4])
    const = const()
    obstacles = [
              np.array([[-1.0, 2.0], [0.5, 2.0], [0.5, 2.5], [-1.0, 2.5]]), # my table
              np.array([[-1.0, 2.0], [0.5, 2.0], [0.5, 2.5], [-1.0, 2.5]]) + np.array([2.0, 0]), # Evgeny's table
              np.array([[-2.0, -0.5], [-2.0, 1.0], [-2.5, 1.0], [-2.5, -0.5]]), # Roman's table
              np.array([[-1.2, -1.2], [-1.2, -2.5], [-2.5, -2.5], [-2.5, -1.2]]), # mats
              np.array([[2.0, 0.8], [2.0, -0.8], [2.5, -0.8], [2.5, 0.8]]), # Mocap table
    
              # bugtrap
              np.array([[0.5, 0], [1.5, 0.], [1.5, 0.3], [0.5, 0.3]]) + np.array([-0.7, -1.5]),
              np.array([[0.5, 0.3], [0.8, 0.3], [0.8, 1.5], [0.5, 1.5]]) + np.array([-0.7, -1.5]),
              np.array([[0.5, 1.5], [1.5, 1.5], [1.5, 1.8], [0.5, 1.8]]) + np.array([-0.7, -1.5]),
              ] 
    
    passage_width = 0.3
    passage_location = 0.0
    # obstacles = [
    #         # narrow passage
    #           np.array([[-2.5, -0.5], [-passage_location-passage_width/2., -0.5], [-passage_location-passage_width/2., 0.5], [-2.5, 0.5]]),
    #           np.array([[-passage_location+passage_width/2., -0.5], [2.5, -0.5], [2.5, 0.5], [-passage_location+passage_width/2., 0.5]]),
    #         ]
    fig2D = plt.figure(figsize=(10,10))
    draw_map(obstacles)
    plt.plot(start[0],start[1],'bo',color='red', markersize=20, label='start')
    plt.plot(goal[0], goal[1],'bo',color='green', markersize=20, label='goal')
    plt.title('RRT Algorithm')
    p = rrtpath(start, goal, obstacles,const)
    s_path  = ShortenPath(p,obstacles)
    s_path = np.vstack([s_path,start])
    plt.plot(s_path[:,0],s_path[:,1],'k',color='yellow', linewidth=3,markersize = 15, label='Smoothen path')
    plt.legend()
    plt.savefig('rrt.png', dpi=300)
    plt.show()
    plt.pause(10)
