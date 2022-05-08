
import numpy as np
from numpy.linalg import norm
from math import *
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Polygon
import xlwt
import time
import os
import psutil


def init_fonts(small=12, medium=16, big=26):
    SMALL_SIZE = small
    MEDIUM_SIZE = medium
    BIGGER_SIZE = big

    plt.rc('font', size=SMALL_SIZE)          # controls default text sizes
    plt.rc('axes', titlesize=BIGGER_SIZE)     # fontsize of the axes title
    plt.rc('axes', labelsize=MEDIUM_SIZE)    # fontsize of the x and y labels
    plt.rc('xtick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
    plt.rc('ytick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
    plt.rc('legend', fontsize=MEDIUM_SIZE)    # legend fontsize
    plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title\
        
def waypts2setpts(P, params):
    """
	construct a long array of setpoints, traj_global, with equal inter-distances, dx,
	from a set of via-waypoints, P = [[x0,y0], [x1,y1], ..., [xn,yn]]
	"""
    V = params.velocity_factor * 1.3 # [m/s], the setpoint should travel a bit faster than the robot
    freq = params.ViconRate; dt = 1./freq
    dx = V * dt
    traj_global = np.array(P[-1])
    for i in range(len(P)-1, 0, -1):
        A = P[i]
        B = P[i-1]

        n = (B-A) / norm(B-A)
        delta = n * dx
        N = int( norm(B-A) / norm(delta) )
        sp = A
        traj_global = np.vstack([traj_global, sp])
        for i in range(N):
            sp += delta
            traj_global = np.vstack([traj_global, sp])
        sp = B
        traj_global = np.vstack([traj_global, sp])

    return traj_global


def formation(num_robots, leader_des, v, l):
    """
    geometry of the swarm: following robots desired locations
    relatively to the leader
    """
    u = np.array([-v[1], v[0]])
    """ followers positions """
    des2 = leader_des - v*l*sqrt(3)/2 + u*l/2
    des3 = leader_des - v*l*sqrt(3)/2 - u*l/2
    des4 = leader_des - v*l*sqrt(3)
    des5 = leader_des - v*l*sqrt(3)   + u*l
    des6 = leader_des - v*l*sqrt(3)   - u*l
    des7 = leader_des - v*l*sqrt(3)*3/2 - u*l/2
    des8 = leader_des - v*l*sqrt(3)*3/2 + u*l/2
    des9 = leader_des - v*l*sqrt(3)*2
    if num_robots<=1: return []
    if num_robots==2: return [des4]
    if num_robots==3: return [des2, des3]
    if num_robots==4: return [des2, des3, des4]
    if num_robots==5: return [des2, des3, des4, des5]
    if num_robots==6: return [des2, des3, des4, des5, des6]
    if num_robots==7: return [des2, des3, des4, des5, des6, des7]
    if num_robots==8: return [des2, des3, des4, des5, des6, des7, des8]
    if num_robots==9: return [des2, des3, des4, des5, des6, des7, des8, des9]
    
    return [des2, des3, des4]




def normalize(vector):
	if norm(vector)==0: return vector
	return vector / norm(vector)

def poses2polygons(poses, l=0.1):
    polygons = []
    for pose in poses:
        pose = np.array(pose)
        polygon = np.array([pose + [-l/2,-l/2], pose + [l/2,-l/2], pose + [l/2,l/2], pose + [-l/2,l/2]])
        polygons.append(polygon)
    return polygons

def poly_area(x,y):
    # https://stackoverflow.com/questions/24467972/calculate-area-of-polygon-given-x-y-coordinates
    # https://en.wikipedia.org/wiki/Shoelace_formula
    return 0.5*np.abs(np.dot(x,np.roll(y,1))-np.dot(y,np.roll(x,1)))

def path_length(pose_array):
    length = 0
    for i in range( 1,len(pose_array) ):
        dl = np.linalg.norm(pose_array[i,:]-pose_array[i-1,:])
        # if dl > 0.01 and dl < 0.2: length += dl
        length += dl
    return length

