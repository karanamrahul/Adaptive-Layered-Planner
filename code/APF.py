
# Implement Artificial Potential Field as a Local Planner


# Importing libraries
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import distance_transform_edt as dt



# Defining the class

class APF:
    def __init__(self,obstacles,start,goal,params):
        self.obstacles = grid_map(obstacles)  # Obstacles
        self.start = m2grid(start)  # start position
        self.goal = m2grid(goal) # goal is a tuple (x,y)
        self.params = params # parameters for the APF
        
        
    def plan(self):
        [x,y] = np.meshgrid(np.arange(self.params.cols),np.arange(self.params.rows))
        attractive_pot = self.params.coef_attr * ((x - self.goal[0])**2 + (y - self.goal[1])**2)
        dist = dt(self.obstacles==0)
        row_x = (dist / 100.0) + 1
        repulsive_pot = self.params.coef_rep * ((1 / row_x) - (1 / self.params.radius))**2
        repulsive_pot[row_x > self.params.radius] = 0 
        self.potential = attractive_pot + repulsive_pot
        
    
        """ 
        Now we find the gradient of the potential field at each point
            
        We reach the gradient at each point by subtracting the potential field at the point from the potential field at the next point.
        
        To reach the goal we need to find the gradient of the potential field at the goal point.
            
        """
    
        [f_y,f_x] = np.gradient(-self.potential)
        
        path = np.vstack([np.array(self.start), np.array(self.start)])
        
        for i in range(self.params.n_iters):
            current_cell = path[-1,:]
            if(sum(abs(current_cell - self.goal)) < 0.01):
                print('Goal Reached')
                break
            # We find the gradient at the current cell
            i_x,i_y = int(round(current_cell[1])), int(round(current_cell[0]))
            v_x = f_x[i_x,i_y]
            v_y = f_y[i_x,i_y]
            d_t =1/np.linalg.norm([v_x,v_y])
            next_cell = current_cell + d_t * np.array([v_x,v_y])
            path = np.vstack([path, next_cell])
        # path = path[1:,:]
        path = grid2m(path)
        return path, self.potential


class Parameters:
    def __init__(self):
        self.radius  = 2 # Influence radius of the potential field
        self.coef_attr = 1./700 # Coefficient of attraction
        self.rows = 600 # Number of rows
        self.cols = 600 # Number of columns
        self.coef_rep = 200 # Coefficient of repulsion
        self.n_iters = 700 # Number of iterations


def m2grid(pose_m, nrows=500, ncols=500):
    # [0, 0](m) -> [250, 250]
    # [1, 0](m) -> [250+100, 250]
    # [0,-1](m) -> [250, 250-100]
    if np.isscalar(pose_m):
        pose_on_grid = int( pose_m*100 + ncols/2 )
    else:
        pose_on_grid = np.array( np.array(pose_m)*100 + np.array([ncols/2, nrows/2]), dtype=int )
    return pose_on_grid


def grid2m(pose_grid, nrows=500, ncols=500):
    # [250, 250] -> [0, 0](m)
    # [250+100, 250] -> [1, 0](m)
    # [250, 250-100] -> [0,-1](m)
    if np.isscalar(pose_grid):
        pose_meters = (pose_grid - ncols/2) / 100.0
    else:
        pose_meters = ( np.array(pose_grid) - np.array([ncols/2, nrows/2]) ) / 100.0
    return pose_meters

def draw_gradient(f, nrows=500, ncols=500):
    skip = 10
    [x_m, y_m] = np.meshgrid(np.linspace(-2.5, 2.5, ncols), np.linspace(-2.5, 2.5, nrows))
    [gy, gx] = np.gradient(-f);
    Q = plt.quiver(x_m[::skip, ::skip], y_m[::skip, ::skip], gx[::skip, ::skip], gy[::skip, ::skip])
    
def grid_map(obstacles, nrows=500, ncols=500):
    """ Obstacles dicretized map """
    grid = np.zeros((nrows, ncols));
    # rectangular obstacles
    for obstacle in obstacles:
        x1 = m2grid(obstacle[0][1]); x2 = m2grid(obstacle[2][1])
        y1 = m2grid(obstacle[0][0]); y2 = m2grid(obstacle[2][0])
        grid[x1:x2, y1:y2] = 1

    return grid

def combined_potential(obstacles_grid, goal, influence_radius=1, attractive_coef=1./700, repulsive_coef=200, nrows=500, ncols=500):
    """ Repulsive potential """
    goal = m2grid(goal)
    d = dt(obstacles_grid==0)
    d2 = (d/100.) + 1 # Rescale and transform distances
    d0 = influence_radius + 1
    nu = repulsive_coef
    repulsive = nu*((1./d2 - 1./d0)**2)
    repulsive [d2 > d0] = 0
    """ Attractive potential """
    [x, y] = np.meshgrid(np.arange(ncols), np.arange(nrows))
    xi = attractive_coef
    attractive = xi * ( (x - goal[0])**2 + (y - goal[1])**2 )
    """ Combine terms """
    total = attractive + repulsive
    return total, attractive, repulsive

if __name__ == '__main__':
    
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
   start = np.array([1.2, 1.0])
   goal = np.array([1.5,  -1.4])
   params = Parameters()
   apf_potential  = APF(obstacles,start,goal,params) # Create the potential field object
   path,potential = apf_potential.plan()
#    plt.figure = plt.figure(figsize=(10,10))
#    draw_gradient(potential)
#    plt.plot(path[:,0], path[:,1], 'r-')
#    plt.plot(start[0], start[1], color = 'green',markersize = 10, marker = 'o')
#    plt.plot(goal[0], goal[1], color = 'red',markersize = 10, marker = 'x')
#    plt.legend(['Path', 'Start', 'Goal'])
#    plt.show()
    