"""

@brief RRT-APF combined algorithm


"""

from rrtstar import *
from APF import *



class RRT_APF_Combined:
    def __init__(self,path,p_short,obstacles,params):
        self.p_short = p_short
        self.path = path
        self.obstacles = obstacles
        self.path = path
        self.path_rrt = np.array([path[-1,:]])
        self.params = params
        
        
    def run(self):
        for i in range(len(self.path)-1,0,-1):
            start = self.path_rrt[-1,:]
            goal = self.p_short[i-1]
            
            path_apf,potential= APF(self.obstacles,start,goal,self.params).plan()
            
            
            # plt.plot(start[0],start[1],'bo',color = 'red',markersize = 10)
            # plt.plot(goal[0],goal[1],'bo',color = 'green',markersize = 10)
            
            
            path_apf_rrt = np.vstack([self.path_rrt ,path_apf])
            
       
        
        return path_apf_rrt
    
    
    
if __name__ == '__main__':
    
    plt.figure(figsize = (10,10))
    start = np.array([1.2, 1.0])
    goal =  np.array([1.75, -1.4])
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
    draw_map(obstacles)
   
    path = rrtpath(start, goal, obstacles,const)
    s_path  = ShortenPath(path,obstacles)
    # s_path = np.vstack([s_path,start])
    params = Parameters()
    obstacles = grid_map(obstacles)
    layered_path = RRT_APF_Combined(path,s_path,obstacles,params).run()       
    plt.plot(s_path[:,0],s_path[:,1],'-',color = 'orange',linewidth = 2,label = 'Shortened Path')
    plt.plot(start[0],start[1],'bo',color='red', markersize=20, label='start')
    plt.plot(goal[0], goal[1],'bo',color='green', markersize=20, label='goal')
    plt.plot(layered_path[:,0],layered_path[:,1],'.',color = 'yellow',linewidth = 2,label = 'RRT-APF Combined')
    plt.legend()
    plt.pause(13)
     
            
            