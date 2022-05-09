# Adaptive-Layered-Planner
Layered Planner using RRT as a global planner and Artificial Potential Field as Local Planner



# Steps to Run the code

```
git clone https://github.com/karanamrahul/Adaptive-Layered-Planner.git
cd Adaptive-Layered-Planner/
python3 combined.py

```


## To change obstacle space, start & goal pos, num of robots .

- Please go the combined.py file and change in the Constants class for other parameters.
- xy_start & xy_goal are the start and goal points.

### To run APF and RRT individually , first uncomment the code which is commented on the end of each file and run the code.


## APF
```
python3 APF.py
```


## RRT
```
python3 rrt.py
```

### Results


Please refer this drive link for the output videos

Google Drive Link : https://drive.google.com/drive/folders/1selsuN77piMY9i_CcUnBPz3N9GBXBf1B?usp=sharing


## References

- https://github.com/AtsushiSakai/PythonRobotics
- https://github.com/RuslanAgishev/motion_planning