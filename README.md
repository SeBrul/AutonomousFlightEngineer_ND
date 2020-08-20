![](/images/enroute.png?raw=false)

# Autonomous flight in an urban environment

Starting with a city map and the respective obstacle coordinates a configuration space is derived that allows performing motion planning tasks. With a **medial_axis transform** of the map and an informed graph search **A^* ** a path is calculated from point A to point B. The graph is then polished with the **Bresenham** raytracing method and the waypoints are sent to the autopilot.

>The code is tested in the framework of the Udacity Flying Car Simulator as one part of the Nanodegree Programm Capstone Project **Motion Planning**. The other projects 

![](/images/drone_starting.gif)
---

## Table of Contents
[Motion Planning](#motion-planning)
- [Built With](#built-with)
- [Configuration Space](#configuration-space)
- [Path Planning](#path-planning)
- [Ray Tracing](#ray-tracing)
[Further Projects](#further-projects)

---
## Build With
The project is simulated in the [Udacity Flying Car Simulator](Udacity Flying Car Simulator) and created based on the [motion planning](https://github.com/udacity/FCND-Motion-Planning) capstone project. The drone class is provided and implemented in an event-driven framework.

After installing the simulator and starting a new instance the environment of the udacity simulation has to be activated

```javascript
source activate fcnd
```

and the **Motion Planning** script can be started

```javascript
cd ~/p2-motionPlaning 
python motion_planning.py 
```

The other projects can be started with the same command, and it is just required to be in the corresponding folder and change the file name.
--- 

## Configuration Space
With the help of the map of a city and the corresponding obstacle data, a configuration space is derived for the requested drone altitude. The configuration space is the base for all path planning performed later on an derived from a given set of information of the city Los Angeles.

![](/images/map.png?raw=false)
![](/images/grid_map_higher.png?raw=false)

---
## Path Planning
With the help of the **medial axis** transform the image can be skeletonized in order to find possible paths on which the drone can travel. The advantage of this method is that it provides pathways with a local maximum distance between any obstacles and the drone. On the other side, those paths are not the most efficient when going from A to B and therefore not optimal.

![](/images/medial-axis.png?raw=false)

Performing an **A^*** graph search on the paths from the **medial axis** transformed map (green) delivers a possible path for the drone to travel. Also, an **A^*** graph search performed directly onto the configuration map (red) achieves a more direct path towards the goal.

![](/images/compare-medialaxis-wo.png?raw=false)

---
## Ray Tracing
In order to remove unnecessary waypoints that lay on an already existing node, a raytracing method is used. In order to avoid floating-point operations during the collinearity check, the **Bresenham** algorithm is used. After performing the optimization, the path is pruned down to fewer coordinates points, leading to fewer tasks for the autopilot and a more smooth travelling path, depending on the death band of the drone.
![](/images/path-pruning.png?raw=false)

---
## Putting It Together
In the motion planning script, all these methods are applied. First, two random coordinates are calculated, a start and a goal point. Then a path is calculated based on the **medial axis** transform and **A^* ** algorithm, the path is pruned down, and the coordinates are commanded to the drone one by one, as soon as the drone reaches the next point. 

![](/images/drone_motion_planning_full.gif)

---
## Further Projects
In addition to the **Motion Planning** project, two interesting other projects are part of the Nanodegree Programm. One is the [**Controll**](https://github.com/SeBrul/Flying-Car-Nanodegree/tree/master/p3-control/p3-control-c) project, in which a nonlinear cascaded controller is developed and tested in C++ to achieve appropriate low-level motor control. The drone is tested in a provided [C++ simulator](https://github.com/udacity/FCND-Controls-CPP) A modified version is then loaded on a real Crazyflie 2.0 drone and tests. In the [**Estimation**](https://github.com/SeBrul/Flying-Car-Nanodegree/tree/master/p4-estimation) project sensor fusion and filtering are utilized to overcome sensor noise in real-world scenarios. With a developed **Extended Kalman Filter** (EKF), the attitude and position are estimated based on IMU and GPS data.
