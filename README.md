![](/images/enroute.png?raw=false)

# Autonomous flight in an urban environment

Starting with a city map and the respective obstacle coordinates a configuration space is derived that allows to perform motion planning tasks. With a **medial_axis transform** of the map and an informed graph search **A^* ** a path is calculated from point A to point B. The graph is then polished with the **Bresenham** raytracing method and the waypoints are send to the autopilot.

>The code is tested in the framework of the Udacity Flying Car Simulator as one part of the Nanodegree Programm Capstone Project **Motion Planning**. The other projects 

![](/images/PickupDropoff.gif)
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
The project is simulated in the [Udacity Flying Car Simulator](Udacity Flying Car Simulator) and created based on the [motion planning](https://github.com/udacity/FCND-Motion-Planning) capstone project. The drone class is provided and implemented in an event driven framework.

After installing the simulator and starting a new isntance the environment of the udacity simulation has to be acvtivated

```javascript
source activate fcnd
```

and the **Motion Planning** script can be started

```
cd ~/p2-motionPlaning 
python motion_planning.py 
```
--- 

## Configuration Space
With help of the map of a city and the corresponding obstacle data a configuration space is derived for the requested drone altitude. Two configuration spaces show the difference for different altitudes.

![](/images/enroute.png?raw=false)
![](/images/enroute.png?raw=false)

---
## Path Planning


