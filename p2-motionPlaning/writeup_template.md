## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
These scripts are seperated in a main script motion_planing, which calls functions from planning_utils.py and grid.py. 

In `planning_utils.py` the valid action, the path finding algorithmus with the heuristice and a path pruning algorithmus are defined. I also added an heading function and a randomised goal state function in there. 

In the `grid.py` i put the grid creation algorithm. It is preparing a grid for the medial axis transform algorithm which is called in the `motion_planning.py`

In the beginning the zick-zack path could be smoothend by adding a collinearty check with broad epsilons and allowing the vehicle diagonal movement.

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
First the latitude and longitude are read and transformed with textread and float commands from the first line of the collision.csv. With those the homeposition is refrenced to to the csv collision map.

	# read lat0, lon0 from colliders into floating point values
	data_pos = np.loadtxt('colliders.csv',dtype='str', max_rows=1)
	(lat0,lon0) = [float(data_pos[1][:-1]),float(data_pos[3][:-1])]
	# set home position to (lon0, lat0, 0)
	self.set_home_position(lon0, lat0, 0)

#### 2. Set your current local position
For knowing the current local position we have to know the global position and the global home position. With those two and the provided function global_to_local the current_local_pos can be derived.

	# retrieve current global position
    global_position = [self._longitude, self._latitude, self._altitude]
    # convert to current local position using global_to_local()        
    current_local_pos = global_to_local(global_position,self.global_home)

#### 3. Set grid start position from local position
For the grid starting position the offset values are required. Those values give the refrence between the local position and the NED coordinate system of the grid.

	start_ne = (int(self.local_position[0]-north_offset), int(self.local_position[1]-east_offset))
#### 4. Set grid goal position from geodetic coords
To demonstrate this one can use global_to_local with the specified lat and lon. I commented it out in the submitted code, because i decided for a randomised goal algorithm, using the NED coordinated of the grid.

	# local_goal to show expected transformation
    goal_ne = global_to_local(global_goal, self.global_home)

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
I submitted a version with the initial A* algorithm, following you can see the diagonal cost and valid_action implementations.

    NORTHWEST = (-1, -1, np.sqrt(2))
    SOUTHWEST = (1, -1, np.sqrt(2))
    NORTHEAST = (-1, 1, np.sqrt(2))
    SOUTHEAST = (1, 1, np.sqrt(2))
    ...
	if (x - 1 < 0 and y - 1 < 0) or (grid[x - 1, y] == 1 and grid[x, y - 1] == 1):
        valid_actions.remove(Action.NORTHWEST)
    if (x + 1 > n and y - 1 < 0) or (grid[x + 1, y] == 1 and grid[x, y - 1] == 1):
        valid_actions.remove(Action.SOUTHWEST)
    if (x - 1 < 0 and y + 1 > m) or (grid[x - 1, y] == 1 and grid[x, y + 1] == 1):
        valid_actions.remove(Action.NORTHEAST)
    if (x + 1 > n and y + 1 > m) or (grid[x + 1, y] == 1 and grid[x, y + 1] == 1):
        valid_actions.remove(Action.SOUTHEAST)

#### 6. Cull waypoints 
I used a collinearity check from the lectures and exercises with relatively broad epsilon, therefore i receive a relative smooth path without that many zick-zack in between stops. 

	def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)

	def collinearity_check(p1, p2, p3, epsilon=10):   
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon

    def collinearity(path):
    pruned_path = [p for p in path]
    i = 0
    while i < len(pruned_path) - 2:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i+1])
        p3 = point(pruned_path[i+2])
        if collinearity_check(p1, p2, p3):
            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1
    return pruned_path

### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

I added a heading function, so that the yaw angle of the vehicle adapts to the next point.

