import argparse
import time
import msgpack
import utm
from enum import Enum, auto
from sampling import Sampler
import numpy as np
from grid import create_grid
from planning_utils import a_star, heuristic
from shapely.geometry import Polygon, Point, LineString
import networkx as nx
import pkg_resources
pkg_resources.require("networkx==2.1")
from sklearn.neighbors import KDTree
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        def can_connect(n1, n2, polygons):
            l = LineString([n1, n2])
            for p in polygons:
                if p.crosses(l) and p.height >= min(n1[2], n2[2]):
                    return False
            return True

        def create_graph(nodes, k, polygons):
            g = nx.Graph()
            tree = KDTree(nodes)
            for n1 in nodes:
                # for each node connect try to connect to k nearest nodes
                idxs = tree.query([n1], k, return_distance=False)[0]
                
                for idx in idxs:
                    n2 = nodes[idx]
                    if n2 == n1:
                        continue
                        
                    if can_connect(n1, n2, polygons):
                        g.add_edge(n1, n2, weight=1)
            return g

        # TODO: read lat0, lon0 from colliders into floating point values
        data_pos = np.loadtxt('colliders.csv',dtype='str', max_rows=1)
        (lat0,lon0) = [float(data_pos[1][:-1]),float(data_pos[3][:-1])]
        # TODO: set home position to (lon0, lat0, 0)
        (east_home, north_home,_,_) = utm.from_latlon(lat0, lon0)

        # TODO: retrieve current global position
        (east, north,_,_) = utm.from_latlon(self.global_position[1], self.global_position[0])
        # TODO: convert to current local position using global_to_local()
        local_position = np.array([north - north_home, east - east_home, -(self.global_position[2] - self.global_home[2])])
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Sample points
        sampler = Sampler(data)
        polygons = sampler._polygons
        nodes = sampler.sample(150)
        print(len(nodes))
        
        # Connect nodes to graph
        g = create_graph(nodes, 10, polygons)
        #grid = create_grid(data, sampler._zmax, 1)

        # define start as first point from graph, goal point is randomized 
        start = list(g.nodes)[0]
        k = np.random.randint(len(g.nodes))
        print("Amount of found nodes:{0} {1}".format(k, len(g.nodes)))
        goal = list(g.nodes)[k]

        # Run A* to find path
        path, cost = a_star(g, heuristic, start, goal)
        path_pairs = zip(path[:-1], path[1:])
        for (n1, n2) in path_pairs:
            print(n1, n2)

        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        '''
        # Define starting point on the grid (this is just grid center)
        grid_start = (int(local_position[0]-north_offset), int(local_position[1]-east_offset))
        # TODO: convert start position to current position rather than map center
        
        # Set goal as some arbitrary position on the grid
        grid_goal = (int(local_position[0]-north_offset), int(local_position[1]-east_offset))
        # TODO: adapt to set goal as latitude / longitude position and convert

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        path_, _ = a_star(grid, heuristic, grid_start, grid_goal)
        # TODO: prune path to minimize number of waypoints
        path = collinearity(path_)
        # TODO (if you're feeling ambitious): Try a different approach altogether!
        '''
        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        print(waypoints)
        print(grid_start, grid_goal)
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
