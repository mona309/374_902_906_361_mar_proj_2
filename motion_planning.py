import argparse
import time
from enum import Enum, auto

import msgpack
import numpy as np
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.frame_utils import global_to_local, local_to_global
from udacidrone.messaging import MsgID

from planning_utils import create_grid, a_star, path_prune, heuristic, pickup_goal, \
collinear_points, path_simplify, convert_25d_3d

# Default values, can be overridden by user input
TARGET_ALTITUDE = 5
SAFETY_DISTANCE = 5

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
        self.original_waypoints = []
        self.flight_history = []
        self.in_mission = True
        self.check_state = {}

        self.interactive_goal = (-122.40017151, 37.7962347, 0)
        self.temporary_scatter = None
        self.previous_location = None
        self.map_grid = None
        self.north_offset = None
        self.east_offset = None
        self.path = None

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state in [States.TAKEOFF, States.WAYPOINT, States.LANDING]:
            self.flight_history.append((self.local_position[0], self.local_position[1], -self.local_position[2]))
            
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 3.0 \
                    and abs(self.target_position[2] - (-self.local_position[2])) < 2.0:
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
        if not self.waypoints:
            print("No waypoints! Landing...")
            self.landing_transition()
            return
        print(self.waypoints)
        self.target_position = self.waypoints.pop(0)
        self.target_position[2] = max(0, self.target_position[2])
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2],
                        self.target_position[3])

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

    def plot_flight_path(self):
        print("Consolidating flight logs and generating trajectory plot...")
        import matplotlib.pyplot as plt
        
        if not self.flight_history or not self.original_waypoints:
            print("No flight history to plot.")
            return

        planned_e = [w[1] for w in self.original_waypoints]
        planned_n = [w[0] for w in self.original_waypoints]
        
        actual_e = [h[1] for h in self.flight_history]
        actual_n = [h[0] for h in self.flight_history]

        plt.figure(figsize=(10, 8))
        plt.plot(planned_e, planned_n, 'rX-', label='Planned Path', linewidth=2, markersize=8)
        plt.plot(actual_e, actual_n, 'b--', label='Actual Flight Trajectory', alpha=0.7)
        
        plt.scatter(actual_e[0], actual_n[0], c='g', marker='o', s=150, label='Start')
        plt.scatter(planned_e[-1], planned_n[-1], c='purple', marker='*', s=200, label='Goal')
        
        plt.xlabel('East (meters)')
        plt.ylabel('North (meters)')
        plt.title('Autonomous Drone Flight: Planned vs Actual Path')
        plt.legend()
        plt.grid(True)
        plt.savefig('flight_trajectory.png')
        print("Success! Flight path plot saved as 'flight_trajectory.png'.")
        plt.show()

    def pick_goal(self, event):
        evt = event.mouseevent
        east = int(evt.xdata)
        north = int(evt.ydata)
        alt = self.map_grid[north, east]
        self.interactive_goal = local_to_global(self.grid_to_local((north, east, alt)), self.global_home)

        if self.temporary_scatter is not None:
            self.temporary_scatter.remove()
        fig = event.artist.figure
        self.temporary_scatter = fig.gca().scatter(east, north, marker='o', c='g')
        fig.canvas.draw()
        print("The goal location is (lat, lon, alt) {}"
                "Close figure to start simulator.".format(self.interactive_goal))

    def grid_to_local(self, grid_coord):
        lat = grid_coord[0] + self.north_offset
        lon = grid_coord[1] + self.east_offset
        return lat, lon, -grid_coord[2]

    def local_to_grid(self, position):
        north = int(position[0] - self.north_offset)
        east = int(position[1] - self.east_offset)
        alt = int(-position[2])
        return north, east, alt

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching path...")

        self.target_position[2] = TARGET_ALTITUDE

        try:
            with open('colliders.csv', 'r') as f:
                header_line = f.readline()
                lat_str, lon_str = header_line.split(',')
                lat = float(lat_str.strip().split(' ')[1])
                lon = float(lon_str.strip().split(' ')[1])
                print("Map home location: ({}, {})".format(lat, lon))
        except Exception as e:
            print(f"ERROR: Could not read 'colliders.csv'. Exception: {e}")
            print("Aborting mission and initiating landing sequence...")
            self.landing_transition()
            return

        home_position = (lon, lat, 0)
        self.set_home_position(*home_position)

        global_position = self.global_position

        local_position = global_to_local(global_position, home_position)

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                        self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype=np.float64, skiprows=2)

        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, SAFETY_DISTANCE)
        self.map_grid = grid
        self.north_offset = north_offset
        self.east_offset = east_offset

        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # starting point on the grid
        grid_start = self.local_to_grid(local_position)
        alt_start = int(max(TARGET_ALTITUDE, grid_start[2] + 1, grid[grid_start[0], grid_start[1]] + 1))
        grid_start = grid_start[0], grid_start[1], alt_start

        # visualize grid: interavtive goal pick up
        self.temporary_scatter = pickup_goal(grid, grid_start, self.pick_goal)

        goal = self.interactive_goal
        if len(goal) < 3:
            goal = (goal[0], goal[1], 0)
        goal_local = global_to_local(goal, self.global_home)
        goal_grid = self.local_to_grid(goal_local)
        goal_north, goal_east, goal_alt = goal_grid
        grid_goal = (goal_north,
                    goal_east,
                    int(max(grid[goal_north, goal_east] + 1, TARGET_ALTITUDE, goal_alt + 1)))

        print('Start and Goal location', grid_start, grid_goal)
        print("Searching path...")
        try:
            path = a_star(grid, heuristic, grid_start, grid_goal, TARGET_ALTITUDE)
            
            if path is None or len(path) == 0:
                print("ERROR: A* search could not find a valid path to the requested goal! Drone is returning to start or landing.")
                self.landing_transition()
                return

            path = path_prune(path, collinear_points)
            print("3D Pruned Path:", path)
            path = path_simplify(grid, path, SAFETY_DISTANCE)
            print("Path found!")
            print(path)
            self.path = path
            waypoints = self.path_to_waypoints(path)
            self.waypoints = list(waypoints)
            self.original_waypoints = list(waypoints)
            self.send_waypoints()
        except Exception as e:
            print(f"CRITICAL ERROR during path calculation: {e}. Initiating emergency landing!")
            self.landing_transition()
            return

    def path_to_waypoints(self, path):
        # Convert path to waypoints
        waypoints = []
        for i in range(len(path)):
            p = path[i]
            p_prev = path[i - 1] if i > 0 else None
            orientation = 0
            if p_prev is not None:
                orientation = np.arctan2(p[1] - p_prev[1], p[0] - p_prev[0])
            waypoints.append([p[0] + self.north_offset, p[1] + self.east_offset, p[2], orientation])
        return waypoints

    def start(self):
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    
    print("\n--- Drone Configuration Interface ---")
    try:
        alt_input = input(f"Enter target altitude (default {TARGET_ALTITUDE}m): ")
        if alt_input.strip():
            TARGET_ALTITUDE = int(alt_input)
            
        safe_input = input(f"Enter safety margin around buildings (default {SAFETY_DISTANCE}m): ")
        if safe_input.strip():
            SAFETY_DISTANCE = int(safe_input)
    except ValueError:
        print("Invalid input. Using default values.")
    print(f"Configuration Set -> Altitude: {TARGET_ALTITUDE}m | Safety Margin: {SAFETY_DISTANCE}m\n")

    time.sleep(1)

    try:
        drone.start()
    except KeyboardInterrupt:
        print("\n\n############################################")
        print("!!! EMERGENCY KILL SWITCH ACTIVATED !!!")
        print("############################################")
        print("Terminating mission, attempting immediate landing...")
        
        try:
            if drone.armed:
                drone.landing_transition()
                drone.disarming_transition()
            drone.manual_transition()
        except:
            print("Drone forcefully shutdown.")
        finally:
            print("Drone controls released.")
    finally:
        # Once the simulator closes or finishes, generate the trajectory plot
        drone.plot_flight_path()
