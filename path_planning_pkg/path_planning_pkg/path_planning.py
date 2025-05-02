"""

A* grid planning

author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)

See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)

This is the simple code for path planning class

"""

import math
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import PathInfo, PathTransition
from std_msgs.msg import Float64MultiArray

show_animation = True


class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr, fc_x, fc_y, tc_x, tc_y,pc_x,pc_y):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        #Cf = float(sys.argv[1]) #cost of fuel per kg
        #Ct = float(sys.argv[2]) #time related cost per minute
        #Cc = float(sys.argv[3]) #fixed cost independent of time
        #dF = float(sys.argv[4]) #trip fuel (e.g. 3000kg/h)
        #dT = float(sys.argv[5] )#trip Time (e.g. 8 hours from Hong Kong to Paris)
        #dFa = float(sys.argv[6])
        #dTa = float(sys.argv[7])

        self.resolution = resolution # get resolution of the grid
        self.rr = rr # robot radis
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model() # motion model for grid search expansion
        self.calc_obstacle_map(ox, oy)

        self.fc_x = fc_x
        self.fc_y = fc_y
        self.tc_x = tc_x
        self.tc_y = tc_y
        self.pc_x = pc_x
        self.pc_y = pc_y


        ############you could modify the setup here for different aircraft models (based on the lecture slide) ##########################
        self.C_F = 1
        self.C_T = 2
        self.C_C = 10
        self.Delta_F = 1
        self.Delta_T = 5
        self.Delta_T_A = 0.2 # additional time 
        self.Delta_F_A = 0.2 # additional fuel
        self.Cp = -4

        

        self.costPerGrid = self.C_F * self.Delta_F + self.C_T * self.Delta_T + self.C_C
        print("\nCostPerGrid for the current configuration: " + str(self.costPerGrid))

    class Node: # definition of a sinle node
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x), # calculate the index based on given position
                               self.calc_xy_index(sy, self.min_y), 0.0, -1) # set cost zero, set parent index -1
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x), # calculate the index based on given position
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict() # open_set: node not been tranversed yet. closed_set: node have been tranversed already
        open_set[self.calc_grid_index(start_node)] = start_node # node index is the grid index

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(self, goal_node,
                                                                     open_set[
                                                                         o])) # g(n) and h(n): calculate the distance between the goal node and openset
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])

            # reaching goal
            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal with cost of -> ",current.cost )
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # print(len(closed_set))

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion): # tranverse the motion matrix
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2] * self.costPerGrid, c_id)
                
                ## add more cost in time-consuming area
                if self.calc_grid_position(node.x, self.min_x) in self.tc_x:
                    if self.calc_grid_position(node.y, self.min_y) in self.tc_y:
                        # print("time consuming area!!")
                        node.cost = node.cost + self.Delta_T_A * self.motion[i][2]
                
                # add more cost in fuel-consuming area
                if self.calc_grid_position(node.x, self.min_x) in self.fc_x:
                    if self.calc_grid_position(node.y, self.min_y) in self.fc_y:
                        # print("fuel consuming area!!")
                        node.cost = node.cost + self.Delta_F_A * self.motion[i][2]

                # minus cost in minus-cost area
                if self.calc_grid_position(node.x, self.min_x) in self.pc_x:
                    if self.calc_grid_position(node.y, self.min_y) in self.pc_y:
                        node.cost = node.cost + self.Cp * self.motion[i][2]      
                
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)
        # print(len(closed_set))
        # print(len(open_set))

        return rx, ry
    
    @staticmethod
    def calculate_deltas(array):
        """
        Calculate the difference between each element and the next element in the array.
        
        :param array: numpy array
        :return: numpy array of deltas
        """
        return np.diff(array, append=array[-1])

    def cal_transition(self, rx, ry):
        # return in a single list
        # [(x1,x2),(y1,y2)]
        
        # calculate the deltas
        dx = self.calculate_deltas(rx)
        dy = self.calculate_deltas(ry)
        # combine the two lists and reverse the list
        return list(zip(dx, dy))
        
    
    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)] # save the goal node as the first point
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index
        
        return rx, ry
    

    @staticmethod
    def calc_heuristic(self, n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        d = d * self.costPerGrid
        return d
    
    def calc_heuristic_maldis(n1, n2):
        w = 1.0  # weight of heuristic
        dx = w * math.abs(n1.x - n2.x)
        dy = w * math.abs(n1.y - n2.y)
        return dx + dy

    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x) 

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)] # allocate memory
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x) # grid position calculation (x,y)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy): # Python's zip() function creates an iterator that will aggregate elements from two or more iterables. 
                    d = math.hypot(iox - x, ioy - y) # The math. hypot() method finds the Euclidean norm
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True # the griid is is occupied by the obstacle
                        break

    @staticmethod
    def get_motion_model(): # the cost of the surrounding 8 points
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion



class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning_node')
        
        # Create subscriber for PathInfo messages
        self.subscription = self.create_subscription(
            PathInfo,
            '/path/info',
            self.path_info_callback,
            10
        )
        
        # Create publisher for path transitions
        self.path_publisher = self.create_publisher(
            PathTransition,
            '/path/transitions',
            10
        )
        
        # Initialize parameters
        self.grid_size = 0.5  # [m]
        self.robot_radius = 0.5  # [m]
        
        self.get_logger().info('Path Planning Node initialized')

    def optimize_transitions(self, transitions):
        """Optimize path by combining consecutive movements in the same direction"""
        if not transitions:
            return [], []  # Return two empty arrays for x and y movements
            
        optimized_x = []
        optimized_y = []
        current_dx, current_dy = transitions[0]
        
        for dx, dy in transitions[1:]:
            if (dx == current_dx and dy == current_dy):
                # Same direction, accumulate movement
                current_dx += dx
                current_dy += dy
            else:
                # Different direction, save accumulated movement and start new
                if current_dx != 0 or current_dy != 0:  # Don't add zero movements
                    optimized_x.append(current_dx)
                    optimized_y.append(current_dy)
                current_dx, current_dy = dx, dy
                
        # Add final movement
        if current_dx != 0 or current_dy != 0:
            optimized_x.append(current_dx)
            optimized_y.append(current_dy)
            
        return optimized_x, optimized_y  # Return two arrays for x and y movements

    def path_info_callback(self, msg):
        self.get_logger().info(f'Received path info for target: {msg.name}')
        
        # Set start position
        sx, sy = 0.0, 0.0
        # Get target position from message
        target_x, target_y = msg.target
        
        # define the map size based on the vision of camera
        self.map_width = abs(int(target_x * 1.5 / self.grid_size))
        self.map_height = [int(-2 * self.grid_size), int(target_y * 1.5 / self.grid_size)]
        

        
        # Generate obstacle map
        ox, oy = [], []
        # Draw border walls
        # top border
        for i in range(-self.map_width, self.map_width):
            ox.append(float(i))
            oy.append(float(self.map_height[1]))
        # bottom
        for i in range(-self.map_width, self.map_width):
            ox.append(float(i))
            oy.append(float(self.map_height[0]))
        # left border
        for i in range(self.map_height[0], self.map_height[1] + 1):
            ox.append(float(-self.map_width))
            oy.append(float(i))
        # Draw right border
        for i in range(self.map_height[0], self.map_height[1] + 1):
            ox.append(float(self.map_width))
            oy.append(float(i))

        # Handle different object types
        if msg.name == "gate":
            gate_points = [
                [target_x - self.grid_size, target_y],     # Left
                [target_x + self.grid_size, target_y],     # Right
                [target_x, target_y + self.grid_size],     # Top
                [target_x - self.grid_size, target_y + self.grid_size], # Top-Left
                [target_x + self.grid_size, target_y + self.grid_size], # Top-Right
                [target_x - self.grid_size, target_y - self.grid_size], # Bottom-Left
                [target_x + self.grid_size, target_y - self.grid_size], # Bottom-Right
            ]
            for point in gate_points:
                ox.append(point[0] / self.grid_size)
                oy.append(point[1] / self.grid_size)
                
            gx = target_x / self.grid_size  # Center of the gate
            gy = target_y / self.grid_size  # Y position of the gate
                
        elif msg.name == "non_gate":
            obstacle_width = 5
            obstacle_height = 5
            for i in range(int(target_x - obstacle_width/2),
                         int(target_x + obstacle_width/2)):
                for j in range(int(target_y),
                             int(target_y + obstacle_height)):
                    ox.append(i)
                    oy.append(j)
                    
            # Set goal position beyond the obstacle
            gx = target_x
            gy = target_y + obstacle_height + 5
            
        elif msg.name == "flare":
            flare_size = 2
            for i in range(int(target_x - flare_size/2),
                         int(target_x + flare_size/2)):
                for j in range(int(target_y),
                             int(target_y + flare_size)):
                    ox.append(i)
                    oy.append(j)
                    
            # Set goal position near the flare
            gx = target_x
            gy = target_y + flare_size + 2
        
        # Initialize empty areas for fuel/time consumption
        fc_x, fc_y = [], []
        tc_x, tc_y = [], []
        pc_x, pc_y = [], []
        
        # Create planner and get path
        a_star = AStarPlanner(ox, oy, self.grid_size, self.robot_radius, 
                             fc_x, fc_y, tc_x, tc_y, pc_x, pc_y)
        rx, ry = a_star.planning(sx, sy, gx, gy)
        
        if rx is None:
            self.get_logger().error('No path found!')
            return
            
        # Reverse path to start from start position
        rx, ry = rx[::-1], ry[::-1]
        # Get transitions and optimize them
        transitions = a_star.cal_transition(rx, ry)
        optimized_x, optimized_y = self.optimize_transitions(transitions)
        
        # Publish optimized transitions using PathTransition custom interface
        msg = PathTransition()
        msg.target_x = optimized_x
        msg.target_y = optimized_y
        self.path_publisher.publish(msg)
        
        self.get_logger().info(f'Published optimized path with {len(optimized_x)} movements')

        # # Plotting the path and obstacles
        if show_animation:  # pragma: no cover
            plt.plot(ox, oy, ".k")  # plot the obstacles
            plt.plot(sx, sy, "og")  # plot the start position 
            plt.plot(target_x, target_y, "xb")  # plot the end position
            
            plt.plot(rx, ry, "-r")  # show the route 
            plt.grid()
            plt.axis("equal")  # set the same resolution for x and y axis 
            # plt.show()  # show the plot
            # plt.pause(0.1)
            plt.savefig('optimized_path_plot.png')  # Save the plot as a PNG file

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()