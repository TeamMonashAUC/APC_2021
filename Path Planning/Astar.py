"""
A* grid planning
author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)
See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)
"""

import math

import matplotlib.pyplot as plt

# show_animation = True


class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr):
        """
        Initialize grid map for a star planning
        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)
        self.ox = ox
        self.oy = oy

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy, show_animation = True):
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

        if (len(gx) != len(gy)):
            print('Error. Unequal number of gx and gy')
            return -1

        if show_animation:  # pragma: no cover
            self.create_plt(sx,sy,gx,gy)


        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
       
        open_set, closed_set = dict(), dict()
        
        rxs = []
        rys = []
        ds = []
        gkey = True

        while 1:
            
            if (gkey == True):
                goal_node = self.Node(self.calc_xy_index(gx.pop(0), self.min_x),
                              self.calc_xy_index(gy.pop(0), self.min_y), 0.0, -1)
                open_set[self.calc_grid_index(start_node)] = start_node
                gkey = False 

            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.min_x),self.calc_grid_position(current.y, self.min_y), "xc")
                
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                gkey = True
            

            if (gkey == True):
                rx, ry = self.calc_final_path(goal_node, closed_set)

                #Calculate distance of final path
                (px,py) = (None,None)
                d = 0

                for (x,y) in zip (rx,ry):
                    if ((px,py) != (None,None)):
                        d += math.hypot(x - px, y - py)

                    (px,py) = (x,y)
                #Append results to list          
                rxs.append(rx)
                rys.append(ry)
                ds.append(d)
                
                print("Found: {}, Left: {}".format(len(rxs),len(gx)) )
                if (len(gx) == 0):
                    break
                else:
                    #Empty open and closed set for next goal
                    open_set.clear()
                    closed_set.clear()
                    if show_animation:  # pragma: no cover
                        plt.clf()
                        self.create_plt(sx,sy,gx,gy)
                    continue
                    

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
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

        if show_animation:  # pragma: no cover
            for (rx,ry,d) in zip(rxs,rys,ds):
                plt.plot(rx, ry, "-r")
                plt.pause(0.001)
            plt.show()

        return rxs, rys, ds

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

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
        i = 1
        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        
        # Map obstacle locations to index
        for i in range(len(ox)):
            (iox,ioy) = (ox[i],oy[i])
            ix = (self.calc_xy_index(iox,self.min_x)-1) 
            iy = (self.calc_xy_index(ioy,self.min_y)-1)

            (pox,poy) = (ox[i-1],oy[i-1])
            px = (self.calc_xy_index(pox,self.min_x)-1) 
            py = (self.calc_xy_index(poy,self.min_y)-1)

            self.obstacle_map[ix][iy] = True        
             
            #Extrapolate obstacles
            d = math.sqrt((px-ix)*(px-ix) + (py-iy)*(py-iy))
            theta = math.degrees(math.atan2((iy-py),(ix-px)))

            n = 1
            while (n < d):
                nx = px + n*(ix-px)/d
                ny = py + n*(iy-py)/d
            
                nx1 = math.ceil(nx)
                ny1 = math.ceil(ny)
                nx2 = math.floor(nx)
                ny2 = math.floor(ny)

                self.obstacle_map[nx1][ny1] = True        
                self.obstacle_map[nx2][ny2] = True   
                n += 1  

        print('Complete map genaration')

    def create_plt(self,sx,sy,gx,gy):
        plt.plot(self.ox, self.oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    @staticmethod
    def get_motion_model():
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



