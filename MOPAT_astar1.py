import matplotlib.pyplot as plt
import math
import numpy as np


class Astar:

    def __init__(self, obmap, minx, miny, maxx, maxy):
        self.minx = minx
        self.miny = miny
        self.maxx = maxx
        self.maxy = maxy
        self.xwidth = round(maxx - minx)
        self.ywidth = round(maxy - miny)
        self.obmap = obmap
        self.motion = self.dynamics()

    class Node:
        def __init__(self, x, y, cost, previous_node):
            self.x = x
            self.y = y
            self.cost = cost
            self.previous_node = previous_node

    def find_best_route(self, sx, sy, gx, gy):
        self.path=True
        start_node = self.Node(sx-self.minx,
                           sy -self.miny, 0, -1)
        goal_node = self.Node(gx- self.minx,
                          gy-self.miny, 0, -1)

        explore, Visited = dict(), dict()
        explore[self.calc_index(start_node)] = start_node

        while 1:
            if not bool(explore):
                self.path=False
                break
            c_id = min(explore, key=lambda o: explore[o].cost)
            current = explore[c_id]
            if current.x == goal_node.x and current.y == goal_node.y:
                print("Found goal")
                goal_node.previous_node = current.previous_node
                goal_node.cost = current.cost
                break

            # delete it explore set
            del explore[c_id]

            # Add it to the visited set
            Visited[c_id] = current

            # searching the adjacent points
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2]+self.calc_hvalue(current,goal_node), c_id)
                node_id = self.calc_index(node)

                if node_id in Visited:
                    continue

                if self.free_node(node):
                    continue

                if node_id not in explore:
                    explore[node_id] = node
                elif explore[node_id].cost >= node.cost:
                    explore[node_id] = node

        px, py = self.calc_final_path(goal_node, Visited)
        if self.path :
            print("path exist")
        else:
            print("path does not exist")

        return px, py

    def calc_hvalue(self, node, goal):
        return math.sqrt ((node.x - goal.x)**2+(node.y - goal.y)**2)

    def calc_final_path(self, goal_node, Visited):
        px, py = [self.calc_position(goal_node.x, self.minx)], [
            self.calc_position(goal_node.y, self.miny)]
        previous_node = goal_node.previous_node
        while previous_node != -1:
            n = Visited[previous_node]
            px.append(self.calc_position(n.x, self.minx))
            py.append(self.calc_position(n.y, self.miny))
            previous_node = n.previous_node
        return px, py

    def calc_position(self, index, shift):
        pos = index+shift
        return pos

    def calc_index(self, node):
        return (node.y - self.miny) * self.xwidth + (node.x - self.minx)

    def free_node(self, node):
        qx = self.calc_position(node.x, self.minx)
        qy = self.calc_position(node.y, self.miny)
        if qx in range(self.minx,self.maxx) and qy in range(self.miny,self.maxy) and not self.obmap[node.x][node.y]:
            return False
        return True

    def dynamics(self):
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]
        return motion



print("Lets use Astar!!!")

# start and goal coordinates
sx = -3
sy = -5
gx = 53
gy = 50

# set obstacle positions
ox, oy = [], []
for i in range(-10, 60):
    ox.append(i)
    oy.append(-10)
for i in range(-10, 60):
    ox.append(60)
    oy.append(i)
# for i in range(-10, 50):
#     ox.append(20)
#     oy.append(i)
for i in range(-10, 61):
    ox.append(i)
    oy.append(60)
for i in range(-10, 61):
    ox.append(-10)
    oy.append(i)
for i in range(20, 60):
    ox.append(20)
    oy.append(i)
for i in range(0, 40):
    ox.append(40)
    oy.append(60 - i)



plt.plot(ox, oy, ".k")
plt.plot(sx, sy, "og")
plt.plot(gx, gy, "xb")
plt.grid(True)
plt.axis("equal")




minx = round(min(ox))
miny = round(min(oy))
maxx = round(max(ox))
maxy = round(max(oy))
xwidth = round(maxx - minx)
ywidth = round(maxy - miny)
# obstacle map generation
obmap=np.zeros((xwidth+1, ywidth+1), dtype=bool)
# self.obmap[(np.asarray(ox)-self.minx).tolist()][(np.asarray(oy)-self.miny).tolist()]=1
# obmap=np.asarray(in_map==1,dtype=bool)
for iox, ioy in zip(ox, oy):
    obmap[iox-minx][ioy-miny] = True


Astar = Astar(obmap,minx,miny,maxx,maxy)
px, py = Astar.find_best_route(sx, sy, gx, gy)
# print(px)
# print(py)
plt.plot(px, py, "r")

# plt.show()
plt.pause(10)
print("Done")
