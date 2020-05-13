#!/usr/bin/env python
import rospy as rp
from grid_map import GridMap
import numpy as np
import random as rd

np.random.seed(444)
vertices = []

class RRT(GridMap):
    def __init__(self):
        super(RRT, self).__init__()
        self.step = 0.05

    def check_if_valid(self, a, b):
        
        in_free_space = True
        x_list = np.linspace(a[0], b[0], num=100)
        y_list = np.linspace(a[1], b[1], num=100)

        for index in range(len(x_list)):
            x = x_list[index]
            y = y_list[index]
            val = self.map[int(y * 10), int(x * 10)]
            if val == 100:
                in_free_space = False
            else:
                pass

        return in_free_space

    def random_point(self):

        x = round(float(rd.randrange(0, int(self.width * 10), int(self.resolution * 10)))/10, 1)
        y = round(float(rd.randrange(0, int(self.height * 10), int(self.resolution * 10)))/10, 1)
        return np.array([x, y])

    def find_closest(self, pos):

        lengths = []
        closest_index = 0
        current_length = 500
        for ver in vertices:

            length = float(((ver[0] - pos[0]) ** 2.00 + (ver[1] - pos[1]) ** 2.00) ** 0.50)
            lengths.append(length)

        for index, length in enumerate(lengths):
            if length < current_length:
                current_length = length
                closest_index = index

        closest = vertices[closest_index]
        return closest

    def new_pt(self, pt, closest):

        a = abs(closest[0] - pt[0])
        b = abs(closest[1] - pt[1])
        c = float((a ** 2.00 + b ** 2.00) ** 0.50)

        if c != 0:
            sin = float(a/c)
            cos = float(b/c)
            a_new = sin * self.step
            b_new = cos * self.step

        else:
            a_new = 0
            b_new = 0

        if closest[0] < pt[0] and closest[1] < pt[1]:
            pt = tuple([closest[0] + a_new, closest[1] + b_new])

        elif closest[0] < pt[0] and closest[1] > pt[1]:
            pt = tuple([closest[0] + a_new, closest[1] - b_new])

        elif closest[0] >= pt[0] and closest[1] >= pt[1]:
            pt = tuple([closest[0] - a_new, closest[1] - b_new])

        elif closest[0] > pt[0] and closest[1] < pt[1]:
            pt = tuple([closest[0] - a_new, closest[1] + b_new])

        else:
            pt = tuple([closest[0] + a_new, closest[1] + b_new])

        return pt

    def search(self):

        path = []
        vertices.append(self.start)

        while(1):
            pt = self.random_point()
            closest = self.find_closest(pt)
            new_point = self.new_pt(pt, closest)
            validity_1 = self.check_if_valid(new_point, closest)
            if validity_1 and new_point not in vertices:
                vertices.append(new_point)
                self.parent[new_point] = closest
            self.publish_search()
            validity_2 = self.check_if_valid(new_point, self.end)
            if validity_2:
                vertices.append(self.end)
                self.parent[self.end] = new_point
                actual = self.end
                while actual is not None:
                    path.append(actual)
                    if actual == self.start:
                        break
                    actual = self.parent[actual]
                path = path[::-1]
                print("Znaleziono rozwiazanie")
                break
        self.publish_search()
        self.publish_path(path)



if __name__ == '__main__':
    rrt = RRT()
    rrt.search()
