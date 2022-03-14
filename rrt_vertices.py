#!/usr/bin/env python
import rospy as rp
from grid_map import GridMap
import numpy as np

np.random.seed(444)


class RRT(GridMap):
    def __init__(self):
        super(RRT, self).__init__()
        self.step = 0.1

    def check_if_valid(self, a, b):
        """
        Checks if the segment connecting a and b lies in the free space.
        :param a: point in 2D
        :param b: point in 2D
        :return: boolean
        """
        size = 100
        check=0
        length = float(np.sqrt((b[0] - a[0]) *(b[0] - a[0]) + (b[1] - a[1]) *(b[1] - a[1]) ))
        array = np.arange(a, b, length / size, )
        for i in range(len(array)):
            if (self.map.data[array[i,0] + array[i,1] * self.map.info.width] )< 50:
                check+1
        if check==(len(array)-1):
         in_free_space = True
        return in_free_space

    def random_point(self):
        """
        Draws random point in 2D
        :return: point in 2D
        """
        x = int(np.random.randint(self.width,size = 1))
        y = int(np.random.randint(self.height,size = 1))

        return np.array([x, y])

    def find_closest(self, pos):
        """
        Finds the closest vertex in the graph to the pos argument

        :param pos: point id 2D
        :return: vertex from graph in 2D closest to the pos
        """
        very_long=1000000
        number = 0
        for i in range(len(self.parent)):
            length=np.sqrt(pow((self.parent.keys()[i][0]-pos[0]),2)+pow((self.parent.keys()[i][1]-pos[1]),2))
            if length<very_long:
                very_long= length
                number = i

        closest = np.array([self.parent.keys()[number][0],self.parent.keys()[number][1]])
        return closest

    def new_pt(self, pt, closest):
        """
        Finds the point on the segment connecting closest with pt, which lies self.step from the closest (vertex in graph)

        :param pt: point in 2D
        :param closest: vertex in the tree (point in 2D)
        :return: point in 2D
        """
        if pt[0]<closest[0]:
            step=self.step
        else:
            step=-self.step

        xarray = np.arange(pt[0], closest[0], step)
        if pt[1] < closest[1]:
            step = self.step
        else:
            step = -self.step
        yarray = np.arange(pt[1], closest[1], step)
        indx = len(xarray) - 1
        indy = len(yarray) - 1
        x = xarray[indx]
        y = yarray[indy]
        pt = np.array([x, y])
        return pt

    def search(self):
        """
        RRT search algorithm for start point self.start and desired state self.end.
        Saves the search tree in the self.parent dictionary, with key value pairs representing segments
        (key is the child vertex, and value is its parent vertex).
        Uses self.publish_search() and self.publish_path(path) to publish the search tree and the final path respectively.
        """
        self.parent[self.start] = None
        point = self.random_point()
        close_point = self.find_closest(point)
        new_point = self.new_pt(point, close_point)
        close_point_1 = self.find_closest(new_point)
        is_valid= self.check_if_valid(new_point, close_point_1)
        if is_valid == true:
            self.publish_search()
            if check_if_valid(new_point, self.end):
                self.parent.append(new_point)
                self.publish_path(path)

        while not rp.is_shutdown():
            rp.sleep(0.01)


if __name__ == '__main__':
    rrt = RRT()
    rrt.search()
