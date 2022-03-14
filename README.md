# Robotic_Planning_Methods
RRT algorithm
1 random_point() Write a method that returns a random point in the two-dimensional task space T = [0; self.width] ⨉ [0; self.height].
2 find_closest(point) Write a method that returns the closest vertex to point in the search tree.
3 new_point(random_point, closest_point) Write a method that returns the point belonging to the segment connecting closest_point and random_point located at a distance self.step from closest_point.
2.4 check_if_valid(point_1, point_2) Write a method that checks whether the segment connecting point_1 and point_2 is outside the task space T and whether it is in collision with an obstacle marked on self.map. You can do this in an approximate way by selecting, say, 100 equidistant points along this segment and checking that each of them lies within the free space.
2.5 search() Write a method that implements the following pseudocode
Initialize the search tree with the initial vertex self.start
Draw a vertex on the map
Find the vertex closest to it that belongs to the search tree
Create a new vertex that lies on the segment connecting the vertices defined in subitems 2 and 3 within the distance self.step from the vertex in subitem 3.
Check if the segment connecting the new point to the nearest vertex in the graph lies in free space
If so, add this new point to the search tree.
Display the search tree with self.publish_search()
Check if the new point can be connected to the end point self.end without collisions.
If so, add it to the tree, display the solution (method self.publish_path(path)) and terminate the algorithm.
If not, go back to step 2.


