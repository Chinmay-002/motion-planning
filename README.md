# motion-planning

* [About](https://github.com/Chinmay-002/motion-planning#about)

* [Working of the Algorithms](https://github.com/Chinmay-002/motion-planning#working-of-the-algorithms)
  - [Probabilistic Roadmap Algorithm (Path Planning)](https://github.com/Chinmay-002/motion-planning#probabilistic-roadmap-algorithm-path-planning)
  - [Path Shortcutting (Post Processing)](https://github.com/Chinmay-002/motion-planning#path-shortcutting-post-processing)

* [Running the Program on a Preset Environment](https://github.com/Chinmay-002/motion-planning#running-the-algorithm-on-a-preset-environment)
 
* [10000 Milestones (Example)](https://github.com/Chinmay-002/motion-planning#10000-milestones-example)
  - [Network](https://github.com/Chinmay-002/motion-planning#network)
  - [Dijkstra's Algorithm](https://github.com/Chinmay-002/motion-planning#dijkstras-algorithm)
  - [Post Processing Path-Shortcutting Algorithm](https://github.com/Chinmay-002/motion-planning#post-processing-path-shortcutting-algorithm)


## About

Probabilistic Roadmap Algorithms are used in robotics to find a collision-free path between a start point and an end point in a map with obstacles. After a collision-free path has been found, a post-processing algorithm like the Path Shortcutting algorithm can be used to find the shortest path between the start point and the destination from the sampled points.

## Working of the Algorithms

### Probabilistic Roadmap Algorithm (Path Planning)
* Generate n random samples points (milestones) within the free space.
* Create a NearestNeighbhor model (sklearn) fitted on the sample points.
* Link each milestone to k nearest neighbours if the link is valid and does not collide with the obstacles. 
* Form a graph consistingof the milestones and corresponding edges
* Find the shortest path in the graph created from the start point to the end point (Dijkstra's Algorithm).

### Path Shortcutting (Post Processing)
* From the solution path found through the path planning algorithm, create a graph from the start point to the destination point.
* Using the created graph, pick 2 random nodes and check if the edge created by joining them is valid
* If the edge created is valid, connect them and update the graph
* Reapeat the process max_rep times

## Running the Program on a Preset Environment

The environment and the obstacles are preset. This is done by using the np.random.seed() function, which enables the random functions to generate the same pseuodo-random values each time the program is run. This sets up the environment in the same way each time the program is run by setting the attributes of the obstacles, start and point to be the same for every run.

If 4 is passed as an argument to the seed() function, as it has been in the program, the environment set-up looks like this:

![environment](https://github.com/Chinmay-002/motion-planning/blob/main/images/environment.png)

## 10000 Milestones (Example)

### Network 

When the program is run with the number of sample points set as 10000, the network formed by the sample points looks like this:

![10000 Milestones Network](https://github.com/Chinmay-002/motion-planning/blob/main/images/network_10000.png)

### Dijkstra's Algorithm

After running Dijkstra's Algorithm on the network, the algorithm returns a solution of the shortest path from the starting coordinates to the destination coordinates. The plotted solution is: 

![10000 Milestones Dijkstra's shortest path](https://github.com/Chinmay-002/motion-planning/blob/main/images/prm_10000.png)

### Post Processing Path-Shortcutting Algorithm

The Path Shortcutting algorithm is then run, and it returns the shortest path that can be formed form all the nodes in the solution graph. This plotted solution is: 

![10000 Milestones Post-Processed Shortest path](https://github.com/Chinmay-002/motion-planning/blob/main/images/post_processed_10000.png)




