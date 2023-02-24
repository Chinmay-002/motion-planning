import random
from typing import Tuple

import numpy as np
from sklearn.neighbors import NearestNeighbors

from environment import Environment


def find_closest_node(nodes: set, distance_dict: dict) -> Tuple[float, float]:
    """
    finds the closest node to the current node
    """
    # set the closest node to None
    closest_node = None

    # iterate through the nodes
    for node in nodes:

        # if the closest node is None, set it to the current node
        if closest_node is None:
            closest_node = node

        # if the distance to the current node is less than the distance to the closest node,
        # set the closest node to the current node
        elif distance_dict[node] < distance_dict[closest_node]:
            closest_node = node

    # return the closest node
    return closest_node


class Graph:
    """
    A class used to represent a Graph of nodes (coordinates) and edges between nodes
    """

    def __init__(self):
        """
        Constructor for the Graph class, initializes the node set and edge dictionary
        """
        self.nodes = set()
        self.edges = dict()

    def add_node(self, node: Tuple[float, float]) -> None:
        """
        Adds a node to the graph
        """
        self.nodes.add(node)

    def add_edge(self, node1: Tuple[float, float], node2: Tuple[float, float], distance: float) -> None:
        """
        creates an edge between two nodes and adds it to the graph
        """
        edge = Edge(node1, node2, distance)

        # add a connection between node 1 and node 2
        if node1 in self.edges:
            node1_edges = self.edges[node1]
        else:
            self.edges[node1] = dict()
            node1_edges = self.edges[node1]
        node1_edges[node2] = edge

        # add a connection between node 2 and node 1
        if node2 in self.edges:
            node2_edges = self.edges[node2]
        else:
            self.edges[node2] = dict()
            node2_edges = self.edges[node2]
        node2_edges[node1] = edge

    def dijkstra(self, source: Tuple[float, float], destination: Tuple[float, float]) -> (dict, dict):
        """
        finds the shortest path between the source node and the destination node in the graph
        """
        # initialize the distance and previous dictionaries
        distances = dict()
        previous = dict()

        # iterate through all nodes in the graph and initialize their distances to infinity and predecessors to None
        for node in self.nodes:
            distances[node] = float('inf')
            previous[node] = None
        distances[source] = 0

        # create a copy of the nodes set
        nodes = self.nodes.copy()

        # while the nodes set is not empty
        while nodes:

            # find the node with the smallest distance in the set
            min_node = find_closest_node(nodes, distances)

            # if the distance to the node is infinity (unreachable node) or the node is the destination, break the loop
            if distances[min_node] == float('inf') or min_node == destination:
                break

            # remove the node from the set
            nodes.remove(min_node)

            # find the distance to the min node from the source
            current_weight = distances[min_node]

            # iterate through all edges connected to the min node
            for edge in self.edges[min_node]:

                # the distance to the new node is equal to the current distance to the min node plus the distance
                # between the min node and the new node
                weight = current_weight + self.edges[min_node][edge].distance

                # if the new distance is less than the current distance to the new node, update the distance and the predecessor of the node
                if weight < distances[edge]:
                    distances[edge] = weight
                    previous[edge] = min_node

        # return the distance and predecessor dictionaries after iterating through all nodes
        return distances, previous

    def get_random_nodes(self) -> (Tuple[float, float], Tuple[float, float]):
        """
        returns two random unique nodes from the graph
        """
        # create a list of all nodes in the graph
        nodes = list(self.nodes)

        # choose two random nodes from the list
        node1 = random.choice(nodes)
        node2 = random.choice(nodes)

        # if the two nodes are the same, choose a new node for node2
        while node2 == node1:
            node2 = random.choice(nodes)

        # return the two nodes
        return node1, node2

    def does_edge_exist(self, node1: Tuple[float, float], node2: Tuple[float, float]) -> bool:
        """
        checks if there is an edge between two nodes
        """
        # if node1 is in the edges dictionary
        if node1 in self.edges:

            # if node2 is in the edges dictionary for node1, return true
            if node2 in self.edges[node1]:
                return True

        # There is no edge between the two nodes, return false
        return False

    @staticmethod
    def get_solution_path(previous: dict, destination: Tuple[float, float]) -> list:
        """
        returns the path from the source node to the destination node
        """
        # initialize the path list
        path = []

        # iterate through the previous dictionary, starting from the destination and add the nodes to the path list
        while previous[destination] is not None:
            path.append(destination)
            destination = previous[destination]

        # add the source node to the path list
        path.append(destination)

        # return reverse of the path list
        return path[::-1]


class Edge:
    """
    A class used to represent an edge between two nodes
    """

    def __init__(self, node1: Tuple[float, float], node2: Tuple[float, float], distance: float):
        """
        Constructor for the Edge class, initializes the two nodes that the edge connects and the distance between them.
        """
        self.node1 = node1
        self.node2 = node2
        self.distance = distance


class PRM:
    """
    A class used to represent a Probabilistic Road Map
    """

    def __init__(self, environment: Environment, start: Tuple[float, float], destination: Tuple[float, float],
                 n_samples: int = 1000):
        """
        Constructor for the PRM class, initializes the environment, number of samples, graph, start node, and destination node
        """

        self.environment = environment
        self.n_samples = n_samples
        self.graph = Graph()
        self.start = start
        self.destination = destination
        self.samples = []

    def generate_samples(self) -> None:
        """
        generates random sample nodes in the environment which are in the free space and adds them to the sample list
        """
        # create a list of samples, including the start and destination nodes
        samples = [self.start, self.destination]

        # generate n_samples random samples within the range of the environment
        i = 0
        while i < self.n_samples:
            x = np.random.rand() * self.environment.size_x
            y = np.random.rand() * self.environment.size_y

            # if the sample is in the free space, add it to the sample list
            if not self.environment.check_collision(x, y):
                samples.append((x, y))
                i += 1

        # set the sample list to the samples attribute of the instance
        self.samples = samples

    def link_nearest_neighbors(self, num_neighbors: int = 15, radius: int = 0.3) -> None:
        """
        links the nearest neighbors of each node in the graph
        """
        # get a numpy array of the sample nodes
        points = np.array(self.samples)

        # create a nearest neighbors model with the number of neighbors and radius
        model = NearestNeighbors(n_neighbors=num_neighbors, radius=radius)

        # fit the model to the sample nodes
        model.fit(points)

        # get the distances and indices of the nearest neighbors for each node
        distances, indices = model.kneighbors(points)

        # iterate through the indices and distances for each node
        for index, coordinates in enumerate(points):

            # iterate through all the neighbors of the node
            for j, neighbor in enumerate(points[indices[index][1:]]):
                start = tuple(coordinates)
                end = tuple(neighbor)

                # if the edge does not already exist in the graph and there is no collision between the two nodes,
                # add the edge to the graph
                if not self.environment.check_line_collision(start, end):
                    self.graph.add_node(start)
                    self.graph.add_node(end)
                    self.graph.add_edge(start, end, distances[index][j + 1])

    def find_shortest_path(self) -> list:
        """
        finds  and plots the shortest path from the start node to the destination node using Dijkstra's algorithm
        """
        # get the distances and predecessors from the start node to all other nodes in the graph from Dijkstra's algorithm
        distances, previous = self.graph.dijkstra(self.start, self.destination)

        # get the path from Dijsktra's algorithm's solution
        path = self.graph.get_solution_path(previous, self.destination)

        # plot the path
        for i in range(len(path) - 1):
            self.environment.draw_line(path[i], path[i + 1], linewidth=1)

        # return the path
        return path

    def executePRM(self) -> list:
        """
        executes the PRM algorithm by calling relevant methods
        """
        # generate samples
        self.generate_samples()

        # link nearest neighbors
        self.link_nearest_neighbors()

        # find the shortest path from the graph
        path = self.find_shortest_path()

        # return the path
        return path
