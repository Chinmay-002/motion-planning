from network import *


class PathShortCutting:
    """
    A class used to represent Path Short-Cutting algorithms
    """

    def __init__(self, path: list, environment: Environment, maxrep: int = 100):
        """
        initialize the Path Short-Cutting algorithm by creating the solution graph, the environment,
        and the maximum number of iterations
        """
        self.maxrep = maxrep
        self.environment = environment
        self.path = path
        self.graph = Graph()

        # add the nodes from the solution path and corresponding edges to the graph
        for i in range(len(path) - 1):
            self.graph.add_node(path[i])
            self.graph.add_edge(path[i], path[i + 1], PathShortCutting.find_distance(path[i], path[i + 1]))
        self.graph.add_node(path[-1])

    def execute_path_short_cutting(self) -> list:
        """
        executes the Path Short-Cutting algorithm
        """
        # for maxrep iterations
        for i in range(self.maxrep):

            # if the graph has more than 2 nodes
            if len(self.graph.nodes) > 2:

                # get two random nodes from the graph
                node1, node2 = self.graph.get_random_nodes()

                # if the edge between the two nodes exists already, continue
                if self.graph.does_edge_exist(node1, node2):
                    continue

                # if the edge between the two nodes does not exist, check if the edge is collision free
                else:
                    if not self.environment.check_line_collision(node1, node2):

                        # if the edge is collision free, remove the nodes in between node1 and node2
                        new_path = self.path.copy()
                        index1 = self.path.index(node1)
                        index2 = self.path.index(node2)
                        if index1 > index2:
                            index1, index2, node1, node2 = index2, index1, node2, node1
                        j = index1 + 1
                        while j < index2:
                            new_path.remove(self.path[j])
                            j += 1

                        # reconstruct the graph from the new path
                        self.path = new_path
                        self.graph = Graph()
                        # add the nodes from the solution path and corresponding edges to the graph
                        for j in range(len(new_path) - 1):
                            self.graph.add_node(new_path[j])
                            self.graph.add_edge(new_path[j], new_path[j + 1],
                                                PathShortCutting.find_distance(new_path[j], new_path[j + 1]))
                        self.graph.add_node(new_path[-1])

        # plot the new path
        for i in range(len(self.path) - 1):
            self.environment.draw_line(self.path[i], self.path[i + 1], linewidth=1)

        # `return the new path
        return self.path

    @staticmethod
    def find_distance(start: Tuple[float, float], end: Tuple[float, float]) -> float:
        """
        finds the distance between two points
        """
        return np.sqrt((start[0] - end[0]) ** 2 + (start[1] - end[1]) ** 2)
