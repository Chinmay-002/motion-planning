from typing import Tuple

import numpy as np
import pylab as pl

pl.ion()


class TriangularObstacle(object):
    def __init__(self, x0, y0, x1, y1, x2, y2):
        self.x0 = x0
        self.y0 = y0
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2

        self.A = np.zeros((3, 2))
        self.C = np.zeros(3)

        a = x1 - x0
        b = y1 - y0
        c = x2 - x0
        d = y2 - y0
        if -b * c + a * d > 0:
            self.A[0, :] = -b, a
        else:
            self.A[0, :] = b, -a
        self.C[0] = np.dot(self.A[0, :], np.array([x0, y0]))

        a = x2 - x1
        b = y2 - y1
        c = x0 - x1
        d = y0 - y1
        if -b * c + a * d > 0:
            self.A[1, :] = -b, a
        else:
            self.A[1, :] = b, -a
        self.C[1] = np.dot(self.A[1, :], np.array([x1, y1]))

        a = x0 - x2
        b = y0 - y2
        c = x1 - x2
        d = y1 - y2
        if -b * c + a * d > 0:
            self.A[2, :] = -b, a
        else:
            self.A[2, :] = b, -a
        self.C[2] = np.dot(self.A[2, :], np.array([x2, y2]))

    def contains(self, x, y):
        r = np.dot(self.A, np.array([x, y])) - self.C
        return all([i > 0 for i in r])

    def plot(self):
        pl.plot([self.x0, self.x1], [self.y0, self.y1], "r", linewidth=1)
        pl.plot([self.x1, self.x2], [self.y1, self.y2], "r", linewidth=1)
        pl.plot([self.x2, self.x0], [self.y2, self.y0], "r", linewidth=1)

    def is_line_intersecting(self, start_x: float, end_x: float, start_y: float, end_y: float) -> bool:
        """
        Checks if a given line intersects with the triangle
        """
        # converting all the lines to the form y = mx + c, and extracting the m and c parameters:

        # given line
        if end_x - start_x == 0:
            m = float('inf')
        else:
            m = (end_y - start_y) / (end_x - start_x)
        c = start_y - (m * start_x)

        # parameters m and c for side 1 of the triangle
        if self.x0 - self.x1 == 0:
            m1 = float('inf')
        else:
            m1 = (self.y0 - self.y1) / (self.x0 - self.x1)
        c1 = self.y0 - (m1 * self.x0)

        # parameters m and c for side 2 of the triangle
        if self.x1 - self.x2 == 0:
            m2 = float('inf')
        else:
            m2 = (self.y1 - self.y2) / (self.x1 - self.x2)
        c2 = self.y1 - (m2 * self.x1)

        # parameters m and c for side 3 of the triangle
        if self.x2 - self.x0 == 0:
            m3 = float('inf')
        else:
            m3 = (self.y2 - self.y0) / (self.x2 - self.x0)
        c3 = self.y2 - (m3 * self.x2)

        # checking for intersection between the given line and side 1 of the triangle
        try:
            a1 = np.array([[m, -1], [m1, -1]])
            b1 = np.array([-c, -c1])
            ans = np.linalg.solve(a1, b1)
            # if the line intersects with the side of the triangle, then the intersection point should be in the range
            # of the line and the side of the triangle
            if (inRange(start_x, ans[0], end_x) and inRange(start_y, ans[1], end_y)) and (
                    inRange(self.x0, ans[0], self.x1) and inRange(self.y0, ans[1], self.y1)):
                return True

        # if the lines are parallel (which means no intersection), then the above code will throw a LinAlgError
        except np.linalg.LinAlgError:
            pass

        # checking for intersection between the given line and side 2 of the triangle
        try:
            a2 = np.array([[m, -1], [m2, -1]])
            b2 = np.array([-c, -c2])
            ans = np.linalg.solve(a2, b2)
            # if the line intersects with the side of the triangle, then the intersection point should be in the range
            # of the line and the side of the triangle
            if (inRange(start_x, ans[0], end_x) and inRange(start_y, ans[1], end_y)) and (
                    inRange(self.x1, ans[0], self.x1) and inRange(self.y1, ans[1], self.y2)):
                return True

        # if the lines are parallel (which means no intersection), then the above code will throw a LinAlgError
        except np.linalg.LinAlgError:
            pass

        # checking for intersection between the given line and side 3 of the triangle
        try:
            a3 = np.array([[m, -1], [m3, -1]])
            b3 = np.array([-c, -c3])
            ans = np.linalg.solve(a3, b3)
            # if the line intersects with the side of the triangle, then the intersection point should be in the range
            # of the line and the side of the triangle
            if (inRange(start_x, ans[0], end_x) and inRange(start_y, ans[1], end_y)) and (
                    inRange(self.x2, ans[0], self.x0) and inRange(self.y2, ans[1], self.y0)):
                return True
        # if the lines are parallel (which means no intersection), then the above code will throw a LinAlgError
        except np.linalg.LinAlgError:
            pass

        # if the line does not intersect with any of the sides of the triangle, then the line is valid, hence return False
        return False


class Environment(object):
    def __init__(self, size_x, size_y, n_obs):
        self.size_x = size_x
        self.size_y = size_y
        self.obs = []
        for i in range(n_obs):
            x0 = np.random.rand() * size_x
            y0 = np.random.rand() * size_y
            x1 = np.random.rand() * size_x
            y1 = np.random.rand() * size_y
            x2 = np.random.rand() * size_x
            y2 = np.random.rand() * size_y
            self.obs.append(TriangularObstacle(x0, y0, x1, y1, x2, y2))

    def check_collision(self, x, y):
        for ob in self.obs:
            if ob.contains(x, y):
                return True
        return False

    def check_line_collision(self, start: Tuple[float, float], end: Tuple[float, float]) -> bool:
        """
        checks if the line segment between the start and end points intersects with any of the obstacles
        """

        # iterate through the obstacles
        for ob in self.obs:
            # check if the line segment intersects with the obstacle
            if ob.is_line_intersecting(start[0], end[0], start[1], end[1]):
                # if the line segment intersects with any obstacle, then return True
                return True
        # if the line segment does not intersect with any obstacle, then return False
        return False

    def random_query(self):
        max_attempts = 100
        found_start = False
        found_goal = False
        x_start, y_start, x_goal, y_goal = 0, 0, 0, 0
        for i in range(max_attempts):
            x_start = np.random.rand() * self.size_x
            y_start = np.random.rand() * self.size_y
            if not self.check_collision(x_start, y_start):
                found_start = True
                break
        for i in range(max_attempts):
            x_goal = np.random.rand() * self.size_x
            y_goal = np.random.rand() * self.size_y
            if not self.check_collision(x_goal, y_goal):
                found_goal = True
                break
        if found_start and found_goal:
            return x_start, y_start, x_goal, y_goal
        else:
            return None

    def plot(self):
        pl.plot([0, self.size_x, self.size_x, 0, 0], [0, 0, self.size_y, self.size_y, 0], "k", linewidth=2)
        for ob in self.obs:
            ob.plot()

    @staticmethod
    def plot_query(x_start, y_start, x_goal, y_goal):
        pl.plot([x_start], [y_start], "bs", markersize=8)
        pl.plot([x_goal], [y_goal], "y*", markersize=12)

    @staticmethod
    def draw_line(start: Tuple[float, float], end: Tuple[float, float], linewidth: int = 1, color: str = "") -> None:
        """
        plots a line between the start and end points
        """
        if color == "":
            pl.plot([start[0], end[0]], [start[1], end[1]], linewidth=linewidth)
        else:
            pl.plot([start[0], end[0]], [start[1], end[1]], color, linewidth=linewidth)


def inRange(x1: float, x: float, x2: float) -> bool:
    """
    checks if x is within range of x1 and x2
    """
    # if x1 is greater than x2, then swap them
    if x1 > x2:
        x1, x2 = x2, x1
    # return True if x is within range of x1 and x2, else False
    return x1 <= x <= x2
