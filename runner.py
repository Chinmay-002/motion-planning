import sys
import numpy as np
import pylab as pl
import environment
from network import PRM
from post_processing import PathShortCutting

sys.path.append('/')

pl.ion()
np.random.seed(4)
env = environment_2d.Environment(10, 6, 5)
pl.clf()
env.plot()
q = env.random_query()

if q is not None:
    x_start, y_start, x_goal, y_goal = q
    env.plot_query(x_start, y_start, x_goal, y_goal)

    # Instantiate PRM
    prm = PRM(env, (x_start, y_start), (x_goal, y_goal), n_samples=10000)

    # Execute PRM
    path = prm.executePRM()

    # save and display the prm plot
    pl.savefig('images/prm_10000.png')
    pl.show()

    # plot the environment fopr the path-cutting algorithm
    env.plot_query(x_start, y_start, x_goal, y_goal)
    env.plot()

    # Instantiate PathShortCutting
    ps = PathShortCutting(path, env, maxrep=200)

    # Execute PathShortCutting
    path2 = ps.execute_path_short_cutting()

    # save and display the best path plot
    pl.savefig('images/post_processed_10000.png')
    pl.show()
