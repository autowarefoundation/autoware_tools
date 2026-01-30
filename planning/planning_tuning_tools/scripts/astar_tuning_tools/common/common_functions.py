from config.astar_params import astar_param
from config.astar_params import planner_param
import numpy as np


def param_vector2obj(vector):
    planner_param.curve_weight = vector[0]
    planner_param.reverse_weight = vector[1]

    # -- A* search Configurations --
    astar_param.distance_heuristic_weight = vector[2]

    return planner_param, astar_param


def param_obj2vector(planner_param_, astar_param_):
    vector = np.zeros(7)
    vector[0] = planner_param_.curve_weight
    vector[1] = planner_param_.reverse_weight

    # -- A* search Configurations --
    vector[2] = astar_param_.distance_heuristic_weight
    return vector
