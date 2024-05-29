import numpy as np
from param.astar_params import astar_param
from param.astar_params import planner_param

def param_vector2obj(vector):
    # planner_param.theta_size = int(vector[0])
    # planner_param.angle_goal_range = vector[1]
    planner_param.curve_weight = vector[0]
    planner_param.reverse_weight = vector[1]
    # planner_param.lateral_goal_range = vector[4]
    # planner_param.longitudinal_goal_range = vector[5]

    # -- A* search Configurations --
    astar_param.distance_heuristic_weight = vector[2]

    return planner_param, astar_param

def param_obj2vector(planner_param_, astar_param_):
    vector = np.zeros(7)
    vector[0] = planner_param_.theta_size
    vector[1] = planner_param_.angle_goal_range
    vector[2] = planner_param_.curve_weight
    vector[3] = planner_param_.reverse_weight
    vector[4] = planner_param_.lateral_goal_range
    vector[5] = planner_param_.longitudinal_goal_range

    # -- A* search Configurations --
    vector[6] = astar_param_.distance_heuristic_weight 
    return vector

