import autoware_freespace_planning_algorithms.astar_search as fp

# -- Vehicle Shape --
vehicle_shape = fp.VehicleShape()
vehicle_shape.length = 4.89
vehicle_shape.width = 1.896
vehicle_shape.base2back = 1.1


# -- Planner Common Parameter --
planner_param = fp.PlannerCommonParam()
# base configs
planner_param.time_limit = 30000.0
planner_param.max_turning_ratio = 0.5
planner_param.turning_steps = 1
# search configs
planner_param.theta_size = 144
planner_param.angle_goal_range = 6.0
planner_param.lateral_goal_range = 0.5
planner_param.longitudinal_goal_range = 2.0
planner_param.curve_weight = 0.5
planner_param.reverse_weight = 1.0
planner_param.direction_change_weight = 1.5
# costmap configs
planner_param.obstacle_threshold = 100


# -- A* search Configurations --
astar_param = fp.AstarParam()
astar_param.search_method = "forward"  # options: forward, backward
astar_param.only_behind_solutions = False
astar_param.use_back = True
astar_param.adapt_expansion_distance = True
astar_param.expansion_distance = 0.5
astar_param.distance_heuristic_weight = 1.5
astar_param.smoothness_weight = 0.5
astar_param.obstacle_distance_weight = 1.5
