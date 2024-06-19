#!/usr/bin/env bash
# $1 = log_name, $2 = costmap

echo '========== initial test =========='
rm -rf result/$1
run-many "python3 search_gridgoal.py --save_name $1 --costmap $2" -n 10 -j 10
python3 evaluate_route.py --save_name $1

echo '========== optimization =========='
python3 simulated_annealing.py --save_name $1 --costmap $2

echo '========== validation test =========='
rm -rf result/$1
run-many "python3 search_gridgoal.py --save_name $1 --costmap $2 --opt_param $1" -n 10 -j 10
python3 evaluate_route.py --save_name $1