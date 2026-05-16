#!/usr/bin/env bash
# $1 = costmap, $2 = log_name

COSTMAP=${1:-costmap_default}
SAVENAME=${2:-test}

echo '========== initial test =========='
rm -rf result/$SAVENAME
run-many "python3 search.py --save_name $SAVENAME --costmap $COSTMAP" -n 10 -j 10
python3 evaluator.py --save_name $SAVENAME

echo '========== optimization =========='
python3 simulated_annealing.py --save_name $SAVENAME --costmap $COSTMAP

echo '========== validation test =========='
rm -rf result/$SAVENAME
run-many "python3 search.py --save_name $SAVENAME --costmap $COSTMAP --opt_param $SAVENAME" -n 10 -j 10
python3 evaluator.py --save_name $SAVENAME
