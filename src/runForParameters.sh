#!/bin/bash
# run two processes in the background and wait for them to finish

python3 simulation.py 20000 0 0 800 --log 30 --plot --bag elbow_left_c 35 95 --render 60 --ctrlOnly elbow_left --folder TuneLeftElbowP
wait
sleep 5

python3 simulation.py 40000 0 0 800 --log 30 --plot --bag elbow_left_c 35 95 --render 60 --ctrlOnly elbow_left --folder TuneLeftElbowP
wait
sleep 5

python3 simulation.py 80000 0 0 800 --log 30 --plot --bag elbow_left_c 35 95 --render 60 --ctrlOnly elbow_left --folder TuneLeftElbowP
wait
sleep 5

python3 simulation.py 160000 0 0 800 --log 30 --plot --bag elbow_left_c 35 95 --render 60 --ctrlOnly elbow_left --folder TuneLeftElbowP
wait
sleep 5


