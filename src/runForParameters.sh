#!/bin/bash
# run two processes in the background and wait for them to finish

python3 simulation.py 40000 0 0 400 --log 30 --plot --bag elbow_left_D 10 0 --ctrlOnly elbow_left --folder TuneLeftElbowP
wait
sleep 5

python3 simulation.py 50000 0 0 400 --log 30 --plot --bag elbow_left_D 10 0 --ctrlOnly elbow_left --folder TuneLeftElbowP
wait
sleep 5

python3 simulation.py 60000 0 0 400 --log 30 --plot --bag elbow_left_D 10 0 --ctrlOnly elbow_left --folder TuneLeftElbowP
wait
sleep 5


