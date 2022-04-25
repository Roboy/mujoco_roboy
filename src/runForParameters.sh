#!/bin/bash
# run two processes in the background and wait for them to finish




python3 simulation.py 14100 68737 723 400 --log 30 --plot --bag shoulder_axis0_left_elbow_0_I 300 360 --ctrlOnly shoulder_left_axis0 --folder TestAx0 --model model_nosidesite.xml --render 60



wait
sleep 3

python3 simulation.py 14100 68737 723 400 --log 30 --plot --bag shoulder_axis0_left_elbow_0_I 300 360 --ctrlOnly shoulder_left_axis0 --folder TestAx0 --model model_nosidesite.xml --render 60



wait
sleep 3


printf '\7'
sleep 1
printf '\7'
sleep 1
printf '\7'

