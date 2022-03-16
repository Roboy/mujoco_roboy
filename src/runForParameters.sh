#!/bin/bash
# run two processes in the background and wait for them to finish

python3 simulation.py 24000 0 0 400 --log 10 --plot --bag shoulder_axis0_left_elbow_0_G 8 0 --ctrlOnly shoulder_left_axis0 --folder TuneShoulderP_axis0 --model model_nosidesite.xml --render 60
wait
sleep 3






printf '\7'
sleep 1
printf '\7'
sleep 1
printf '\7'

