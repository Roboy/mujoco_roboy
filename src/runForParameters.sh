#!/bin/bash
# run two processes in the background and wait for them to finish


python3 simulation.py 17500 0 0 400 --log 60 --plot --bag shoulder_axis1_left_elbow_0_I 310 30 --ctrlOnly shoulder_left_axis1 --folder TuneShoulderP_axis1 --model model_nosidesite.xml





wait
sleep 3

printf '\7'
sleep 1
printf '\7'
sleep 1
printf '\7'

