#!/bin/bash
# run two processes in the background and wait for them to finish

python3 parallelPID.py 7500 10 750
wait
python3 parallelPID.py 7500 100 750
wait
python3 parallelPID.py 12500 250 250
wait
