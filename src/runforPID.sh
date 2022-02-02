#!/bin/bash
# run two processes in the background and wait for them to finish

python3 PID.py 7500 10 750
wait
python3 PID.py 7500 100 750
wait
python3 PID.py 7500 200 250
wait
python3 PID.py 12500 250 250
wait
