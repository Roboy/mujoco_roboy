#!/bin/bash
# run two processes in the background and wait for them to finish

python3 PID.py 10000 0 0
wait
python3 PID.py 15000 0 0
wait
python3 PID.py 20000 0 0
wait
python3 PID.py 25000 0 0
wait
