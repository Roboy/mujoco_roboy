#!/bin/bash
# run two processes in the background and wait for them to finish

python3 alonas_wish.py 12500 250 250
wait
python3 alonas_wish.py 7500 200 250
