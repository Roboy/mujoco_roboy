#!/bin/bash
# run two processes in the background and wait for them to finish

python3 sim_plot.py 7500 200 200 100 --log 30 --plot --bag fixed_sp_allaxis 60 60 --addName compareRates
wait
python3 sim_plot.py 7500 200 200 200 --log 30 --plot --bag fixed_sp_allaxis 60 60 --addName compareRates
wait
python3 sim_plot.py 7500 200 200 400 --log 30 --plot --bag fixed_sp_allaxis 60 60 --addName compareRates
wait
python3 sim_plot.py 7500 200 200 400 --log 30 --plot --render 30 --bag fixed_sp_allaxis 60 60 --addName compareRatesRender
wait
python3 sim_plot.py 7500 200 200 800 --log 30 --plot --bag fixed_sp_allaxis 60 60 --addName compareRates
wait
python3 sim_plot.py 7500 200 200 1600 --log 30 --plot --bag fixed_sp_allaxis 60 60 --addName compareRates
wait
python3 sim_plot.py 7500 200 200 3200 --log 30 --plot --bag fixed_sp_allaxis 60 60 --addName compareRates
wait

