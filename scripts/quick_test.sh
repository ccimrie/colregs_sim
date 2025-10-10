#!/bin/bash

cd ${SIM_DIR_PATH}/build
make
cd ${SIM_DIR_PATH}
source scripts/run_simulation.sh "$3"

cd ${SIM_DIR_PATH}/visualise_sim
python3 visualise.py $1 $2

cd ${SIM_DIR_PATH}