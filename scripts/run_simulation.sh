#!/bin/bash
YAML_FILE=setup_hexagonal.yaml
if [ -n "$1" ] && [ $1=="lanes" ]; then
	YAML_FILE=setup_lane.yaml
	echo "Using lanes"
else
	echo "Not using lanes"
fi

cd ${SIM_DIR_PATH}
rm results/*
cd build
./test ${SIM_DIR_PATH}/yaml_files/setup/${YAML_FILE}
cd ${SIM_DIR_PATH}