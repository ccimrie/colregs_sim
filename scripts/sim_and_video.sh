cd ${SIM_DIR_PATH}
source scripts/run_simulation.sh "$5"
printf "\n--Completed simulations; making video--\n"
cd ${SIM_DIR_PATH}/visualise_sim
source scripts/make_video.sh $1 $2 $3 $4
cd ${SIM_DIR_PATH}
echo "Video completed; saved as: ${SIM_DIR_PATH}/visualise_sim/output_video/$4.mp4"