Random notes:

Install and setup
- Install Box2D
Set Box2D env variable:
`export BOX2D_ROOT_DIR=/path/to/box2d`

Build simulator:
`mkdir build && cd build && cmake -DBOX2D_ROOT_DIR=$BOX2D_ROOT_DIR ../`
`make`


Set the env variable for navigating to simulator directory:
`export SIM_DIR_PATH=/path/to/colregs_sim`

Test to see if simulator works:
`source scripts/run_simulation.sh`
You should be able to run this script from any location in terminal.

Config files can be found in **colregs_sim/yaml_files**
For setup look at **colregs_sim/yaml_files/setup/** and for individual agents **colregs_sim/yaml_files/agents/**

Developing code
- places to look
	- mass_agent.cpp...
	- normal_agent.cpp...

Analysing simulation
- to visualise results run scripts/visualise X Y
- to create video script/...
- to get analysis results...
