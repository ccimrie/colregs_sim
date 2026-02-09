import numpy as np
import os
from te_analysis import TECalcEst
import matplotlib.pyplot as plt


results_dir='../results'
files=os.listdir(results_dir)
files=[file for file in files if file[-4:]=='.txt']

max_vals=[]


agent_info={}

TT=0
def convertRow(val_in):
  # global current_colour_ind
  val=val_in.decode()
  try:
      float(val)
      return float(val)
  except ValueError:
      # if val not in agent_types:
      #     val_new=current_colour_ind
      #     current_colour_ind+=1
      #     agent_types[val]=int(val_new)
      return 0.0

## Get all data
for file in files:
  results=np.loadtxt(f'{results_dir}/{file}',converters=convertRow)
  pose=results[:,0:3]
  velocity=results[:,-4:-1]
  # print(f"Shapes: {np.shape(pose)=}  {np.shape(velocity)}")
  agent_info[file]=np.hstack([pose,velocity])
  TT=len(agent_info[file])
  # print(np.shape(agent_info[file]))

## Get TE across time for all agents
r=10.0 ## Could be smarter and use perception ranges
r_square=r**2
interact_dict={} ## 3D array: x, x_past, y_past

def resetDict():
  global interact_dict
  for neighbour in agent_info:
    interact_dict[neighbour]=np.empty((0,3,3), dtype=np.float32)

for agent in files:
  TE=0.0
  resetDict()

  def getTE(agent, neighbour,t):
    ## Check distance
    dist=np.dot(agent_info[agent][t,0:2]-agent_info[neighbour][t,0:2])
    if dist>r_square:
      return 0
    ## Add new interaction
    agent_vel=agent_info[agent][t+1,0:2]
    agent_past_vel=agent_info[agent][t,0:2]
    neigh_past_vel=agent_info[neighbour][t,0:2]

    data_point=np.array()
    interact_dict[neighbour]

    return

  for t in np.arange(TT-1):
    for neighbour in agent_info:
      if neighbour!=agent:
        TE=getTE(agent, neighbour,t)


