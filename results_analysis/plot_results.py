import numpy as np
import matplotlib.pyplot as plt
import os
import distinctipy
from data_utils import agent_types
from data_utils import *
import time
import sys
import pandas as pd

# data_override=True if (len(sys.argv)>1 and sys.argv[1].lower() == 'true') else False

df=pd.read_csv('test_output/avg_neighbour_count.csv')

vessel_no=df.shape[0]
axes_count=[]
axes_dist=[]
counter=0
for i in np.arange(vessel_no):
  data=df.loc[i]
  print(data["Vessel type"])
  if len(data)>0:
    axes[int(counter/2),counter%2].plot(avg_neigh_count_data[agent_type], label=agent_types[agent_type], color=colours[agent_type])
    axes[int(counter/2),counter%2].set_ylabel("Avg num")
    axes[int(counter/2),counter%2].set_title(agent_types[agent_type])
    if int(counter/2)!=int(len(avg_neigh_count_data)/2):
      axes[int(counter/2),counter%2].set_xticklabels("")

    axes_dist.append(axes[int(counter/2),counter%2].twinx())
    axes_dist[counter].plot(avg_neigh_dist_data[agent_type], label=agent_types[agent_type], linestyle='--', color=colours[agent_type])
    axes_dist[counter].set_ylabel("Avg dist")
    counter+=1
  sys.exit()
colours=distinctipy.get_colors(len(agent_types))

axes_count=[]
axes_dist=[]
counter=0
for agent_type in avg_neigh_count_data:
  if len(avg_neigh_count_data)>0:
    axes[int(counter/2),counter%2].plot(avg_neigh_count_data[agent_type], label=agent_types[agent_type], color=colours[agent_type])
    axes[int(counter/2),counter%2].set_ylabel("Avg num")
    axes[int(counter/2),counter%2].set_title(agent_types[agent_type])
    if int(counter/2)!=int(len(avg_neigh_count_data)/2):
      axes[int(counter/2),counter%2].set_xticklabels("")

    axes_dist.append(axes[int(counter/2),counter%2].twinx())
    axes_dist[counter].plot(avg_neigh_dist_data[agent_type], label=agent_types[agent_type], linestyle='--', color=colours[agent_type])
    axes_dist[counter].set_ylabel("Avg dist")
    counter+=1

## Number of collisions/near misses
start_time=time.time()
collisions=nearMissMatrix(data, data_override=data_override)
print(collisions)
print(f"Acquired collision count in {np.round(time.time()-start_time,2)}s")
fig_collisions, ax_collisions=plt.subplots()
ax_total_collisions=ax_collisions.twinx()
ax_collisions.plot(collisions, label="number of collisions", linewidth=2.5)
collisions_total=np.cumsum(collisions)
ax_total_collisions.plot(collisions_total, label="accumulative collisions", linestyle='--', linewidth=2.5)
ax_collisions.legend()
ax_collisions.set_ylabel("Number of collisions at current time")
ax_total_collisions.set_ylabel("Accumulative number of collisions")
plt.show()