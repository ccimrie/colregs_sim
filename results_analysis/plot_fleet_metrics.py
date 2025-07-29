import numpy as np
import matplotlib.pyplot as plt
import os
import distinctipy
from data_utils import *
import time
import sys

# vessel_types=['COLREGS', 'non-COLREGS', 'MASS']

## Loading and storing data
start_time=time.time()
storeData()
print(f"Stored data in {np.round(time.time()-start_time,2)}s")
start_time=time.time()
data=loadData()
print(f"Loaded data in {np.round(time.time()-start_time,2)}s")

# setAgentTypes(['bulker', 'container ship', 'cruise' 'car carrier'])

## Average journey time
start_time=time.time()
journey_times={}
dims=data.shape
for agent in np.arange(dims[1]):
  _journey_times=getJourneyTimes(data[:,agent,:])
  agent_type=data[0,agent,-1]
  if agent_type not in journey_times:
    journey_times[agent_type]=[]
  for _time in _journey_times:
    journey_times[agent_type].append(_time)
for agent_type in journey_times:
  journey_times[agent_type]=np.mean(journey_times[agent_type])
print(journey_times)
print(f"Acquired journey times in {np.round(time.time()-start_time,2)}s")


## Average neighbour number and distance
start_time=time.time()
avg_neigh_count_data=avgNeighCount(data)
avg_neigh_dist_data=avgNeighDist(data)
print(f"Acquired average neighbours in {np.round(time.time()-start_time,2)}s")
start_time=time.time()
collisions=calculateNearMiss(data)
print(f"Acquired collision count in {np.round(time.time()-start_time,2)}s")
fig, ax_neigh_count=plt.subplots()
ax_neigh_dist=ax_neigh_count.twinx()
# colours=distinctipy.get_colors(len(vessel_types))

for agent_type in avg_neigh_count_data:
  if len(avg_neigh_count_data)>0:
    ax_neigh_count.plot(avg_neigh_count_data[agent_type], label=agent_types[agent_type])
    ax_neigh_dist.plot(avg_neigh_dist_data[agent_type], label=agent_types[agent_type], linestyle='--')
ax_neigh_count.legend()
ax_neigh_count.set_ylabel("Average number of neighbours")
ax_neigh_dist.set_ylabel("Average distance of neighbours")

## Number of collisions/near misses
fig_collisions, ax_collisions=plt.subplots()
ax_total_collisions=ax_collisions.twinx()
ax_collisions.plot(collisions, label="number of collisions", linewidth=2.5)
collisions_total=np.cumsum(collisions)
ax_total_collisions.plot(collisions_total, label="accumulative collisions", linestyle='--', linewidth=2.5)
ax_collisions.legend()
ax_collisions.set_ylabel("Number of collisions at current time")
ax_total_collisions.set_ylabel("Accumulative number of collisions")
plt.show()