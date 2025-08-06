import numpy as np
import matplotlib.pyplot as plt
import os
import distinctipy
from data_utils import agent_types
from data_utils import *
import time
import sys

# vessel_types=['COLREGS', 'non-COLREGS', 'MASS']



data_override=False

if len(sys.argv)>1 and sys.argv[1]:
  data_override=True

## Loading and storing data
start_time=time.time()
storeData(data_override=data_override)
print(f"Stored data in {np.round(time.time()-start_time,2)}s")
start_time=time.time()
data, agent_types=loadData()
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
  journey_times[agent_type]=[np.mean(journey_times[agent_type]), np.std(journey_times[agent_type])]
print(f"Acquired journey times in {np.round(time.time()-start_time,2)}s")
for agent_type in agent_types:
  print(f"\t-{agent_types[int(agent_type)]}: {journey_times[agent_type][0]} (+/- {journey_times[agent_type][1]})")


## Average neighbour number and distance
start_time=time.time()
no_agents=len(agent_types)
ncols=1
nrows=1
if no_agents>1:
  ncols=2
  nrows=int(np.ceil(no_agents/float(ncols)))
print(ncols, nrows)
fig, axes=plt.subplots(ncols=ncols, nrows=nrows)
avg_neigh_count_data=avgNeighCount(data, data_override=data_override)
avg_neigh_dist_data=avgNeighDist(data, data_override=data_override)
print(f"Acquired average neighbours in {np.round(time.time()-start_time,2)}s")
colours=distinctipy.get_colors(len(agent_types))

axes_count=[]
axes_dist=[]
counter=0
for agent_type in avg_neigh_count_data:
  if len(avg_neigh_count_data)>0:
    # fig.add_subplot(gs[counter, -1])
    # axes_count.append(fig.add_subplot(1, 1,(counter,counter%2)))
    # axes_count[counter].plot(avg_neigh_count_data[agent_type], label=agent_types[agent_type], color=colours[agent_type])
    # axes_count[counter].set_ylabel("Average number of neighbours")
    # axes_count[counter].set_title(agent_types[agent_type])
    # axes_count.append(fig.add_subplot(1, 1,(counter,counter%2)))
    axes[int(counter/2),counter%2].plot(avg_neigh_count_data[agent_type], label=agent_types[agent_type], color=colours[agent_type])
    axes[int(counter/2),counter%2].set_ylabel("Avg num")
    axes[int(counter/2),counter%2].set_title(agent_types[agent_type])
    if int(counter/2)!=int(len(avg_neigh_count_data)/2):
      axes[int(counter/2),counter%2].set_xticklabels("")

    # axes_dist.append(axes_count[-1].twinx())
    # axes_dist[counter].plot(avg_neigh_dist_data[agent_type], label=agent_types[agent_type], linestyle='--', color=colours[agent_type])
    # axes_dist[counter].set_ylabel("Average distance of neighbours")
    axes_dist.append(axes[int(counter/2),counter%2].twinx())
    axes_dist[counter].plot(avg_neigh_dist_data[agent_type], label=agent_types[agent_type], linestyle='--', color=colours[agent_type])
    axes_dist[counter].set_ylabel("Avg dist")
    # axes_count[-1].set_ylabel("Average distance of neighbours")
# axes_count[0].legend()
    counter+=1

## Number of collisions/near misses
start_time=time.time()
collisions=calculateNearMiss(data, data_override=data_override)
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