import numpy as np
import matplotlib.pyplot as plt
import os
from data_utils import agent_types
from data_utils import *
# import time
import sys
import csv

data_override=True if (len(sys.argv)>1 and sys.argv[1].lower() == 'true') else False
# print(data_override)

results_dict={}

## Loading and storing data
storeData(data_override=data_override)
print(f"Stored data")
data, agent_types=loadData()
print(f"Loaded data")

## Average journey time
journey_times={}
avg_journey_times={}
std_journey_times={}
dims=data.shape
for agent in np.arange(dims[1]):
  _journey_times=getJourneyTimes(data[:,agent,:])
  agent_type=data[0,agent,-1]
  if agent_type not in journey_times:
    journey_times[agent_type]=[]
  for _time in _journey_times:
    journey_times[agent_type].append(_time)
for agent_type in journey_times:
  avg_journey_times[agent_type]=np.mean(journey_times[agent_type])
  std_journey_times[agent_type]=np.std(journey_times[agent_type])

results_dict['Average journey time']=avg_journey_times
results_dict['Std of journey time']=std_journey_times
print(f"Acquired journey times")

## Average neighbour number and distance
neighbours_count_dict={}
neighbours_dist_dict={}
neighbours_count_dict['Average number of visible neighbours']=avgNeighCount(data, data_override=data_override)
neighbours_dist_dict['Average distance of visible neighbours']=avgNeighDist(data, data_override=data_override)

avg_neigh_count={}
avg_neigh_dist={}
for agent_type in agent_types:
  avg_neigh_count[agent_type]=np.mean(neighbours_count_dict['Average number of visible neighbours'][agent_type])
  avg_neigh_dist[agent_type]=np.mean(neighbours_dist_dict['Average distance of visible neighbours'][agent_type])

for agent_type in agent_types:
  results_dict['Average number of visible neighbours']=avg_neigh_count
  results_dict['Average distance of visible neighbours']=avg_neigh_dist

## Number of collisions/near misses
collisions_dict={}
near_miss_mat_dict={}
for agent_type in agent_types:
  near_miss_mat_dict[agent_type]={}
  for neigh_agent_type in agent_types:
    near_miss_mat_dict[agent_type][neigh_agent_type]=0
# collisions=calculateNearMiss(data, data_override=data_override)
# collisions_total=np.cumsum(collisions)
near_miss_mat=nearMissMatrix(data, data_override=data_override)
print(near_miss_mat)
for agent_type in agent_types:
  collisions_dict[agent_type]=np.sum(near_miss_mat[agent_type])
  for collision in np.arange(len(near_miss_mat[agent_type])):
    near_miss_mat_dict[agent_type][collision]+=near_miss_mat[agent_type][collision]
results_dict['Near misses']=collisions_dict
print(results_dict)

print(f"Acquired collision count")

def writeCSV(filename, data_dict, attributes):
 ## Create CSV files
  csv_output=open(filename,'w', newline='')
  csv_writer = csv.writer(csv_output, delimiter=',',
                          quotechar='|', quoting=csv.QUOTE_MINIMAL)
 ## Write attributes to file
  csv_writer.writerow(attributes)
 ## Write data to file
  for agent_type in agent_types:
    results_row=[agent_types[agent_type]]
    for field in data_dict:
      results_row.append(data_dict[field][agent_type])
    csv_writer.writerow(results_row)

## Print and save general results to csv file
attr_str=['Vessel type']
for field in results_dict:
  attr_str.append(field)
writeCSV('test_output/results_general.csv', results_dict, attr_str)

## Write and save neighbour information to csv file
attr_str=['Vessel type']+[str(t) for t in np.arange(len(data))]
writeCSV('test_output/avg_neighbour_count.csv', neighbours_count_dict, attr_str)
writeCSV('test_output/avg_neighbour_dist.csv', neighbours_dist_dict, attr_str)

## Write collision information
attr_str=['Vessel type']
for agent_type in agent_types:
  attr_str.append(agent_types[agent_type])
writeCSV('test_output/near_miss.csv', near_miss_mat_dict, attr_str)