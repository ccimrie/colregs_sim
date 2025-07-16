import numpy as np
import matplotlib.pyplot as plt
import os
import pickle

results_dir='../results'
out_dir="data"
vessel_breakdown_data_filename="vessel_breakdown_data.pickle"
data_filename="fleet_info.npz"
vessel_types=['agent0', 'pf normal', 'mass agent']
agent_types=[0,1,2]

def calculateNearMiss(data, tolerance=0.01):
  def checkCollision(vals):
    ego=vals[0]
    others=vals[1:,0:2]
    pos=ego[0:2]
    diff=others-pos
    dist=np.sqrt(diff[:,0]**2+diff[:,1]**2)-vals[0,3]-vals[1:,3]
    return np.sum(dist<tolerance)
  TT=len(data)
  agent_num=data.shape[1]
  near_collisions=[0]*TT
  for t in np.arange(TT):
    collisions=[checkCollision(data[t][v:]) for v in np.arange(agent_num)]
    near_collisions[t]=np.sum(collisions)
  return near_collisions


def loadData():
  data=np.load(f"{out_dir}/{data_filename}")['data']
  return data


def avgNeighCount(data):
  TT=len(data)
  results={}
  for agent_type in agent_types:
    results[agent_type]=[-1]*TT
  for t in np.arange(TT):
    for agent_type in agent_types:
      vals=[v[5] for v in data[t,:,:] if int(v[12])==agent_type]
      results[agent_type][t]=np.mean(vals)
  return results


def setAgentTypes(_agent_types):
  agent_types=_agent_types


def avgNeighDist(data):
  TT=len(data)
  results={}
  for agent_type in agent_types:
    results[agent_type]=[-1]*TT
  for t in np.arange(TT):
    for agent_type in agent_types:
      vals=[v[6] for v in data[t,:,:] if (int(v[12])==agent_type and v[5]>0)]
      results[agent_type][t]=np.mean(vals)
  return results


def storeData():
  TT=1
  features=13
  data=np.ones((TT,0,features), dtype=float)*-1
  files=os.listdir(results_dir)
  files=[file for file in files if file[-4:]=='.txt']
  for file in files:
    new_data=np.loadtxt(f"{results_dir}/{file}")
    start_time=int(new_data[0,11])
    end_time=int(new_data[-1,11])
    if end_time>TT:
      dims=data.shape
      data=np.append(data, np.ones([end_time+1-TT,dims[1],dims[2]])*-1, axis=0)
      TT=end_time+1
    data=np.append(data, np.ones([TT, 1, dims[2]])*-1, axis=1)
    data[start_time:end_time+1, -1, :]=new_data
  data=np.savez(f"{out_dir}/{data_filename}", data=data)




def getJourneyTimes(vals):
  def checkGoal(dist, tolerance):
    if dist[0]<tolerance[0] and dist[1]<tolerance[1]:
      return True
    else:
      return False
  dist_goal=abs(vals[:,7:10:2]-vals[:,0:2])
  goal_tolerance=vals[:,8:11:2]
  dist_reached=[checkGoal(dist, tolerance) for (dist, tolerance) in zip(dist_goal, goal_tolerance)]
  dist_reached[0]=True
  
  start_ind=0
  time_counter=0
  times=[]  
  for dr in dist_reached[1:]:
    if dr:
      times.append(time_counter)
      time_counter=0
    else:
      time_counter+=1 
  return times