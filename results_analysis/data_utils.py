import numpy as np
import matplotlib.pyplot as plt
import os
import pickle

results_dir='../results'
out_dir="data"
agent_types_pickle_name="agent_types.pickle"
data_filename="fleet_info.npz"

neigh_dist_data_filename="neigh_dist_info.pickle"
neigh_count_data_filename="neigh_count_info.pickle"

near_miss_data_filename="near_miss_info.npz"
near_miss_matrix_data_filename="near_miss_matrix.npz"
current_agent_ind=0
vessel_types=[]
agent_types={}

def calcTranfserEntr(data):
  return


def nearMissMatrix(data, tolerance=0.001, data_override=False):
  if not data_override and os.path.exists(f"{out_dir}/{near_miss_matrix_data_filename}"):
    results=np.load(f"{out_dir}/{near_miss_matrix_data_filename}")['near_miss_matrix']
    return results
  def checkCollision(vals, mat):
    ego=vals[0]
    v_type=int(ego[-1])
    others=vals[1:,0:2]
    pos=ego[0:2]
    diff=others-pos
    dist=np.sqrt(diff[:,0]**2+diff[:,1]**2)-vals[0,3]-vals[1:,3]
    
    for ind in np.arange(len(dist)):
      if dist[ind]<tolerance:
        mat[v_type, int(vals[ind,-1])]+=1
        mat[int(vals[ind,-1]), v_type]+=1
    ## Remove if in constant collision?
    return mat

  no_agent_types=len(agent_types)
  near_miss_mat=np.zeros([no_agent_types, no_agent_types])
  TT=len(data)
  agent_num=data.shape[1]
  for t in np.arange(TT):
    for v in np.arange(agent_num):
      near_miss_mat=checkCollision(data[t][v:], near_miss_mat)

    # near_miss_mat=[checkCollision(data[t][v:]) for v in np.arange(agent_num)]

  np.savez(f"{out_dir}/{near_miss_matrix_data_filename}", near_miss_matrix=near_miss_mat)

  return near_miss_mat


def calculateNearMiss(data, tolerance=0.001, data_override=False):
  if not data_override and os.path.exists(f"{out_dir}/{near_miss_data_filename}"):
    results=np.load(f"{out_dir}/{near_miss_data_filename}")['near_miss']
    return results
  def checkCollision(vals):
    ego=vals[0]
    others=vals[1:,0:2]
    pos=ego[0:2]
    diff=others-pos
    dist=np.sqrt(diff[:,0]**2+diff[:,1]**2)-vals[0,3]-vals[1:,3]
    ## Remove if in constant collision?
    return np.sum(dist<tolerance)
  TT=len(data)
  agent_num=data.shape[1]
  near_collisions=[0]*TT
  for t in np.arange(TT):
    collisions=[checkCollision(data[t][v:]) for v in np.arange(agent_num)]
    near_collisions[t]=np.sum(collisions)
  np.savez(f"{out_dir}/{near_miss_data_filename}", near_miss=near_collisions)
  
  mass_collisions=0
  test_t=0
  for t in np.arange(TT):
    v_ind=-1
    for v in np.arange(agent_num):
      if "MASS" in agent_types[data[t][v][-1]]:
        print(agent_types[data[t][v][-1]])
        v_ind=v
        test_t+=1
        break

    if v_ind>-1:
      ego=data[t][v_ind]
      others=np.delete(data[t], v_ind, axis=0)
      pos=ego[0:2]
      diff=others[:,0:2]-pos
      dist=np.sqrt(diff[:,0]**2+diff[:,1]**2)-data[t][v_ind][3]-others[:,3]
      ## Remove if in constant collision?
      mass_collisions+=np.sum(dist<tolerance)
  print(f"Collision ratio: {test_t} {mass_collisions}/{np.sum(near_collisions)}")
  return near_collisions


def loadData():
  data=np.load(f"{out_dir}/{data_filename}")['data']
  with open(f"{out_dir}/{agent_types_pickle_name}", 'rb') as handle:
    agent_types=pickle.load(handle)
  return data, agent_types


def avgNeighCount(data, data_override=False):
  if not data_override and os.path.exists(f"{out_dir}/{neigh_count_data_filename}"):
    # results=np.load(f"{out_dir}/{neigh_count_data_filename}", allow_pickle=True)['avg_neigh_count']
    with open(f"{out_dir}/{neigh_count_data_filename}", 'rb') as handle:
      results=pickle.load(handle)
    return results

  TT=len(data)
  results={}
  for agent_type in agent_types:
    results[agent_type]=[-1]*TT
  for t in np.arange(TT):
    for agent_type in agent_types:
      vals=[v[5] for v in data[t,:,:] if v[12]==agent_type]
      results[agent_type][t]=np.mean(vals)
  # np.savez(f"{out_dir}/{neigh_count_data_filename}", avg_neigh_count=results)
  with open(f"{out_dir}/{neigh_count_data_filename}", 'wb') as handle:
    pickle.dump(results, handle, protocol=pickle.HIGHEST_PROTOCOL)
  return results


def setAgentTypes(_agent_types):
  # global agent_types
  agent_types=_agent_types


def avgNeighDist(data, data_override=False):
  if not data_override and os.path.exists(f"{out_dir}/{neigh_dist_data_filename}"):
    with open(f"{out_dir}/{neigh_dist_data_filename}", 'rb') as handle:
      results=pickle.load(handle)
    return results
  TT=len(data)
  results={}
  for agent_type in agent_types:
    results[agent_type]=[-1]*TT
  for t in np.arange(TT):
    for agent_type in agent_types:
      vals=[v[6] for v in data[t,:,:] if (v[12]==agent_type)]# and v[5]>0)]
      results[agent_type][t]=np.mean(vals)
  # np.savez(f"{out_dir}/{neigh_dist_data_filename}", avg_neigh_dist=results)
  with open(f"{out_dir}/{neigh_dist_data_filename}", 'wb') as handle:
    pickle.dump(results, handle, protocol=pickle.HIGHEST_PROTOCOL)
  return results

def storeData(data_override=False):
  if not data_override and os.path.exists(f"{out_dir}/{agent_types_pickle_name}"):
    if os.path.exists(f"{out_dir}/{data_filename}"):
      print("Data already extracted")
      return

  print("\tExtracting raw data...")
  TT=1
  features=13
  data=np.ones((TT,0,features), dtype=float)*-1
  files=os.listdir(results_dir)
  files=[file for file in files if file[-4:]=='.txt']

  temp_types={}

  def convertRow(val_in):
    global current_agent_ind
    val=val_in.decode()
    try:
      float(val)
      return float(val)
    except ValueError:
      # if val not in agent_types:
      if val not in temp_types:
        val_new=current_agent_ind
        current_agent_ind+=1
        agent_types[int(val_new)]=val
        temp_types[val]=int(val_new)
      return temp_types[val]
      # return agent_types[int(val_new)]

  for file in files:
    new_data=np.loadtxt(f"{results_dir}/{file}", converters=convertRow)
    start_time=int(new_data[0,11])
    end_time=int(new_data[-1,11])
    if end_time>TT:
      dims=data.shape
      data=np.append(data, np.ones([end_time+1-TT,dims[1],dims[2]])*-1, axis=0)
      TT=end_time+1
    data=np.append(data, np.ones([TT, 1, dims[2]])*-1, axis=1)
    data[start_time:end_time+1, -1, :]=new_data
  with open(f"{out_dir}/{agent_types_pickle_name}", 'wb') as handle:
    pickle.dump(agent_types, handle, protocol=pickle.HIGHEST_PROTOCOL)
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