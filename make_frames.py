import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
# import matplotlib.animation as animation
from multiprocessing import Pool
import tqdm
import istarmap
from visualise_utils import *
import distinctipy

import os
import sys

speed=int(sys.argv[1])
zoom=float(sys.argv[2])

agent_colours=distinctipy.get_colors(20)

# fig, ax, ax_zoom, ax_lines, agent_deployed_info, agent_waiting_info, TT=setupAxes(zoom)

agent_info, max_vals, goals, TT=getAllAgentInfo()
# [rect_info, goal_type]=goals[goal]
goal_coords=np.array([[goals[key][0][0]+goals[key][0][1], goals[key][0][0]-goals[key][0][1], 
                goals[key][0][2]+goals[key][0][3], 
                goals[key][0][2]-goals[key][0][3]] for key in goals.keys()])
print(goal_coords)
max_vals=max_vals[:len(goal_coords)]
for i in np.arange(len(goal_coords)):
    max_vals[i][:4]=goal_coords[i]
# print(max_vals)
# sys.exit()
# start_x=rect_info[0]-rect_info[1]
# end_x=rect_info[0]+rect_info[1]
# start_y=rect_info[2]-rect_info[3]
# end_y=rect_info[2]+rect_info[3]


def makeFrame(t):
  ## Check if new agents needs to be added
    fig, ax, ax_zoom=setUpSimAxesOnly(zoom, goals, max_vals, colour=agent_colours)
    for key in agent_info.copy():
        # print(key)
        agent=agent_info[key]
        time_step=np.where(agent[:,9]==t*speed)
        if np.size(time_step)!=0:
            new_ax_agent=plotOnAx(agent, int(agent[time_step[0][0],9]), ax, colour=agent_colours)
            new_ax_zoom_agent=plotOnAx(agent, int(agent[time_step[0][0],9]), ax_zoom, colour=agent_colours)
    ax.plot(np.arange(-200,200), np.zeros(400), linestyle='--', c=[0,0,0])
    ax_zoom.plot(np.arange(-200,200), np.zeros(400), linestyle='--', c=[0,0,0])
    str_t=str(t)
    no_zeros=10
    char_len=no_zeros-len(str_t)
    for c in np.arange(char_len):
        str_t='0'+str_t
    plt.savefig(f'output_video/temp_image_{str_t}.png', bbox_inches='tight', dpi=500)
    plt.close()
    return 0


# TT=len(output[0])
frames=np.arange(0,TT,speed)

# makeFrame(100)

# for t in frames:
#     makeFrame(int(t))
#     print(f"Completion:   {t/TT}")

with Pool(5) as pool:
    # x=[(t, ax, ax_zoom, ax_lines, agent_deployed_info, agent_waiting_info) for t in frames]
    x=[(t,) for t in frames]
    for _ in tqdm.tqdm(pool.istarmap(makeFrame, x),
                       total=len(x)):
        pass