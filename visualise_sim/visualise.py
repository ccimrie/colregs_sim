import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import matplotlib.animation as animation
import os
import sys
from visualise_utils import *

speed=int(sys.argv[1])
fig, ax, ax_lines, agent_deployed_info, agent_waiting_info, TT=setupAxes(axis_lines=True)

def getCommands(agent_id):
    commands=np.loadtxt("../build/results/"+agent_id[:-4]+"_nmpc_seq.txt")
    return commands

mpc_commands={}
for ship in agent_deployed_info.keys():
    test_commands=getCommands(ship)
    command_length=int(len(test_commands[0])/2)
    # print(command_length)

    # test_commands=np.reshape(test_commands[0,:],[11,10,5])
    # print(test_commands[:,:,0])
    # sys.exit()
    # mpc_plan,=ax.plot(test_commands[0,0:command_length],test_commands[0,command_length:command_length*2],marker='o',color='blue')
    mpc_plan,=ax.plot(test_commands[0,0:command_length],test_commands[0,command_length:],marker='o',color='blue')
    mpc_commands[ship]=[test_commands,mpc_plan]

# print(np.shape(test_commands))

def animate(t):
    if t==0:
        plt.waitforbuttonpress()
    if t%10==0:
        print(f"{t*speed}/{int(TT)}")
    updates=[]

  ## Check if new agents needs to be added
    for key in agent_waiting_info.copy():
        if agent_waiting_info[key][0,11]<=t*speed:
            new_ax_agent=plotOnAx(agent_waiting_info[key], int(agent_waiting_info[key][0,11]), ax)
            agent_deployed_info[key]=[new_ax_agent, agent_waiting_info[key]]
            agent_waiting_info.pop(key)

    for key in agent_deployed_info.copy():
        # print(f"KEY {key}")
        if agent_deployed_info[key][1][-1,11]<t*speed:
            # print("  Removing")
            def removeAgent(agent_obj):
                agent_obj[0].remove()
                agent_obj[2].remove()
                agent_obj[4].remove()
            removeAgent(agent_deployed_info[key][0])
            agent_deployed_info.pop(key)
        else:
            updateAgent(agent_deployed_info[key][0], agent_deployed_info[key][1], t*speed)
            # print(mpc_commands[key][0][int(t*speed),0:command_length])
            if all(mpc_commands[key][0][int(t*speed),0:command_length]==mpc_commands[key][0][int(t*speed),0:command_length][0]) and all(mpc_commands[key][0][int(t*speed),command_length:]==mpc_commands[key][0][int(t*speed),command_length:][0]):
                if mpc_commands[key][1].axes is not None:
                    mpc_commands[key][1].remove()
            else:
                if mpc_commands[key][1].axes is None:
                    mpc_plan,=ax.plot(mpc_commands[key][0][int(t*speed),0:command_length],mpc_commands[key][0][int(t*speed),command_length:],marker='o',color='blue')
                    mpc_commands[key][1]=mpc_plan
                else:
                    mpc_commands[key][1].set_data(mpc_commands[key][0][int(t*speed),0:command_length],mpc_commands[key][0][int(t*speed),command_length:])
line_ani=animation.FuncAnimation(fig, animate, frames=int(TT/speed), interval=16, blit=False, repeat=False)
plt.show()