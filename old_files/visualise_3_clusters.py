import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import matplotlib.animation as animation
import os
import sys

def getInd(filename):
    fn=filename.split('.')[0]
    fn=fn.split('_')[1]
    return int(fn)

def getColour(x_type):
    blue_agent=np.array([0,0,1])
    green_agent=np.array([0.6,0.6,0])
    pink_agent=np.array([0.4,0.6,0.5])
    x_colour=np.array([0,0,0])
    if x_type==0:
        x_colour=blue_agent
    elif x_type==1:
        x_colour=green_agent
    else:
        x_colour=pink_agent
    return x_colour

files=os.listdir('results')
fig=plt.figure()
axs=[]
gs=fig.add_gridspec(2, 3)
ax=fig.add_subplot(gs[0,0:2])
ax_lines=fig.add_subplot(gs[:,2])
ax_zoom=fig.add_subplot(gs[1,0:2])

output=[]
agents=[]
agents_zoom=[]
# new_agents=len(files)/3

def plotOnAx(values, t, ax_x):
    r_radius=values[t,3]
    rng_radius_small=values[t,4]

    ## Setting up robot
    # r_colour=np.array([0,0,0]) 
    r_colour=getColour(values[t,-1])

    rbt=plt.Circle(values[t,0:2], radius=r_radius, fc=r_colour, alpha=1.0)
    rbt=ax_x.add_patch(rbt)
    rbt_sns=plt.Circle(values[t,0:2], radius=rng_radius_small, fc=np.array([0,1,0]), alpha=0.2)
    rbt_sns=ax_x.add_patch(rbt_sns)
    rbt_angle=(values[t,2]*np.pi/180.0)
    rbt_lnx=(1.2*r_radius)*np.cos(values[t,2]*np.pi/180.0)
    rbt_lny=(1.2*r_radius)*np.sin(values[t,2]*np.pi/180.0)

    ## Local line
    rbt_ln,=ax_x.plot([results[0,0],results[0,0]+rbt_lnx], [results[0,1], results[0,1]+rbt_lny], linewidth=r_radius*3, c=[0,0,0])
    agent=[rbt, r_radius, rbt_sns, rbt_angle, rbt_lnx, rbt_lny, rbt_ln]
    return agent

goals={}
goal_ind=0

for file in files:
    results=np.loadtxt('results/'+file)

    goal=results[0,5:9]
    found=False
    for key in goals:
        if (goals[key][0]==goal).all():
            found=True
            break
    if not found:
        goals[goal_ind]=[goal, results[0,-1]]
        goal_ind+=1

    agent=plotOnAx(results, 0, ax)
    agents.append(agent)

    agent_zoom=plotOnAx(results,0,ax_zoom)
    agents_zoom.append(agent_zoom)

    output.append(results)
    ax_lines.plot(results[:,0], results[:,1], alpha=0.3)

buffer=2.5
x_max=buffer+np.max(np.array([np.max(arr[:,0]) for arr in output]))
x_min=-buffer+np.min(np.array([np.min(arr[:,0]) for arr in output]))
y_max=buffer+np.max(np.array([np.max(arr[:,1]) for arr in output]))
y_min=-buffer+np.min(np.array([np.min(arr[:,1]) for arr in output]))

#add rectangle to plot
ax.set_xlim([x_min,x_max])
ax.set_ylim([y_min,y_max])
ax.set_aspect('equal')

zoom_mag=1.0/float(sys.argv[2])
ax_zoom.set_xlim([zoom_mag*x_min,zoom_mag*x_max])
ax_zoom.set_ylim([zoom_mag*y_min,zoom_mag*y_max])
ax_zoom.set_aspect('equal')

speed=int(sys.argv[1])

TT=len(output[0])

## Plot agents's goal locations
for goal in goals:
    [rect_info, goal_type]=goals[goal]
    rect_centre=[rect_info[0]-rect_info[1],rect_info[2]-rect_info[3]]
    width=2*rect_info[1]
    height=2*rect_info[3]
    temp_goal_area_reg=plt.Rectangle(rect_centre,width,height,fc=getColour(goal_type), alpha=0.1)
    temp_goal_area_zoom=plt.Rectangle(rect_centre,width,height,fc=getColour(goal_type), alpha=0.1)
    ax.add_patch(temp_goal_area_reg)
    ax_zoom.add_patch(temp_goal_area_zoom)

def animate(t):
    def updateAgent(vals, output_vals, ind):
        r_radius=vals[i][1]
        vals[i][0].center=output_vals[ind][speed*t,0],output_vals[ind][speed*t,1]
        vals[i][2].center=output_vals[ind][speed*t,0],output_vals[ind][speed*t,1]
        pt_x=(1.2*r_radius)*np.cos(output_vals[ind][speed*t,2]*np.pi/180.0)
        pt_y=(1.2*r_radius)*np.sin(output_vals[ind][speed*t,2]*np.pi/180.0)
        vals[i][6].set_data([output_vals[ind][speed*t,0], output_vals[ind][speed*t,0]+pt_x], [output_vals[ind][speed*t,1], output_vals[ind][speed*t,1]+pt_y])

    if t==0:
        plt.waitforbuttonpress()
    if t%100==0:
        print(t*speed)

    for i in np.arange(len(agents)):
        updateAgent(agents, output, i)
        updateAgent(agents_zoom, output, i)

line_ani=animation.FuncAnimation(fig, animate, frames=int(TT/speed), interval=16, blit=False, repeat=False)
plt.show()