import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import matplotlib.animation as animation
import os
import sys
from visualise_utils import *


speed=int(sys.argv[1])
zoom=float(sys.argv[2])
fig, ax, ax_zoom, ax_lines, agents, output=setupAxes(zoom)

TT=len(output[0])

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