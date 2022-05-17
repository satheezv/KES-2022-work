import time
start = time.time() 
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d.art3d as art3d
from coordinates import * 
from segment import *
#from CSP_Head import *
#from head_policy_calculation_MC_MODIFIED import *
#from base_csp_test import *
from head_conversion_policy_to_location_test import *
from base1_motion import *
from base2_motion import *
#from plots import *
#from head_ret_val_plot import *
import matplotlib.font_manager as font_manager



# IMPORTING PIN COODINATES

X=      [P1.x,P2.x,P3.x,P4.x,P5.x,P6.x,P7.x,P8.x,P9.x,P10.x,\
        P11.x,P12.x,P13.x,P14.x,P15.x,P16.x,P17.x,P18.x,P19.x,P20.x,\
        P21.x,P22.x,P23.x,P24.x,P25.x,P26.x,P27.x,P28.x,P29.x,P30.x,\
        P31.x,P32.x,P33.x,P34.x,P35.x,P36.x,P37.x,P38.x,P39.x,P40.x,\
        P41.x,P42.x,P43.x,P44.x,P45.x,P46.x,P47.x,P48.x,P49.x,P50.x,\
        P51.x,P52.x,]

Y=      [P1.y,P2.y,P3.y,P4.y,P5.y,P6.y,P7.y,P8.y,P9.y,P10.y,\
        P11.y,P12.y,P13.y,P14.y,P15.y,P16.y,P17.y,P18.y,P19.y,P20.y,\
        P21.y,P22.y,P23.y,P24.y,P25.y,P26.y,P27.y,P28.y,P29.y,P30.y,\
        P31.y,P32.y,P33.y,P34.y,P35.y,P36.y,P37.y,P38.y,P39.y,P40.y,\
        P41.y,P42.y,P43.y,P44.y,P45.y,P46.y,P47.y,P48.y,P49.y,P50.y,\
        P51.y,P52.y,]





# PLOTTING METAL SHEET CONTOUR
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
patch = patches.PathPatch(path,alpha = 0.4,lw=1,facecolor="none")
ax.add_patch(patch)
art3d.pathpatch_2d_to_3d(patch, z=61.46, zdir="z")

"""
#inches to cm
X1 = [l * 2.54 for l in X]
Y1 = [l * 2.54 for l in Y]
A1 = [l * 2.54 for l in A]
B1 = [l * 2.54 for l in B]
"""

#PLOTTING PINS AND CENTROIDS OF THE WORKBENCH

ax.scatter(X,Y,zs=0,zdir="z", label='Pins',s=30, color="r", marker="x", alpha = 0.3)
ax.scatter(A,B,zs=0,zdir="z", label='centroid',s=10, color="k", marker=".", alpha = 0.3)

headcentroid_a1_x=[]
headcentroid_a1_y=[]
headcentroid_a2_x=[]
headcentroid_a2_y=[]
for R in range(len(pathhead_mdp)): 
    if R % 2 == 0:
        ax.scatter(lefthead_mdp[R][0],lefthead_mdp[R][1],zs=61.46,zdir="z",s=5,  color="r",  marker="D", alpha = 0.1)  #2.54 ---> inches2cm
        ax.scatter(righthead_mdp[R][0],righthead_mdp[R][1],zs=61.46,zdir="z",s=5, color="g", marker="D", alpha = 0.1) #2.54 ---> inches2cm
        ax.scatter(fronthead_mdp[R][0],fronthead_mdp[R][1],zs=61.46,zdir="z",s=5, color="b", marker="D", alpha = 0.1) #2.54 ---> inches2cm
        patchhead_mdp = patches.PathPatch(pathhead_mdp[R],facecolor='none',edgecolor='g',linestyle='-',alpha = 1, lw=1.2,)
        ax.add_patch(patchhead_mdp)
        ax.scatter(centroidhead_mdp[R][0],centroidhead_mdp[R][1],zs=61.46,zdir="z",s=10,  color="g", marker=".", alpha = 1)
        headcentroid_a1_x.append(centroidhead_mdp[R][0])
        headcentroid_a1_y.append(centroidhead_mdp[R][1])
        art3d.pathpatch_2d_to_3d(patchhead_mdp, z=61.46, zdir="z")
 
    elif R % 2 != 0:
        patchhead_mdp = patches.PathPatch(pathhead_mdp[R],facecolor='none',edgecolor='b',alpha = 1, lw=1,)
        ax.add_patch(patchhead_mdp)
        ax.scatter(centroidhead_mdp[R][0],centroidhead_mdp[R][1],zs=61.46,zdir="z",s=10, color="b", marker=".", alpha = 1)
        headcentroid_a2_x.append(centroidhead_mdp[R][0])
        headcentroid_a2_y.append(centroidhead_mdp[R][1])
        art3d.pathpatch_2d_to_3d(patchhead_mdp, z=61.46, zdir="z")

#base_circle-agent1
for k in range(len(agent1_goal_centroid_x)):
    circle2 = plt.Circle((agent1_goal_centroid_x[k], agent1_goal_centroid_y[k]), 11.5, color='g',fill=False, alpha = 0.8)
    ax.add_patch(circle2)
    art3d.pathpatch_2d_to_3d(circle2, z=0, zdir="z")
    
for f in range(len(agent1_intermediate_centroid_x)):
    circle3 = plt.Circle((agent1_intermediate_centroid_x[f], agent1_intermediate_centroid_y[f]), 11.5, color='g',linestyle='--',fill=False, alpha = 0.8)
    ax.add_patch(circle3)
    art3d.pathpatch_2d_to_3d(circle3, z=0, zdir="z")

#base_circle-agent2
for k in range(len(agent2_goal_centroid_x)):
    circle4 = plt.Circle((agent2_goal_centroid_x[k], agent2_goal_centroid_y[k]), 11.5, color='b',fill=False, alpha = 0.8)
    ax.add_patch(circle4)
    art3d.pathpatch_2d_to_3d(circle4, z=0, zdir="z")
    
for f in range(len(agent2_intermediate_centroid_x)):
    circle5 = plt.Circle((agent2_intermediate_centroid_x[f], agent2_intermediate_centroid_y[f]), 11.5, color='b',linestyle='--',fill=False, alpha = 0.8)
    ax.add_patch(circle5)
    art3d.pathpatch_2d_to_3d(circle5, z=0, zdir="z")

      
#connection-robot1
no_home_goalbase1_x=agent1_goal_centroid_x.copy()
no_home_goalbase1_y=agent1_goal_centroid_y.copy()
no_home_goalbase1_x.remove(no_home_goalbase1_x[0])
no_home_goalbase1_y.remove(no_home_goalbase1_y[0])
for m in range(len(headcentroid_a1_x)):
    ox_c_a1=[headcentroid_a1_x[m],no_home_goalbase1_x[m]]
    oy_c_al=[headcentroid_a1_y[m],no_home_goalbase1_y[m]]
    oz_c_al=[61.46,0]
    plt.plot(ox_c_a1,oy_c_al,oz_c_al, 'r')

#connection-robot2
no_home_goalbase2_x=agent2_goal_centroid_x.copy()
no_home_goalbase2_y=agent2_goal_centroid_y.copy()
no_home_goalbase2_x.remove(no_home_goalbase2_x[0])
no_home_goalbase2_y.remove(no_home_goalbase2_y[0])
for m in range(len(headcentroid_a2_x)):
    ox_c_a2=[headcentroid_a2_x[m],no_home_goalbase2_x[m]]
    oy_c_a2=[headcentroid_a2_y[m],no_home_goalbase2_y[m]]
    oz_c_a2=[61.46,0]
    plt.plot(ox_c_a2,oy_c_a2,oz_c_a2, 'r')


#ax.plot( color='-b', label='Head 1')
#ax.plot( color='-g', label='Head 2')
ax.legend(loc='upper center')
"""
ax.set_xlim3d(0, 250)
ax.set_ylim3d(0, 200)
ax.set_zlim3d(0, 200)
"""
font_prop = font_manager.FontProperties(size=10)
ax.set_xlabel('x (in)',fontproperties=font_prop)
ax.set_ylabel('y (in)',fontproperties=font_prop)
ax.set_zlabel('z (in)',fontproperties=font_prop)
#plt.axis('equal')
end = time.time()
exe_time= end - start
print(end - start) 
plt.show()
