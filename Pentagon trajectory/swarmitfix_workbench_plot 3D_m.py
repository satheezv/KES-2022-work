import time
start = time.time() 
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d.art3d as art3d
from coordinates import * 
from segment_ch import *
#from CSP_Head import *
#from head_policy_calculation_MC_MODIFIED import *
#from base_csp_test import *
from head_support_location import *
from base1_motion import *
#from base2_motion import *
#from plots import *
#from head_ret_val_plot import *
import matplotlib.font_manager as font_manager



# IMPORTING PIN COODINATES

X=      [0.0254*P1.x,0.0254*P2.x,0.0254*P3.x,0.0254*P4.x,0.0254*P5.x,0.0254*P6.x,0.0254*P7.x,0.0254*P8.x,0.0254*P9.x,0.0254*P10.x,\
        0.0254*P11.x,0.0254*P12.x,0.0254*P13.x,0.0254*P14.x,0.0254*P15.x,0.0254*P16.x,0.0254*P17.x,0.0254*P18.x,0.0254*P19.x,0.0254*P20.x,\
        0.0254*P21.x,0.0254*P22.x,0.0254*P23.x,0.0254*P24.x,0.0254*P25.x,0.0254*P26.x,0.0254*P27.x,0.0254*P28.x,0.0254*P29.x,0.0254*P30.x,\
        0.0254*P31.x,0.0254*P32.x,0.0254*P33.x,0.0254*P34.x,0.0254*P35.x,0.0254*P36.x,0.0254*P37.x,0.0254*P38.x,0.0254*P39.x,0.0254*P40.x,\
        0.0254*P41.x,0.0254*P42.x,0.0254*P43.x,0.0254*P44.x,0.0254*P45.x,0.0254*P46.x,0.0254*P47.x,0.0254*P48.x,0.0254*P49.x,0.0254*P50.x,\
        0.0254*P51.x,0.0254*P52.x,]

Y=      [0.0254*P1.y,0.0254*P2.y,0.0254*P3.y,0.0254*P4.y,0.0254*P5.y,0.0254*P6.y,0.0254*P7.y,0.0254*P8.y,0.0254*P9.y,0.0254*P10.y,\
        0.0254*P11.y,0.0254*P12.y,0.0254*P13.y,0.0254*P14.y,0.0254*P15.y,0.0254*P16.y,0.0254*P17.y,0.0254*P18.y,0.0254*P19.y,0.0254*P20.y,\
        0.0254*P21.y,0.0254*P22.y,0.0254*P23.y,0.0254*P24.y,0.0254*P25.y,0.0254*P26.y,0.0254*P27.y,0.0254*P28.y,0.0254*P29.y,0.0254*P30.y,\
        0.0254*P31.y,0.0254*P32.y,0.0254*P33.y,0.0254*P34.y,0.0254*P35.y,0.0254*P36.y,0.0254*P37.y,0.0254*P38.y,0.0254*P39.y,0.0254*P40.y,\
        0.0254*P41.y,0.0254*P42.y,0.0254*P43.y,0.0254*P44.y,0.0254*P45.y,0.0254*P46.y,0.0254*P47.y,0.0254*P48.y,0.0254*P49.y,0.0254*P50.y,\
        0.0254*P51.y,0.0254*P52.y,]


for g in range(len(A)):
    A[g]=0.0254*A[g]
    B[g]=0.0254*B[g]




fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
patch = patches.PathPatch(path,alpha = 0.4,lw=1,facecolor="none")
ax.add_patch(patch)
art3d.pathpatch_2d_to_3d(patch, z=1.55, zdir="z")

for t in range(len(verts_m)):
    plt.plot(verts_m[t][0], verts_m[t][1], 1.55, 'k', marker=".")


ax.scatter(X,Y,zs=0,zdir="z", label='Pins',s=30, color="r", marker="x", alpha = 0.3)
ax.scatter(A,B,zs=0,zdir="z", label='centroid',s=10, color="k", marker=".", alpha = 0.3)



for R in range(len(pathhead_cornor)): 
    patchhead_mdp = patches.PathPatch(pathhead_cornor[R],facecolor='none',edgecolor='g',linestyle='-',alpha = 1, lw=1.2,)
    ax.add_patch(patchhead_mdp)
    art3d.pathpatch_2d_to_3d(patchhead_mdp, z=1.55, zdir="z")    

#plt.axis('equal')




#base_circle-agent1
for k in range(len(agent1_goal_centroid_x)):
    circle2 = plt.Circle((0.0254*agent1_goal_centroid_x[k], 0.0254*agent1_goal_centroid_y[k]), 0.0254*11.5, color='g',fill=False, alpha = 0.8)
    ax.add_patch(circle2)
    art3d.pathpatch_2d_to_3d(circle2, z=0, zdir="z")
    
for f in range(len(agent1_intermediate_centroid_x)):
    circle3 = plt.Circle((0.0254*agent1_intermediate_centroid_x[f], 0.0254*agent1_intermediate_centroid_y[f]), 0.0254*11.5, color='b',linestyle='--',fill=False, alpha = 0.8)
    ax.add_patch(circle3)
    art3d.pathpatch_2d_to_3d(circle3, z=0, zdir="z")

# #base_circle-agent2
# for k in range(len(agent2_goal_centroid_x)):
#     circle4 = plt.Circle((0.0254*agent2_goal_centroid_x[k], 0.0254*agent2_goal_centroid_y[k]), 0.0254*11.5, color='b',fill=False, alpha = 0.8)
#     ax.add_patch(circle4)
#     art3d.pathpatch_2d_to_3d(circle4, z=0, zdir="z")
    
# for f in range(len(agent2_intermediate_centroid_x)):
#     circle5 = plt.Circle((0.0254*agent2_intermediate_centroid_x[f], 0.0254*agent2_intermediate_centroid_y[f]), 0.0254*11.5, color='b',linestyle='--',fill=False, alpha = 0.8)
#     ax.add_patch(circle5)
#     art3d.pathpatch_2d_to_3d(circle5, z=0, zdir="z")

      
#connection-robot1
no_home_goalbase1_x=agent1_goal_centroid_x.copy()
no_home_goalbase1_y=agent1_goal_centroid_y.copy()
# no_home_goalbase1_x.remove(no_home_goalbase1_x[0])
# no_home_goalbase1_y.remove(no_home_goalbase1_y[0])
for m in range(len(head_c)):
    ox_c_a1=[head_c[m][0],0.0254*no_home_goalbase1_x[m]]
    oy_c_al=[head_c[m][1],0.0254*no_home_goalbase1_y[m]]
    oz_c_al=[1.55,0]
    plt.plot(ox_c_a1,oy_c_al,oz_c_al, 'r')

# #connection-robot2
# no_home_goalbase2_x=agent2_goal_centroid_x.copy()
# no_home_goalbase2_y=agent2_goal_centroid_y.copy()
# no_home_goalbase2_x.remove(no_home_goalbase2_x[0])
# no_home_goalbase2_y.remove(no_home_goalbase2_y[0])
# for m in range(len(headcentroid_a2_x)):
#     ox_c_a2=[0.0254*headcentroid_a2_x[m],0.0254*no_home_goalbase2_x[m]]
#     oy_c_a2=[0.0254*headcentroid_a2_y[m],0.0254*no_home_goalbase2_y[m]]
#     oz_c_a2=[1.55,0]
#     plt.plot(ox_c_a2,oy_c_a2,oz_c_a2, 'r')


#ax.plot( color='-b', label='Head 1')
#ax.plot( color='-g', label='Head 2')
ax.legend(loc='upper center')

ax.set_xlim3d(0, 2.75)
ax.set_ylim3d(0, 2.50)
ax.set_zlim3d(0, 2.00)

font_prop = font_manager.FontProperties(size=10)
ax.set_xlabel('x (m)',fontproperties=font_prop)
ax.set_ylabel('y (m)',fontproperties=font_prop)
ax.set_zlabel('z (m)',fontproperties=font_prop)
#plt.axis('equal')
end = time.time()
exe_time= end - start
print(end - start) 

plt.savefig('head.png', dpi=300)
plt.show()   
