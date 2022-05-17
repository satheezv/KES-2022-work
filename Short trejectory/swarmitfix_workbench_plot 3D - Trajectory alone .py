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
patch = patches.PathPatch(path,alpha = 0.7,lw=1,edgecolor="r", facecolor="none")
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

ax.scatter(X,Y,zs=0,zdir="z", label='Pins',s=30, color="r", marker="x", alpha = 0.5)
ax.scatter(A,B,zs=0,zdir="z", label='centroid',s=10, color="k", marker=".", alpha = 0.5)

headcentroid_x=[]
headcentroid_y=[]

font = {'family' : 'normal',
        #'weight' : 'bold',
        'size'   : 4}

plt.rc('font', **font)
#ax.legend(loc='upper center')

ax.set_xlim3d(0, 100)
ax.set_ylim3d(0, 80)
ax.set_zlim3d(0, 90)
ax.tick_params(labelcolor='k', labelsize='8', width=7)
font_prop = font_manager.FontProperties(size=8)
ax.set_xlabel('x (in)',fontproperties=font_prop)
ax.set_ylabel('y (in)',fontproperties=font_prop)
ax.set_zlabel('z (in)',fontproperties=font_prop)
#plt.axis('equal')
end = time.time()
exe_time= end - start
print(end - start) 
plt.show()
