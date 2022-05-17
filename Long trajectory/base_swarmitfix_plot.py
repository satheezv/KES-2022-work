import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import matplotlib.animation as animation
from coordinates import * 
from segment import *
from CSP_Head import *
from base_csp_test import *
import time


start = time.time() 
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
ax = fig.add_subplot(111)
patch = patches.PathPatch(path,alpha = 0.4,lw=1,facecolor="y")
ax.add_patch(patch)



#PLOTTING PINS AND CENTROIDS OF THE WORKBENCH

plt.scatter(X,Y, label='Pins', color="r", s=70, marker="x", alpha = 0.4)
plt.scatter(A,B, label='centroid', color="k", s=10, marker=".", alpha = 0.4)


##rectangle test for base

for R in range(len(ocoord_1)):
    plt.scatter(ocoord_1[R][0],ocoord_1[R][1],  color="k", s=10, marker="D", alpha = 0.4)
    plt.scatter(ocoord_2[R][0],ocoord_2[R][1],  color="k", s=10, marker="D", alpha = 0.4)
    plt.scatter(ocoord_3[R][0],ocoord_3[R][1],  color="k", s=10, marker="D", alpha = 0.4)
    plt.scatter(ocoord_4[R][0],ocoord_4[R][1],  color="k", s=10, marker="D", alpha = 0.4)

for U in range(len(icoord_1)):
    plt.scatter(icoord_1[U][0],icoord_1[U][1],  color="k", s=10, marker="s", alpha = 0.4)
    plt.scatter(icoord_2[U][0],icoord_2[U][1],  color="k", s=10, marker="s", alpha = 0.4)
    plt.scatter(icoord_3[U][0],icoord_3[U][1],  color="k", s=10, marker="s", alpha = 0.4)
    plt.scatter(icoord_4[U][0],icoord_4[U][1],  color="k", s=10, marker="s", alpha = 0.4)

#CONNECTING ARM


# PLOTTING HEAD

for p in range(len(path_head)):
    if p % 2 == 0:
        patch_head = patches.PathPatch(path_head[p],facecolor='none',edgecolor='b',alpha = 1, lw=1,)
        ax.add_patch(patch_head)
        plt.scatter(total_headscentrd[p][0],total_headscentrd[p][1],  color="b", s=10, marker=".", alpha = 1)
    else:
        patch_head = patches.PathPatch(path_head[p],facecolor='none',edgecolor='g',alpha = 1, lw=1,)
        ax.add_patch(patch_head)
        plt.scatter(total_headscentrd[p][0],total_headscentrd[p][1],  color="g", s=10, marker=".", alpha = 1)
for k in range(len(icentroid_final)):
    circle1 = plt.Circle((icentroid_final[k][0], icentroid_final[k][1]), 11.5, color='g',fill=False, alpha = 0.4)
    ax.add_artist(circle1)

for k in range(len(ocentroid_final)):
    circle2 = plt.Circle((ocentroid_final[k][0], ocentroid_final[k][1]), 11.5, color='b',fill=False, alpha = 0.4)
    ax.add_artist(circle2)

#BASE-HEAD CONNECTION
##outer head    
oconnection=[]
for e in range(len(centrds_Head1)):
    oconnection.append(0)
for f in range(len(ocentroid_final)-1):
    if oconnection[f]==0:
        ox_c=[centrds_Head1[f][0],ocentroid_final[f][0]]
        oy_c=[centrds_Head1[f][1],ocentroid_final[f][1]]
        plt.plot(ox_c, oy_c, 'b')
        oconnection.insert(f, 1) 
    if oconnection[f+1]==0:
        ox_c=[centrds_Head1[f+1][0],ocentroid_final[f+1][0]]
        oy_c=[centrds_Head1[f+1][1],ocentroid_final[f+1][1]]
        plt.plot(ox_c, oy_c, 'b')
        oconnection.insert(f+1, 1) 

##inner head
iconnection=[]
for e in range(len(centrds_Head2)):
    iconnection.append(0)
for f in range(len(icentroid_final)-1):
    if iconnection[f]==0:
        ix_c=[centrds_Head2[f][0],icentroid_final[f][0]]
        iy_c=[centrds_Head2[f][1],icentroid_final[f][1]]
        plt.plot(ix_c, iy_c, 'g')
        iconnection.insert(f, 1) 
    if iconnection[f+1]==0:
        ix_c=[centrds_Head2[f+1][0],icentroid_final[f+1][0]]
        iy_c=[centrds_Head2[f+1][1],icentroid_final[f+1][1]]
        plt.plot(ix_c, iy_c, 'g')
        iconnection.insert(f+1, 1) 
       
ax.plot( color='-b', label='Head 1')
ax.plot( color='-g', label='Head 2')
ax.legend(loc='upper left')
plt.xlim(0,110)
plt.ylim(0,80)
plt.axis('equal')
end = time.time()
print(end - start) 
plt.show()
