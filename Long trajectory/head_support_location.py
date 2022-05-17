# -*- coding: utf-8 -*-
"""
Created on Mon Jan 24 14:24:56 2022

@author: scesv7
"""

import numpy as np
#from transition_matrix_test import *
import time
start=time.time()
#from CSP_Head import *
#from TEMP_CHECK_MC import *
from segment_ch import *
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d.art3d as art3d
import matplotlib.font_manager as font_manager

segment=[]
for i in range(len(verts_m)-1):
    segment.append([verts_m[i],verts_m[i+1]])
    
# finding slope of all the segments
def slope(x1,y1,x2,y2):
    if (y2>y1):
        return (((y2-y1)/(x2-x1)))
    elif (y1>y2):
        return (((y1-y2)/(x1-x2)))



    
def rotate(l,n):
    return l[n:]+l[:n]
    # M,PHI1, PERPANGLE1, SEGMENT LENGTH
m=[]                                    # SLOPE
phi=[]                                 # SEGMENT ANGLE
perpangle=[]                           # PERPENDICULAR ANGLE TO SEGMENT
segmentlength=[]                        # SEGMENT LENGTH
quadrant=[]
for i in range(len(verts_m)):
    if i==0:
        x1 = verts_m[i][0]
        y1 = verts_m[i][1]
        x2 = verts_m[i+1][0]
        y2 = verts_m[i+1][1]
    elif i>0:
        x1 = verts_m[i-1][0]
        y1 = verts_m[i-1][1]
        x2 = verts_m[i][0]
        y2 = verts_m[i][1]
    m.append(slope(x1,y1,x2,y2))
    dx=x2-x1
    dy=y2-y1
    segmentlength.append((math.sqrt(math.pow(dy,2)+math.pow(dx,2))))
    ph=math.degrees(math.atan(m[i]))
    phi.append(ph+180)
    perpangle.append(phi[i]-90)
    
# IDENTIFYING QUADRANT OF THE SEGMENT
    if dx>0 and dy>0:
        quad=1
    elif dx<0 and dy>0:
        quad=2
    elif dx<0 and dy<0:
        quad=3
    elif dx>0 and dy<0:
        quad=4
    quadrant.append(quad)
quadrant_previous=quadrant.copy()    
quadrant_previous.insert(0,quadrant_previous.pop())
# FINDING TYPER OF THE VERTEX ANGLE OF THE CONTOUR
unit_vector=[]
segment_angle=[]
vertex_type=[]
head_coords=[]
path_head_vertex=[]
xC_vh=[]
yC_vh=[]
left_vertexhead=[]
right_vertexhead=[]
front_vertexhead=[]
centrd_vertexhead=[]



left_vertexhead_cm=[]
right_vertexhead_cm=[]
front_vertexhead_cm=[]
centrd_vertexhead_cm=[]




vertshead_cornor=[]
pathhead_cornor=[]
start_x=[]
start_y=[]
end_x=[]
end_y=[]
end_temp_x=[]
end_temp_y=[]


head_c=[]
head_l=[]
head_r=[]
head_f=[]

for j in range(len(verts_m)):
    Drill_x = verts_m[j][0]
    Drill_y = verts_m[j][1]
    safe_dist = 0.064
    head_width = 0.1016
    centroid2vertex=head_width*(math.sqrt(3))/3
    p_angle = perpangle[j]
    phi_angle = phi[j]
    c_x=(((safe_dist)*(math.cos(math.radians(p_angle))))+Drill_x)
    c_y=(((safe_dist)*(math.sin(math.radians(p_angle))))+Drill_y)
    
    l_x=(((centroid2vertex)*(math.cos(math.radians(phi_angle+150))))+c_x)
    l_y=(((centroid2vertex)*(math.sin(math.radians(phi_angle+150))))+c_y)
    
    r_x=(((centroid2vertex)*(math.cos(math.radians(phi_angle+30))))+c_x)
    r_y=(((centroid2vertex)*(math.sin(math.radians(phi_angle+30))))+c_y)
    
    f_x=(((centroid2vertex)*(math.cos(math.radians(phi_angle-90))))+c_x)
    f_y=(((centroid2vertex)*(math.sin(math.radians(phi_angle-90))))+c_y)
    
    
    head_c.append((c_x,c_y))
    head_l.append((l_x,l_y))
    head_r.append((r_x,r_y))
    head_f.append((f_x,f_y))
    



    vertshead_cornor.append ([
                head_f[j], # front
                head_l[j], # left
                head_r[j], # right
                head_f[j] # ignored
                ])


    codeshead_cornor=[Path.MOVETO,
                     Path.LINETO,
                     Path.LINETO,
                     Path.CLOSEPOLY,
                     ]

    pathhead_cornor.append(Path(vertshead_cornor[j], codeshead_cornor))

    
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# patch = patches.PathPatch(path,alpha = 0.4,lw=1,facecolor="none")
# ax.add_patch(patch)
# art3d.pathpatch_2d_to_3d(patch, z=1.55, zdir="z")
# for R in range(len(pathhead_cornor)): 
#     patchhead_mdp = patches.PathPatch(pathhead_cornor[R],facecolor='none',edgecolor='g',linestyle='-',alpha = 1, lw=1.2,)
#     ax.add_patch(patchhead_mdp)
#     art3d.pathpatch_2d_to_3d(patchhead_mdp, z=1.55, zdir="z")    
#     ax.legend(loc='upper center')

# ax.set_xlim3d(0, 2.75)
# ax.set_ylim3d(0, 2.50)
# ax.set_zlim3d(0, 2.00)

# font_prop = font_manager.FontProperties(size=10)
# ax.set_xlabel('x (m)',fontproperties=font_prop)
# ax.set_ylabel('y (m)',fontproperties=font_prop)
# ax.set_zlabel('z (m)',fontproperties=font_prop)
# #plt.axis('equal')
# end = time.time()
# exe_time= end - start
# print(end - start) 
# plt.savefig('head.png', dpi=300)
# plt.show()    
  