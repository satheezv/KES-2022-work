import numpy as np
#from transition_matrix_test import *
import time
start=time.time()
from CSP_Head import *
from TEMP_CHECK_MC import *
from segment_ch import *
import matplotlib.pyplot as plt


m_mdp=[]                                    # SLOPE
phi_mdp=[]                                 # SEGMENT ANGLE
perpangle_mdp=[]                           # PERPENDICULAR ANGLE TO SEGMENT
segmentlength_mdp=[]                        # SEGMENT LENGTH
headsOnSegment_mdp=[]
perpLineSpacing_mdp=[]
quad_mdp=[]
lefthead_mdp=[]
righthead_mdp=[]
fronthead_mdp=[]

lefthead_mdp_cm=[]
righthead_mdp_cm=[]
fronthead_mdp_cm=[]

pp=policy_array
for k in range(len(segment)):
    if k==0:
        x1 = verts[k][0]
        y1 = verts[k][1]
        x2 = verts[k+1][0]
        y2 = verts[k+1][1]
    else:
        x1 = verts[k][0]
        y1 = verts[k][1]
        x2 = verts[k-1][0]
        y2 = verts[k-1][1]
    m_mdp.append(slope(x1,y1,x2,y2))
    dx=x2-x1
    dy=y2-y1
    segmentlength_mdp.append((math.sqrt(math.pow(dy,2)+math.pow(dx,2))))
    if dx>0 and dy>0:
        quad_mdp.append(1)
    elif dx<0 and dy>0:
        quad_mdp.append(2)
    elif dx<0 and dy<0:
        quad_mdp.append(3)
    elif dx>0 and dy<0:
        quad_mdp.append(4)

    perpLineSpacing_mdp.append((4+HH_dist[k]))
    phi_mdp.append(math.degrees(math.atan(m[k])))
    perpangle_mdp.append(phi[k]-(90))
    headsOnSegment_mdp.append(Num_Heads[k])

"""  conversion of final policy(P) into head location"""
xxL=[]
yyL=[]
for v in range(len(verts)):
    xxL.append(verts[v][0])
    yyL.append(verts[v][1])
"""
(xL,yL)=(33.231377425495786, 34.37745322860093)
(xL,yL)=(58.759577061615545, 71.11304032614986)
(xL,yL)=(79.42267718047735, 48.16391954087808)
(xL,yL)=(64.84358858616812, 23.05517882007026)
"""
# reward calculation
reward=[]
for rew in range(len(segment)):
    reward.append([])

xL=[]
yL=[]
xR=[]
yR=[]
xF=[]
yF=[]
count=0
for seg in range(len(segment)):
    #xL.append(xxL[seg])
    #yL.append(yyL[seg])                                    #"""important"""     ###### get right and front vertices of 1st heads of each segment and enter along 
    initial_location=0
    initial_policy=pp[seg][initial_location]
    current_location=initial_location
    current_policy=initial_policy
    NH=Num_Heads[seg]
    for head in range(NH):
        if head==0:
            if quad_mdp[seg]==2 or quad_mdp[seg]==3:
                    xL.append(xxL[seg]-((4.33+HH_dist[seg])*math.cos(math.radians(phi_mdp[seg])))-(0.0787402*math.cos(math.radians(perpangle[seg]))))
                    yL.append(yyL[seg]-((4.33+HH_dist[seg])*math.sin(math.radians(phi_mdp[seg])))-(0.0787402*math.sin(math.radians(perpangle[seg]))))
                    xR.append((xL[count]-(4*math.cos(math.radians(phi_mdp[seg])))))
                    yR.append((yL[count]-(4*math.sin(math.radians(phi_mdp[seg])))))
                    xF.append((xL[count]-(4*math.cos(math.radians(phi_mdp[seg]-(60))))))
                    yF.append((yL[count]-(4*math.sin(math.radians(phi_mdp[seg]-(60))))))
                    
            if quad_mdp[seg]==1 or quad_mdp[seg]==4:
                    xL.append(xxL[seg]+((4.33+HH_dist[seg])*math.cos(math.radians(phi_mdp[seg])))+(0.0787402*math.cos(math.radians(perpangle[seg]))))
                    yL.append(yyL[seg]+((4.33+HH_dist[seg])*math.sin(math.radians(phi_mdp[seg])))+(0.0787402*math.sin(math.radians(perpangle[seg]))))
                    xR.append((xL[count]+(4*math.cos(math.radians(phi_mdp[seg])))))
                    yR.append((yL[count]+(4*math.sin(math.radians(phi_mdp[seg])))))
                    xF.append((xL[count]+(4*math.cos(math.radians(phi_mdp[seg]-(60))))))
                    yF.append((yL[count]+(4*math.sin(math.radians(phi_mdp[seg]-(60))))))
                    
        if head>0:
            if current_location in range(0,NH+1):
              if current_policy==0:
                next_location=current_location+1
                next_policy=pp[seg][next_location]
                if quad_mdp[seg]==2 or quad_mdp[seg]==3:
                    xL.append(xL[count]-((4+HH_dist[seg])*math.cos(math.radians(phi_mdp[seg]))))
                    yL.append(yL[count]-((4+HH_dist[seg])*math.sin(math.radians(phi_mdp[seg]))))
                    xR.append((xL[count+1]-(4*math.cos(math.radians(phi_mdp[seg])))))
                    yR.append((yL[count+1]-(4*math.sin(math.radians(phi_mdp[seg])))))
                    xF.append((xL[count+1]-(4*math.cos(math.radians(phi_mdp[seg]-(60))))))
                    yF.append((yL[count+1]-(4*math.sin(math.radians(phi_mdp[seg]-(60))))))
                    count+=1
                if quad_mdp[seg]==1 or quad_mdp[seg]==4:
                    xL.append(xL[count]+((4+HH_dist[seg])*math.cos(math.radians(phi_mdp[seg]))))
                    yL.append(yL[count]+((4+HH_dist[seg])*math.sin(math.radians(phi_mdp[seg]))))
                    xR.append((xL[count+1]+(4*math.cos(math.radians(phi_mdp[seg])))))
                    yR.append((yL[count+1]+(4*math.sin(math.radians(phi_mdp[seg])))))
                    xF.append((xL[count+1]+(4*math.cos(math.radians(phi_mdp[seg]-(60))))))
                    yF.append((yL[count+1]+(4*math.sin(math.radians(phi_mdp[seg]-(60))))))
                    count+=1
                reward[seg].append(-0.5)
                
              if current_policy==1:
                next_location=current_location+1+NH
                next_policy=pp[seg][next_location]
                if quad_mdp[seg]==2 or quad_mdp[seg]==3:
                    xL.append(xL[count]-((4+HH_dist[seg])*math.cos(math.radians(phi_mdp[seg])))-(0.314961*math.cos(math.radians(perpangle[seg]))))
                    yL.append(yL[count]-((4+HH_dist[seg])*math.sin(math.radians(phi_mdp[seg])))-(0.314961*math.sin(math.radians(perpangle[seg]))))
                    xR.append((xL[count+1]-(4*math.cos(math.radians(phi_mdp[seg])))))
                    yR.append((yL[count+1]-(4*math.sin(math.radians(phi_mdp[seg])))))
                    xF.append((xL[count+1]-(4*math.cos(math.radians(phi_mdp[seg]-(60))))))
                    yF.append((yL[count+1]-(4*math.sin(math.radians(phi_mdp[seg]-(60))))))
                    count+=1
                if quad_mdp[seg]==1 or quad_mdp[seg]==4:
                    xL.append(xL[count]+((4+HH_dist[seg])*math.cos(math.radians(phi_mdp[seg])))+(0.314961*math.cos(math.radians(perpangle[seg]))))
                    yL.append(yL[count]+((4+HH_dist[seg])*math.sin(math.radians(phi_mdp[seg])))+(0.314961*math.sin(math.radians(perpangle[seg]))))
                    xR.append((xL[count+1]+(4*math.cos(math.radians(phi_mdp[seg])))))
                    yR.append((yL[count+1]+(4*math.sin(math.radians(phi_mdp[seg])))))
                    xF.append((xL[count+1]+(4*math.cos(math.radians(phi_mdp[seg]-(60))))))
                    yF.append((yL[count+1]+(4*math.sin(math.radians(phi_mdp[seg]-(60))))))
                    count+=1
                reward[seg].append(-0.3)
              if current_policy==2:
                next_location=current_location+1+(2*NH)
                next_policy=pp[seg][next_location]
                if quad_mdp[seg]==2 or quad_mdp[seg]==3:
                    xL.append(xL[count]-((4+HH_dist[seg])*math.cos(math.radians(phi_mdp[seg])))-(0.708661*math.cos(math.radians(perpangle[seg]))))
                    yL.append(yL[count]-((4+HH_dist[seg])*math.sin(math.radians(phi_mdp[seg])))-(0.708661*math.sin(math.radians(perpangle[seg]))))
                    xR.append((xL[count+1]-(4*math.cos(math.radians(phi_mdp[seg])))))
                    yR.append((yL[count+1]-(4*math.sin(math.radians(phi_mdp[seg])))))
                    xF.append((xL[count+1]-(4*math.cos(math.radians(phi_mdp[seg]-(60))))))
                    yF.append((yL[count+1]-(4*math.sin(math.radians(phi_mdp[seg]-(60))))))
                    count+=1
                if quad_mdp[seg]==1 or quad_mdp[seg]==4:
                    xL.append(xL[count]+((4+HH_dist[seg])*math.cos(math.radians(phi_mdp[seg])))+(0.708661*math.cos(math.radians(perpangle[seg]))))
                    yL.append(yL[count]+((4+HH_dist[seg])*math.sin(math.radians(phi_mdp[seg])))+(0.708661*math.sin(math.radians(perpangle[seg]))))
                    xR.append((xL[count+1]+(4*math.cos(math.radians(phi_mdp[seg])))))
                    yR.append((yL[count+1]+(4*math.sin(math.radians(phi_mdp[seg])))))
                    xF.append((xL[count+1]+(4*math.cos(math.radians(phi_mdp[seg]-(60))))))
                    yF.append((yL[count+1]+(4*math.sin(math.radians(phi_mdp[seg]-(60))))))
                    count+=1
                reward[seg].append(-0.1)
                    
            if current_location in range(NH+1,(2*NH)+1):
              if current_policy==0:
                next_location=current_location+1-(NH)
                next_policy=pp[seg][next_location]
                if quad_mdp[seg]==2 or quad_mdp[seg]==3:
                    xL.append(xL[count]-((4+HH_dist[seg])*math.cos(math.radians(phi_mdp[seg])))-(-0.314961*math.cos(math.radians(perpangle[seg]))))
                    yL.append(yL[count]-((4+HH_dist[seg])*math.sin(math.radians(phi_mdp[seg])))-(-0.314961*math.sin(math.radians(perpangle[seg]))))
                    xR.append((xL[count+1]-(4*math.cos(math.radians(phi_mdp[seg])))))
                    yR.append((yL[count+1]-(4*math.sin(math.radians(phi_mdp[seg])))))
                    xF.append((xL[count+1]-(4*math.cos(math.radians(phi_mdp[seg]-(60))))))
                    yF.append((yL[count+1]-(4*math.sin(math.radians(phi_mdp[seg]-(60))))))
                    count+=1
                if quad_mdp[seg]==1 or quad_mdp[seg]==4:
                    xL.append(xL[count]+((4+HH_dist[seg])*math.cos(math.radians(phi_mdp[seg])))+(-0.314961*math.cos(math.radians(perpangle[seg]))))
                    yL.append(yL[count]+((4+HH_dist[seg])*math.sin(math.radians(phi_mdp[seg])))+(-0.314961*math.sin(math.radians(perpangle[seg]))))
                    xR.append((xL[count+1]+(4*math.cos(math.radians(phi_mdp[seg])))))
                    yR.append((yL[count+1]+(4*math.sin(math.radians(phi_mdp[seg])))))
                    xF.append((xL[count+1]+(4*math.cos(math.radians(phi_mdp[seg]-(60))))))
                    yF.append((yL[count+1]+(4*math.sin(math.radians(phi_mdp[seg]-(60))))))
                    count+=1
                reward[seg].append(-0.5)
                
              if current_policy==1:
                next_location=current_location+1
                next_policy=pp[seg][next_location]
                if quad_mdp[seg]==2 or quad_mdp[seg]==3:
                    xL.append(xL[count]-((4+HH_dist[seg])*math.cos(math.radians(phi_mdp[seg]))))
                    yL.append(yL[count]-((4+HH_dist[seg])*math.sin(math.radians(phi_mdp[seg]))))
                    xR.append((xL[count+1]-(4*math.cos(math.radians(phi_mdp[seg])))))
                    yR.append((yL[count+1]-(4*math.sin(math.radians(phi_mdp[seg])))))
                    xF.append((xL[count+1]-(4*math.cos(math.radians(phi_mdp[seg]-(60))))))
                    yF.append((yL[count+1]-(4*math.sin(math.radians(phi_mdp[seg]-(60))))))
                    count+=1
                if quad_mdp[seg]==1 or quad_mdp[seg]==4:
                    xL.append(xL[count]+((4+HH_dist[seg])*math.cos(math.radians(phi_mdp[seg]))))
                    yL.append(yL[count]+((4+HH_dist[seg])*math.sin(math.radians(phi_mdp[seg]))))
                    xR.append((xL[count+1]+(4*math.cos(math.radians(phi_mdp[seg])))))
                    yR.append((yL[count+1]+(4*math.sin(math.radians(phi_mdp[seg])))))
                    xF.append((xL[count+1]+(4*math.cos(math.radians(phi_mdp[seg]-(60))))))
                    yF.append((yL[count+1]+(4*math.sin(math.radians(phi_mdp[seg]-(60))))))
                    count+=1
                reward[seg].append(-0.3)
                
              if current_policy==2:
                next_location=current_location+1+NH
                next_policy=pp[seg][next_location]
                if quad_mdp[seg]==2 or quad_mdp[seg]==3:
                    xL.append(xL[count]-((4+HH_dist[seg])*math.cos(math.radians(phi[seg])))-(0.314961*math.cos(math.radians(perpangle[seg]))))
                    yL.append(yL[count]-((4+HH_dist[seg])*math.sin(math.radians(phi[seg])))-(0.314961*math.sin(math.radians(perpangle[seg]))))
                    xR.append((xL[count+1]-(4*math.cos(math.radians(phi_mdp[seg])))))
                    yR.append((yL[count+1]-(4*math.sin(math.radians(phi_mdp[seg])))))
                    xF.append((xL[count+1]-(4*math.cos(math.radians(phi_mdp[seg]-(60))))))
                    yF.append((yL[count+1]-(4*math.sin(math.radians(phi_mdp[seg]-(60))))))
                    count+=1
                if quad_mdp[seg]==1 or quad_mdp[seg]==4:
                    xL.append(xL[count]+((4+HH_dist[seg])*math.cos(math.radians(phi_mdp[seg])))+(0.314961*math.cos(math.radians(perpangle[seg]))))
                    yL.append(yL[count]+((4+HH_dist[seg])*math.sin(math.radians(phi_mdp[seg])))+(0.314961*math.sin(math.radians(perpangle[seg]))))
                    xR.append((xL[count+1]+(4*math.cos(math.radians(phi_mdp[seg])))))
                    yR.append((yL[count+1]+(4*math.sin(math.radians(phi_mdp[seg])))))
                    xF.append((xL[count+1]+(4*math.cos(math.radians(phi_mdp[seg]-(60))))))
                    yF.append((yL[count+1]+(4*math.sin(math.radians(phi_mdp[seg]-(60))))))
                    count+=1
                reward[seg].append(-0.1)
                
            if current_location in range((2*NH)+1,(3*NH)+1):
              if current_policy==0:
                next_location=current_location+1-(2*NH)
                next_policy=pp[seg][next_location]
                if quad_mdp[seg]==2 or quad_mdp[seg]==3:
                    xL.append(xL[count]-((4+HH_dist[seg])*math.cos(math.radians(phi_mdp[seg])))-(-0.708661*math.cos(math.radians(perpangle[seg]))))
                    yL.append(yL[count]-((4+HH_dist[seg])*math.sin(math.radians(phi_mdp[seg])))-(-0.708661*math.sin(math.radians(perpangle[seg]))))
                    xR.append((xL[count+1]-(4*math.cos(math.radians(phi_mdp[seg])))))
                    yR.append((yL[count+1]-(4*math.sin(math.radians(phi_mdp[seg])))))
                    xF.append((xL[count+1]-(4*math.cos(math.radians(phi_mdp[seg]-(60))))))
                    yF.append((yL[count+1]-(4*math.sin(math.radians(phi_mdp[seg]-(60))))))
                    count+=1
                if quad_mdp[seg]==1 or quad_mdp[seg]==4:
                    xL.append(xL[count]+((4+HH_dist[seg])*math.cos(math.radians(phi_mdp[seg])))+(-0.708661*math.cos(math.radians(perpangle[seg]))))
                    yL.append(yL[count]+((4+HH_dist[seg])*math.sin(math.radians(phi_mdp[seg])))+(-0.708661*math.sin(math.radians(perpangle[seg]))))
                    xR.append((xL[count+1]+(4*math.cos(math.radians(phi_mdp[seg])))))
                    yR.append((yL[count+1]+(4*math.sin(math.radians(phi_mdp[seg])))))
                    xF.append((xL[count+1]+(4*math.cos(math.radians(phi_mdp[seg]-(60))))))
                    yF.append((yL[count+1]+(4*math.sin(math.radians(phi_mdp[seg]-(60))))))
                    count+=1
                reward[seg].append(-0.5)
                
              if current_policy==1:
                next_location=current_location+1-(NH)
                next_policy=pp[seg][next_location]
                if quad_mdp[seg]==2 or quad_mdp[seg]==3:
                    xL.append(xL[count]-((4+HH_dist[seg])*math.cos(math.radians(phi_mdp[seg])))-(-0.314961*math.cos(math.radians(perpangle[seg]))))
                    yL.append(yL[count]-((4+HH_dist[seg])*math.sin(math.radians(phi_mdp[seg])))-(-0.314961*math.sin(math.radians(perpangle[seg]))))
                    xR.append((xL[count+1]-(4*math.cos(math.radians(phi_mdp[seg])))))
                    yR.append((yL[count+1]-(4*math.sin(math.radians(phi_mdp[seg])))))
                    xF.append((xL[count+1]-(4*math.cos(math.radians(phi_mdp[seg]-(60))))))
                    yF.append((yL[count+1]-(4*math.sin(math.radians(phi_mdp[seg]-(60))))))
                    count+=1
                if quad_mdp[seg]==1 or quad_mdp[seg]==4:
                    xL.append(xL[count]+((4+HH_dist[seg])*math.cos(math.radians(phi_mdp[seg])))+(-0.314961*math.cos(math.radians(perpangle[seg]))))
                    yL.append(yL[count]+((4+HH_dist[seg])*math.sin(math.radians(phi_mdp[seg])))+(-0.314961*math.sin(math.radians(perpangle[seg]))))
                    xR.append((xL[count+1]+(4*math.cos(math.radians(phi_mdp[seg])))))
                    yR.append((yL[count+1]+(4*math.sin(math.radians(phi_mdp[seg])))))
                    xF.append((xL[count+1]+(4*math.cos(math.radians(phi_mdp[seg]-(60))))))
                    yF.append((yL[count+1]+(4*math.sin(math.radians(phi_mdp[seg]-(60))))))
                    count+=1
                reward[seg].append(-0.3)
                
              if current_policy==2:
                next_location=current_location+1
                next_policy=pp[seg][next_location]
                if quad_mdp[seg]==2 or quad_mdp[seg]==3:
                    xL.append(xL[count]-((4+HH_dist[seg])*math.cos(math.radians(phi_mdp[seg]))))
                    yL.append(yL[count]-((4+HH_dist[seg])*math.sin(math.radians(phi_mdp[seg]))))
                    xR.append((xL[count+1]-(4*math.cos(math.radians(phi_mdp[seg])))))
                    yR.append((yL[count+1]-(4*math.sin(math.radians(phi_mdp[seg])))))
                    xF.append((xL[count+1]-(4*math.cos(math.radians(phi_mdp[seg]-(60))))))
                    yF.append((yL[count+1]-(4*math.sin(math.radians(phi_mdp[seg]-(60))))))
                    count+=1
                if quad_mdp[seg]==1 or quad_mdp[seg]==4:
                    xL.append(xL[count]+((4+HH_dist[seg])*math.cos(math.radians(phi_mdp[seg]))))
                    yL.append(yL[count]+((4+HH_dist[seg])*math.sin(math.radians(phi_mdp[seg]))))
                    xR.append((xL[count+1]+(4*math.cos(math.radians(phi_mdp[seg])))))
                    yR.append((yL[count+1]+(4*math.sin(math.radians(phi_mdp[seg])))))
                    xF.append((xL[count+1]+(4*math.cos(math.radians(phi_mdp[seg]-(60))))))
                    yF.append((yL[count+1]+(4*math.sin(math.radians(phi_mdp[seg]-(60))))))
                    count+=1
                reward[seg].append(-0.1)
                
            current_location=next_location
            current_policy=next_policy
        
            print(next_location)
            print(next_policy)
    count+=1   
    print("^^segment"+str(seg+1))

    # combining heads
for comb_heads in range(len(xL)):
    lefthead_mdp.append((xL[comb_heads],yL[comb_heads]))
    righthead_mdp.append((xR[comb_heads],yR[comb_heads]))
    fronthead_mdp.append((xF[comb_heads],yF[comb_heads]))


    lefthead_mdp_cm.append((0.0254*xL[comb_heads],0.0254*yL[comb_heads]))
    righthead_mdp_cm.append((0.0254*xR[comb_heads],0.0254*yR[comb_heads]))
    fronthead_mdp_cm.append((0.0254*xF[comb_heads],0.0254*yF[comb_heads]))
"""
# automatically inserting vertex heads into the intermediate heads
for i in range(len(left_vertexhead)):
    if i==0:
        lefthead_mdp.insert(i,left_vertexhead[i])
        righthead_mdp.insert(i,right_vertexhead[i])
        fronthead_mdp.insert(i,front_vertexhead[i])
        
             
    elif i>0:
        lefthead_mdp.insert((sum(Num_Heads[0:(i)]))+i,left_vertexhead[i])
        righthead_mdp.insert((sum(Num_Heads[0:(i)]))+i,right_vertexhead[i])
        fronthead_mdp.insert((sum(Num_Heads[0:(i)]))+i,front_vertexhead[i])
"""
# Vertex 1
lefthead_mdp.insert(0,left_vertexhead[0])
righthead_mdp.insert(0,right_vertexhead[0])
fronthead_mdp.insert(0,front_vertexhead[0])

lefthead_mdp_cm.insert(0,left_vertexhead_cm[0])
righthead_mdp_cm.insert(0,right_vertexhead_cm[0])
fronthead_mdp_cm.insert(0,front_vertexhead_cm[0])

lefthead_mdp.insert(1,left_vertexhead[1])
righthead_mdp.insert(1,right_vertexhead[1])
fronthead_mdp.insert(1,front_vertexhead[1])

lefthead_mdp_cm.insert(1,left_vertexhead_cm[1])
righthead_mdp_cm.insert(1,right_vertexhead_cm[1])
fronthead_mdp_cm.insert(1,front_vertexhead_cm[1])
# Vertex 2
lefthead_mdp.insert(5,left_vertexhead[2])
righthead_mdp.insert(5,right_vertexhead[2])
fronthead_mdp.insert(5,front_vertexhead[2])

lefthead_mdp_cm.insert(5,left_vertexhead_cm[2])
righthead_mdp_cm.insert(5,right_vertexhead_cm[2])
fronthead_mdp_cm.insert(5,front_vertexhead_cm[2])

lefthead_mdp.insert(6,left_vertexhead[3])
righthead_mdp.insert(6,right_vertexhead[3])
fronthead_mdp.insert(6,front_vertexhead[3])

lefthead_mdp_cm.insert(6,left_vertexhead_cm[3])
righthead_mdp_cm.insert(6,right_vertexhead_cm[3])
fronthead_mdp_cm.insert(6,front_vertexhead_cm[3])
# Vertex 3
lefthead_mdp.insert(10,left_vertexhead[4])
righthead_mdp.insert(10,right_vertexhead[4])
fronthead_mdp.insert(10,front_vertexhead[4])

lefthead_mdp_cm.insert(10,left_vertexhead_cm[4])
righthead_mdp_cm.insert(10,right_vertexhead_cm[4])
fronthead_mdp_cm.insert(10,front_vertexhead_cm[4])

lefthead_mdp.insert(11,left_vertexhead[5])
righthead_mdp.insert(11,right_vertexhead[5])
fronthead_mdp.insert(11,front_vertexhead[5])

lefthead_mdp_cm.insert(11,left_vertexhead_cm[5])
righthead_mdp_cm.insert(11,right_vertexhead_cm[5])
fronthead_mdp_cm.insert(11,front_vertexhead_cm[5])
# Vertex 4
lefthead_mdp.insert(15,left_vertexhead[6])
righthead_mdp.insert(15,right_vertexhead[6])
fronthead_mdp.insert(15,front_vertexhead[6])

lefthead_mdp_cm.insert(15,left_vertexhead_cm[6])
righthead_mdp_cm.insert(15,right_vertexhead_cm[6])
fronthead_mdp_cm.insert(15,front_vertexhead_cm[6])

lefthead_mdp.insert(16,left_vertexhead[7])
righthead_mdp.insert(16,right_vertexhead[7])
fronthead_mdp.insert(16,front_vertexhead[7])

lefthead_mdp_cm.insert(16,left_vertexhead_cm[7])
righthead_mdp_cm.insert(16,right_vertexhead_cm[7])
fronthead_mdp_cm.insert(16,front_vertexhead_cm[7])
# Vertex 5
lefthead_mdp.insert(20,left_vertexhead[8])
righthead_mdp.insert(20,right_vertexhead[8])
fronthead_mdp.insert(20,front_vertexhead[8])

lefthead_mdp_cm.insert(20,left_vertexhead_cm[8])
righthead_mdp_cm.insert(20,right_vertexhead_cm[8])
fronthead_mdp_cm.insert(20,front_vertexhead_cm[8])

lefthead_mdp.insert(21,left_vertexhead[9])
righthead_mdp.insert(21,right_vertexhead[9])
fronthead_mdp.insert(21,front_vertexhead[9])

lefthead_mdp_cm.insert(21,left_vertexhead_cm[9])
righthead_mdp_cm.insert(21,right_vertexhead_cm[9])
fronthead_mdp_cm.insert(21,front_vertexhead_cm[9])
# Vertex 6
lefthead_mdp.insert(25,left_vertexhead[10])
righthead_mdp.insert(25,right_vertexhead[10])
fronthead_mdp.insert(25,front_vertexhead[10])

lefthead_mdp_cm.insert(25,left_vertexhead_cm[10])
righthead_mdp_cm.insert(25,right_vertexhead_cm[10])
fronthead_mdp_cm.insert(25,front_vertexhead_cm[10])

lefthead_mdp.insert(26,left_vertexhead[11])
righthead_mdp.insert(26,right_vertexhead[11])
fronthead_mdp.insert(26,front_vertexhead[11])

lefthead_mdp_cm.insert(26,left_vertexhead_cm[11])
righthead_mdp_cm.insert(26,right_vertexhead_cm[11])
fronthead_mdp_cm.insert(26,front_vertexhead_cm[11])


centroidhead_mdp=[]
centroidhead_mdp_cm=[]
# centroid calculation
for c in range(len(lefthead_mdp)):
    xC=((1/3)*(lefthead_mdp[c][0]+righthead_mdp[c][0]+fronthead_mdp[c][0]))
    yC=((1/3)*(lefthead_mdp[c][1]+righthead_mdp[c][1]+fronthead_mdp[c][1]))
    centroidhead_mdp.append((xC,yC))



    xC_cm=((1/3)*(lefthead_mdp_cm[c][0]+righthead_mdp_cm[c][0]+fronthead_mdp_cm[c][0]))
    yC_cm=((1/3)*(lefthead_mdp_cm[c][1]+righthead_mdp_cm[c][1]+fronthead_mdp_cm[c][1]))
    centroidhead_mdp_cm.append((xC_cm,yC_cm))
    
pathhead_mdp=[]
vertshead_mdp=[]
for draw in range(len(lefthead_mdp)):
    vertshead_mdp.append ([
                fronthead_mdp_cm[draw], # front
                lefthead_mdp_cm[draw], # left
                righthead_mdp_cm[draw], # right
                fronthead_mdp_cm[draw] # ignored
                ])


    codeshead_mdp=[Path.MOVETO,
                     Path.LINETO,
                     Path.LINETO,
                     Path.CLOSEPOLY,
                     ]

    pathhead_mdp.append(Path(vertshead_mdp[draw], codeshead_mdp))
   


# reward even and odd
reward=np.concatenate(reward)
head1=[]
head2=[]
for evod in range(len(reward)):
     # Loop to find even, odd Sum 
        if evod % 2 == 0: 
            head2.append(reward[evod])
        else: 
            head1.append(reward[evod])

"""
#plotting rewards of Head 1 and Head 2
steps=[1,2,3,4,5,6,7,8]
plt.figure()
y=head2
z=head1
plt.plot( steps, y, marker='o', linestyle='-',color='b', label='Head 2')
plt.plot( steps, z, marker='o', linestyle=':',color='g', label='Head 1')
plt.xlabel('steps',fontsize=17)
plt.ylabel('rewards',fontsize=17)
    #plt.title("Iteration : "+str(px+1))
    #plt.suptitle('Segment : '+str(pol+1), fontsize=16)
    #plt.title('Segment: '+str(m+1),loc='left',color='blue')
plt.legend(loc='bottom left')
plt.show()
"""
end=time.time()-start
plt.show()



"""            
# manually inserting vertex heads into the intermediate heads


"""
