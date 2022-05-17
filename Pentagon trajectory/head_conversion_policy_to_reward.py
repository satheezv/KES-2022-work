import numpy as np
#from transition_matrix_test import *
from CSP_Head import *
from TEMP_CHECK_MC import *
from segment import *
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
pp=policy_array
for k in range(len(segment)):
    x1 = verts[k][0]
    y1 = verts[k][1]
    x2 = verts[k+1][0]
    y2 = verts[k+1][1]
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

    perpLineSpacing_mdp.append((5+HH_dist[k]))
    phi_mdp.append(math.degrees(math.atan(m[k])))
    perpangle_mdp.append(phi[k]-(90))
    headsOnSegment_mdp.append(Num_Heads[k])

"""  conversion of final policy(P) into head location"""

xxL=[verts[0][0],verts[1][0],verts[2][0],verts[3][0]]
yyL=[verts[0][1],verts[1][1],verts[2][1],verts[3][1]]
"""
(xL,yL)=(33.231377425495786, 34.37745322860093)
(xL,yL)=(58.759577061615545, 71.11304032614986)
(xL,yL)=(79.42267718047735, 48.16391954087808)
(xL,yL)=(64.84358858616812, 23.05517882007026)
"""
# reward calculation
reward=[]
for rew in range(len(Num_Heads)):
    reward.append([])

xL=[]
yL=[]
xR=[]
yR=[]
xF=[]
yF=[]
count=0
for seg in range(len(Num_Heads)):
    #xL.append(xxL[seg])
    #yL.append(yyL[seg])                                    #"""important"""     ###### get right and front vertices of 1st heads of each segment and enter along 
    initial_location=0
    initial_policy=pp[seg][initial_location]
    current_location=initial_location
    current_policy=initial_policy
    NH=Num_Heads[seg]
    for head in range(NH):
        if head==0:
            if current_policy==0:
                next_location=current_location+1
                next_policy=pp[seg][next_location]
                count+=1
                reward[seg].append(-0.5)
                
            if current_policy==1:
                next_location=current_location+1+NH
                next_policy=pp[seg][next_location]
                count+=1
                reward[seg].append(-0.3)
            if current_policy==2:
                next_location=current_location+1+(2*NH)
                next_policy=pp[seg][next_location]
                count+=1
                reward[seg].append(-0.1)
            current_location=next_location
            current_policy=next_policy
        
            print(next_location)
            print(next_policy)
            
        if head>0:
            if current_location in range(0,NH):
              if current_policy==0:
                next_location=current_location+1
                next_policy=pp[seg][next_location]
                count+=1
                reward[seg].append(-0.5)
                
              if current_policy==1:
                next_location=current_location+1+NH
                next_policy=pp[seg][next_location]
                count+=1
                reward[seg].append(-0.3)
              if current_policy==2:
                next_location=current_location+1+(2*NH)
                next_policy=pp[seg][next_location]
                count+=1
                reward[seg].append(-0.1)
                
            if current_location==NH:
              if current_policy==0:
                next_location=current_location+1
                next_policy=pp[seg][next_location]
                count+=1
                reward[seg].append(1.4)
                
              if current_policy==1:
                next_location=current_location+1+NH
                next_policy=pp[seg][next_location]
                count+=1
                reward[seg].append(1.2)
              if current_policy==2:
                next_location=current_location+1+(2*NH)
                next_policy=pp[seg][next_location]
                count+=1
                reward[seg].append(1.0)
                    
            if current_location in range(NH+1,(2*NH)):
              if current_policy==0:
                next_location=current_location+1-(NH)
                next_policy=pp[seg][next_location]
                count+=1
                reward[seg].append(-0.5)
                
              if current_policy==1:
                next_location=current_location+1
                next_policy=pp[seg][next_location]
                count+=1
                reward[seg].append(-0.3)
                
              if current_policy==2:
                next_location=current_location+1+NH
                next_policy=pp[seg][next_location]
                count+=1
                reward[seg].append(-0.1)

            if current_location==2*NH:
              if current_policy==0:
                next_location=current_location+1-(NH)
                next_policy=pp[seg][next_location]
                count+=1
                reward[seg].append(1.4)
                
              if current_policy==1:
                next_location=current_location+1
                next_policy=pp[seg][next_location]
                count+=1
                reward[seg].append(1.2)
                
              if current_policy==2:
                next_location=current_location+1+NH
                next_policy=pp[seg][next_location]
                count+=1
                reward[seg].append(1.0)
                
                
            if current_location in range((2*NH)+1,(3*NH)-1):
              if current_policy==0:
                next_location=current_location+1-(2*NH)
                next_policy=pp[seg][next_location]
                count+=1
                reward[seg].append(-0.5)
                
              if current_policy==1:
                next_location=current_location+1-(NH)
                next_policy=pp[seg][next_location]
                count+=1
                reward[seg].append(-0.3)
                
              if current_policy==2:
                next_location=current_location+1
                next_policy=pp[seg][next_location]
                count+=1
                reward[seg].append(-0.1)

            if current_location==3*NH:
              if current_policy==0:
                next_location=current_location+1-(2*NH)
                next_policy=pp[seg][next_location]
                count+=1
                reward[seg].append(1.4)
                
              if current_policy==1:
                next_location=current_location+1-(NH)
                next_policy=pp[seg][next_location]
                count+=1
                reward[seg].append(1.2)
                
              if current_policy==2:
                next_location=current_location+1
                next_policy=pp[seg][next_location]
                count+=1
                reward[seg].append(1.0)
                
            current_location=next_location
            current_policy=next_policy
        
            print(next_location)
            print(next_policy)
    count+=1   
    print("^^segment"+str(seg+1))

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

plt.show()
