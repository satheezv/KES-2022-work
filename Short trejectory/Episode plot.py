import numpy as np
#from head_gridworld import GridWorld
#from CSP_Head import *
#from head_policy_calculation_MC_MODIFIED import *
from mpl_toolkits.mplot3d import Axes3D
import csv

ga1=np.load("goals_agent1.npy")
ga2=np.load("goals_agent2.npy")
u1=agent1_policies=np.load("utility_value_agent1.npy")
u2=agent1_policies=np.load("utility_value_agent2.npy")
e1=agent1_goals=np.load("episodes_agent1.npy")
e2=agent1_goals=np.load("episodes_agent2.npy")

with open('baseepisodeagent1.csv','w',newline='') as outfile1:      
    for entries in e1[3]:
            outfile1.write(str(entries))
            outfile1.write("\n")

with open('baseepisodeagent2.csv','w',newline='') as outfile1:       
    for entries in e2[3]:
            outfile1.write(str(entries))
            outfile1.write("\n")

#### agent 1###########
with open('baseutilityagent1position20action0.csv','w',newline='') as outfile1:                
    outfile1.write("\n")
    outfile1.write(str("ACTION 0"))
    outfile1.write("\n")
    for entries0 in range(len(u1[3])):
            outfile1.write(str(u1[3][entries0][0][20]))
            outfile1.write("\n")
            
    outfile1.write("\n")
    outfile1.write(str("ACTION 1"))
    outfile1.write("\n")
    for entries0 in range(len(u1[3])):
            outfile1.write(str(u1[3][entries0][1][20]))
            outfile1.write("\n")
            
    outfile1.write("\n")
    outfile1.write(str("ACTION 2"))
    outfile1.write("\n")
    for entries0 in range(len(u1[3])):
            outfile1.write(str(u1[3][entries0][2][20]))
            outfile1.write("\n")

    outfile1.write("\n")
    outfile1.write(str("ACTION 3"))
    outfile1.write("\n")
    for entries0 in range(len(u1[3])):
            outfile1.write(str(u1[3][entries0][3][20]))
            outfile1.write("\n")
            
    outfile1.write("\n")
    outfile1.write(str("ACTION 4"))
    outfile1.write("\n")     
    for entries0 in range(len(u1[3])):
            outfile1.write(str(u1[3][entries0][4][20]))
            outfile1.write("\n")
            
    outfile1.write("\n")
    outfile1.write(str("ACTION 5"))
    outfile1.write("\n")      
    for entries0 in range(len(u1[3])):
            outfile1.write(str(u1[3][entries0][5][20]))
            outfile1.write("\n")
            
    outfile1.write("\n")
    outfile1.write(str("ACTION 6"))
    outfile1.write("\n")    
    for entries0 in range(len(u1[3])):
            outfile1.write(str(u1[3][entries0][6][20]))
            outfile1.write("\n")
            
    outfile1.write("\n")
    outfile1.write(str("ACTION 7"))
    outfile1.write("\n")  
    for entries0 in range(len(u1[3])):
            outfile1.write(str(u1[3][entries0][7][20]))
            outfile1.write("\n")
            
    outfile1.write("\n")
    outfile1.write(str("ACTION 8"))
    outfile1.write("\n")     
    for entries0 in range(len(u1[3])):
            outfile1.write(str(u1[3][entries0][8][20]))
            outfile1.write("\n")


###### agent 2#############
with open('baseutilityagent2position17action0.csv','w',newline='') as outfile1:
    outfile1.write("\n")
    outfile1.write(str("ACTION 0"))
    outfile1.write("\n")
    for entries0 in range(len(u2[1])):
            outfile1.write(str(u2[1][entries0][0][77]))
            outfile1.write("\n")
            
    
    outfile1.write(str("ACTION 1"))
    outfile1.write("\n")
    for entries0 in range(len(u2[1])):
            outfile1.write(str(u2[1][entries0][1][77]))
            outfile1.write("\n")
            
    
    outfile1.write(str("ACTION 2"))
    outfile1.write("\n")
    for entries0 in range(len(u2[1])):
            outfile1.write(str(u2[1][entries0][2][77]))
            outfile1.write("\n")

    
    outfile1.write(str("ACTION 3"))
    outfile1.write("\n")
    for entries0 in range(len(u2[1])):
            outfile1.write(str(u2[1][entries0][3][77]))
            outfile1.write("\n")
            
    
    outfile1.write(str("ACTION 4"))
    outfile1.write("\n")     
    for entries0 in range(len(u2[1])):
            outfile1.write(str(u2[1][entries0][4][77]))
            outfile1.write("\n")
            
    
    outfile1.write(str("ACTION 5"))
    outfile1.write("\n")      
    for entries0 in range(len(u2[1])):
            outfile1.write(str(u2[1][entries0][5][77]))
            outfile1.write("\n")
            
    
    outfile1.write(str("ACTION 6"))
    outfile1.write("\n")    
    for entries0 in range(len(u2[1])):
            outfile1.write(str(u2[1][entries0][6][77]))
            outfile1.write("\n")
            
    
    outfile1.write(str("ACTION 7"))
    outfile1.write("\n")  
    for entries0 in range(len(u2[1])):
            outfile1.write(str(u2[1][entries0][7][77]))
            outfile1.write("\n")
            
    
    outfile1.write(str("ACTION 8"))
    outfile1.write("\n")     
    for entries0 in range(len(u2[1])):
            outfile1.write(str(u2[1][entries0][8][77]))
            outfile1.write("\n")
