import numpy as np
#from head_gridworld import GridWorld
#from CSP_Head import *
#from head_policy_calculation_MC_MODIFIED import *
#from mpl_toolkits.mplot3d import Axes3D
import csv


#AGENT1
episodes_head=np.load("episodes_head.npy")
policy_matrix_1=np.load("allpolicies_agent1.npy")
utility_value_1=np.load("utility_value_agent1.npy")
episodes_base_1=np.load("episodes_agent1.npy")

state_utility_1=[]
for i in range(len(utility_value_1[0])):
    state_utility_1.append(utility_value_1[12][i][:,72]) #state 37 (37-1), all actions, 2nd Goal (nO.1)

action_utility_1=[]
for x in range(len(state_utility_1[0])):
    action_utility_1.append([])
for j in range(len(state_utility_1[0])):
    for k in range(len(state_utility_1)):
        action_utility_1[j].append(state_utility_1[k][j])


with open('G8 C73_agent1_q_action_utility.csv','w',newline='') as outfile1:
    for m in range(len(action_utility_1)):
        for n in action_utility_1[m]:
                outfile1.write(str(n))
                outfile1.write("\n")
            
with open('episodes_head.csv','w',newline='') as outfile2:
    for m in episodes_head[0]:
        outfile2.write(str(m))
        outfile2.write("\n")

with open('agent1_episodes_base.csv','w',newline='') as outfile3:
    for m in episodes_base_1[1]:
        outfile3.write(str(m))
        outfile3.write("\n")

with open('agent1_base_policies.csv','w',newline='') as outfile4:
    for m in policy_matrix_1:
        outfile4.write(str(m))
        outfile4.write("\n")


#AGENT2
policy_matrix_2=np.load("allpolicies_agent2.npy")
utility_value_2=np.load("utility_value_agent2.npy")
episodes_base_2=np.load("episodes_agent2.npy")
utility_value_2=np.load("utility_value_agent2.npy")
state_utility_2=[]
for i in range(len(utility_value_2[0])):
    state_utility_2.append(utility_value_2[13][i][:,46]) #state 37 (37-1), all actions, 2nd Goal (nO.1)




action_utility_2=[]
for x in range(len(state_utility_2[0])):
    action_utility_2.append([])
for j in range(len(state_utility_2[0])):
    for k in range(len(state_utility_2)):
        action_utility_2[j].append(state_utility_2[k][j])


with open('a2 G14 C47_agent2_q_action_utility.csv','w',newline='') as outfile1:
    for m in range(len(action_utility_1)):
        for n in action_utility_2[m]:
                outfile1.write(str(n))
                outfile1.write("\n")

with open('agent2_q_action_utility.csv','w',newline='') as outfile5:
    for m in range(len(action_utility_2)):
        for n in action_utility_2[m]:
                outfile5.write(str(n))
                outfile5.write("\n")
            
with open('agent2_episodes_base.csv','w',newline='') as outfile6:
    for m in episodes_base_2[1]:
        outfile6.write(str(m))
        outfile6.write("\n")

with open('agent2_base_policies.csv','w',newline='') as outfile7:
    for m in policy_matrix_2:
        outfile7.write(str(m))
        outfile7.write("\n")
