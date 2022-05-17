# -*- coding: utf-8 -*-
"""
Created on Wed May  8 18:45:47 2019

@author: Admin
"""
import numpy as np
from coordinates import *

def next_pos(action):
    if action == 1:
        np1=current_position[0]-1,current_position[1]
    elif action == 2:
        np1=current_position[0]-1,current_position[1]+1
    elif action == 3:
        np1=current_position[0],current_position[1]+1
    elif action == 4:
        np1=current_position[0]+1,current_position[1]+1
    elif action == 5:
        np1=current_position[0]+1,current_position[1]
    elif action == 6:
        np1=current_position[0]+1,current_position[1]-1
    elif action == 7:
        np1=current_position[0],current_position[1]-1
    elif action == 8:
        np1=current_position[0]-1,current_position[1]-1
    elif action == -1:
        np1=current_position[0],current_position[1]
    return np1

def odd_even(location):
    if location % 2==0:
        movement="even-LT "   #even-lower triangle
        print ("current_location is " + str(current_location) + " (even)")
    elif location % 2!=0:
        movement="odd-UT "    #odd-upper triangle
        print ("current_location is " + str(current_location) + " (odd)")
    return movement
home=[0,0]
agent2_positions=[]
agent2_actions=[]
agent2_locations=[]
agent2_movements=[]
agent2_policies=np.load("allpolicies_agent2.npy")
agent2_goals=np.load("goals_agent2.npy")
agent2_goals=agent2_goals.tolist()
print (agent2_goals)
for j in range (len(agent2_policies)):
    policy= agent2_policies[j] 
    current_position=home
    agent2_positions.append([])
    agent2_actions.append([])
    agent2_locations.append([])
    agent2_movements.append([])    
    iteration=1
    while True:
        print ("step : " +str(iteration))
        if iteration==1:
            agent2_positions[j].append(current_position)
        action=int(policy[current_position[0],current_position[1]])
        current_location=(int(current_position[0])*13)+int(current_position[1])+1
        agent2_locations[j].append(current_location)
        print ("current position is " +str(current_position))
        movement=odd_even(current_location)
        agent2_movements[j].append(movement)
        print ("movement is " + str(movement) + "act " + str(action))
        #print "action is " + str(action)    
        next_position=next_pos(action)
        next_location=(int(next_position[0])*13)+int(next_position[1])+1
        print ("next_location is " + str(next_location))
        print ("next position is " +str(next_position))
        agent2_actions[j].append(action) 
        if int(policy[next_position])==-1:
            break
        current_position=next_position
        agent2_positions[j].append(current_position)
        iteration+=1
    home=agent2_goals[j]

###### this needs to be changed
n=len(agent2_positions)
agent2_positions.append(agent2_positions[n-1])
agent2_locations.append(agent2_locations[n-1])

# base centroids coordinates calculation 
agent2_intermediate_centroid_x=[] 
agent2_intermediate_centroid_y=[] 
agent2_goal_centroid_x=[] 
agent2_goal_centroid_y=[] 
agent2_intermediate_locations=[]
agent2_goal_locations=[]
for a in range(len(agent2_locations)):
    for b in range(len(agent2_locations[a])):
        if b==0:
            if  1<=agent2_locations[a][b]<=13:
                actual_location=agent2_locations[a][b]+65
            elif  14<=agent2_locations[a][b]<=26:
                actual_location=agent2_locations[a][b]+39
            elif  27<=agent2_locations[a][b]<=39:
                actual_location=agent2_locations[a][b]+13
            elif  40<=agent2_locations[a][b]<=52:
                actual_location=agent2_locations[a][b]-13
            elif  53<=agent2_locations[a][b]<=65:
                actual_location=agent2_locations[a][b]-39
            elif  66<=agent2_locations[a][b]<=78:
                actual_location=agent2_locations[a][b]-65
            agent2_goal_locations.append(actual_location)
            agent2_goal_centroid_x.append(A[actual_location-1])
            agent2_goal_centroid_y.append(B[actual_location-1])
        elif b!=0:
            if  1<=agent2_locations[a][b]<=13:
                actual_location=agent2_locations[a][b]+65
            elif  14<=agent2_locations[a][b]<=26:
                actual_location=agent2_locations[a][b]+39
            elif  27<=agent2_locations[a][b]<=39:
                actual_location=agent2_locations[a][b]+13
            elif  40<=agent2_locations[a][b]<=52:
                actual_location=agent2_locations[a][b]-13
            elif  53<=agent2_locations[a][b]<=65:
                actual_location=agent2_locations[a][b]-39
            elif  66<=agent2_locations[a][b]<=78:
                actual_location=agent2_locations[a][b]-65
            agent2_intermediate_locations.append(actual_location)
            agent2_intermediate_centroid_x.append(A[actual_location-1])
            agent2_intermediate_centroid_y.append(B[actual_location-1])



