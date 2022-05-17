import numpy as np
from head_gridworld import GridWorld
from CSP_Head import *
import time



def print_policy(policy_matrix):
    '''Print the policy using specific symbol.
    * terminal state
    ^ > v < up, right, down, left
    # obstacle
    '''
    counter = 0
    shape = policy_matrix.shape
    policy_string = ""
    for row in range(shape[0]):
        for col in range(shape[1]):
            if(policy_matrix[row,col] == -1): policy_string += " *  "            
            elif(policy_matrix[row,col] == 0): policy_string += " A1  "
            elif(policy_matrix[row,col] == 1): policy_string += " A2  "
            elif(policy_matrix[row,col] == 2): policy_string += " A3  "           
            #elif(policy_matrix[row,col] == 3): policy_string += " <  "
            #elif(np.isnan(policy_matrix[row,col])): policy_string += " #  "
            counter += 1
        policy_string += '\n'
    print(policy_string)

def get_return(state_list, gamma):
    '''Get the return for a list of action-state values.
    @return get the Return
    '''
    counter = 0
    return_value = 0
    reward_avg=[]
    for visit in state_list:
        reward = visit[2]
        return_value += reward * np.power(gamma, counter)
        reward_avg.append(reward)
        counter += 1
    return return_value, reward_avg

def update_policy(episode_list, policy_matrix, state_action_matrix):
    '''Update a policy making it greedy in respect of the state-action matrix.
    @return the updated policy
    '''
    for visit in episode_list:
        observation = visit[0]
        col = observation[1] + (observation[0]*Nh[n])
        if(policy_matrix[observation[0], observation[1]] != -1):      
            policy_matrix[observation[0], observation[1]] = \
                np.argmax(state_action_matrix[:,col])
    return policy_matrix

Nh=Num_Heads
policy_matrices=[]
policy_array=[]
iter_segment=[] #Number of heads
e_list=[]
e_list2=[]
e_list3=[]
exe_time=[]


ret_val=[]   # return value plot
ret_val2=[]
utility_val=[]
policy_plot=[]
avg_rew=[]
episodes=[]
for rv in range(len(Nh)):
    ret_val.append([])
    e_list.append([])
    ret_val2.append([])
    e_list2.append([])
    e_list3.append([])
    utility_val.append([])
    policy_plot.append([])
    avg_rew.append([])
    episodes.append([])
for n in range(len(Nh)):
    start_time = time.time()
    env = GridWorld(3, Nh[n])
    #Define the state matrix
    state_matrix = np.zeros((3,Nh[n]))
    state_matrix[0, (Nh[n]-1)] = 1
    state_matrix[1, (Nh[n]-1)] = 1
    state_matrix[2, (Nh[n]-1)] = 1
    #state_matrix[1, 1] = -1
    print("State Matrix:")
    print(state_matrix)

    #Define the reward matrix


    r1 = np.full((3,Nh[n]), -0.04)
    for i in range(Nh[n]-1):
      r1[0,i]=-.5
      r1[1,i]=-.3
      r1[2,i]=-.1
      r1[0,Nh[n]-1]=1.4
      r1[1,Nh[n]-1]=1.2
      r1[2,Nh[n]-1]=1.0
    print("Reward Matrix:")
    reward_matrix=r1
    print(reward_matrix)

    #Define the transition matrix
    transition_matrix = np.array([[1, 0, 0],
                            [0, 1, 0],
                            [0, 0, 1]])
    #Random policy
    policy_matrix = np.random.randint(low=0, high=3, size=(3, Nh[n])).astype(np.float32)
    #policy_matrix[1,1] = np.NaN #NaN for the obstacle at (1,1)
    policy_matrix[0,Nh[n]-1] = policy_matrix[1,Nh[n]-1] = policy_matrix[2,Nh[n]-1] = -1 #No action for the terminal states

    #Set the matrices in the world
    env.setStateMatrix(state_matrix)
    env.setRewardMatrix(reward_matrix)
    env.setTransitionMatrix(transition_matrix)

    state_action_matrix = np.random.random_sample((3,(3*Nh[n]))) # Q
    #init with 1.0e-10 to avoid division by zero
    running_mean_matrix = np.full((3,(3*Nh[n])), 1.0e-10) 
    gamma = 0.999
    epsilon = 0.006
    tot_epoch = 2000
    print_epoch = 10
    iteration = 0
    while True:
        epi_iteration=0
        utility_old = state_action_matrix / running_mean_matrix
        iteration += 1
        #Starting a new episode
        episode_list = list()
        #Reset and return the first observation and reward
        observation = env.reset(exploring_starts=True)
        #action = np.random.choice(4, 1)
        #action = policy_matrix[observation[0], observation[1]]
        #episode_list.append((observation, action, reward))
        is_starting = True
        for _ in range(1000):
            epi_iteration+=1
            #Take the action from the action matrix
            action = policy_matrix[observation[0], observation[1]]
            #If the episode just started then it is
                #necessary to choose a random action (exploring starts)
            if(is_starting): 
                action = np.random.randint(0, 3)
                is_starting = False      
            #Move one step in the environment and get obs and reward
            new_observation, reward, done = env.step(action)
            #Append the visit in the episode list
            episode_list.append((observation, action, reward))
            observation = new_observation
            if done: break
        episodes[n].append(epi_iteration)
        #The episode is finished, now estimating the utilities
        counter = 0
        #Checkup to identify if it is the first visit to a state
        checkup_matrix = np.zeros((3,(3*Nh[n])))
        #This cycle is the implementation of First-Visit MC.
        #For each state stored in the episode list check if it
        #is the rist visit and then estimate the return.
        #e_list.append(episode_list)
        for visit in episode_list:
            observation = visit[0]
            action = visit[1]
            col = int(observation[1] + (observation[0]*Nh[n]))
            row = int(action)
            if(checkup_matrix[row, col] == 0):
                e_list[n].append(episode_list[counter:])
                return_value, reward_avg = get_return(episode_list[counter:], gamma)
                ret_val[n].append(return_value)
                avg_rew[n].append(np.mean(reward_avg))
                running_mean_matrix[row, col] += 1
                state_action_matrix[row, col] += return_value
                checkup_matrix[row, col] = 1
            counter += 1
            e_list2[n].append(episode_list)
            e_list3[n].append(len(episode_list))
            ret_val2[n].append(return_value)  
        #Policy Update
        policy_matrix_old=policy_matrix
        policy_matrix = update_policy(episode_list, 
                                      policy_matrix, 
                                      state_action_matrix/running_mean_matrix)
        utility_new=state_action_matrix / running_mean_matrix
        delta = np.absolute(utility_new - utility_old).max()
        if delta < epsilon * (1 - gamma) / gamma: break
        #Printing
        if(iteration % print_epoch == 0):
            print("")
            print("State-Action matrix after " + str(iteration) + " iterations:") 
            print(state_action_matrix / running_mean_matrix)
            print("Policy matrix after " + str(iteration) + " iterations:") 
            print(policy_matrix)
            policy_plot[n].append(policy_matrix)
            print_policy(policy_matrix)
        uv=(state_action_matrix / running_mean_matrix)
        utility_val[n].append(uv)
         
    #Time to check the utility matrix obtained
    print("Utility matrix after " + str(iteration) + " iterations:")
    print(state_action_matrix / running_mean_matrix)
    print(policy_matrix)
    print_policy(policy_matrix)
    policy_matrices.append(policy_matrix)
    Matrix = np.array(policy_matrix)
    Array = Matrix.flatten()
    policy_array.append(Array)    
    iter_segment.append(iteration)
    exe_time.append(time.time() - start_time)
np.save("episodes_head",episodes)
