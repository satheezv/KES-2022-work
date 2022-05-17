import numpy as np
#from gridworld import GridWorld
from CSP_Head import *
from policy_calculation_MC_MODIFIED import *
from mpl_toolkits.mplot3d import Axes3D
import csv

with open('test1.csv','w',newline='') as outfile1:
    for entries in e_list3[0]:
            outfile1.write(str(entries))
            outfile1.write("\n")
            
    for entries in ret_val2[0]:
            outfile1.write(str(entries))
            outfile1.write("\n")

with open('test2.csv','w',newline='') as outfile2:
    for entries in e_list3[1]:
            outfile2.write(str(entries))
            outfile2.write("\n")
            
    for entries in ret_val2[1]:
            outfile2.write(str(entries))
            outfile2.write("\n")

with open('test3.csv','w',newline='') as outfile3:
    for entries in e_list3[2]:
            outfile3.write(str(entries))
            outfile3.write("\n")
            
    for entries in ret_val2[2]:
            outfile3.write(str(entries))
            outfile3.write("\n")

with open('test4.csv','w',newline='') as outfile4:
    for entries in e_list3[3]:
            outfile4.write(str(entries))
            outfile4.write("\n")
            
    for entries in ret_val2[3]:
            outfile4.write(str(entries))
            outfile4.write("\n")
          
