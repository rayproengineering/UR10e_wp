#!/usr/bin/env python3
import requests
import lkh
import tsplib95
import os
import subprocess
import numpy as np

def write_problem_no_edge_constraints_file(weight_matrix,problem_file_name):
    tsp_file = open(problem_file_name + ".tsp",'w')
    tsp_file.write("NAME: "+problem_file_name+"\n")
    tsp_file.write("COMMENT : Welding Problem\n")
    tsp_file.write("TYPE : TSP\n")
    weight_martix_dim=len(weight_matrix)
    tsp_file.write("DIMENSION :"+str(weight_martix_dim)+"\n")
    tsp_file.write("EDGE_WEIGHT_TYPE : EXPLICIT\n")
    tsp_file.write("EDGE_WEIGHT_FORMAT: FULL_MATRIX \n")
    tsp_file.write("EDGE_WEIGHT_SECTION \n")
    for i in range(len(weight_matrix)):
        for j in range(len(weight_matrix)):       
            tsp_file.write(str(int(weight_matrix[i][j])) + "\t")
           
        # Entering line spacing to start the next row on the next line
        tsp_file.write("\n")
    # tsp_file.write(str(int(-1))+'\n') 
    tsp_file.write("EOF\n")


def solve_tour_tsp(weight_matrix,trials,runs):
    problem_file_name='joint_tsp2'
    write_problem_no_edge_constraints_file(problem_file_name=problem_file_name,weight_matrix=weight_matrix)
    
    # if edge_constraints==True:
    #     write_problem_edge_constraints_file(problem_file_name=problem_file_name,weight_matrix=weight_matrix)
    
    with open('joint_tsp2'+'.tsp') as f:
        p = f.read()
    problem = lkh.LKHProblem.parse(p)
    print(problem)
    solver_path = '/home/rylab/LKH-3.0.6/LKH'
    solution=lkh.solve(solver_path, problem=problem, max_trials=trials, runs=runs)[0]
    solution=np.array(solution)-1
    return solution

