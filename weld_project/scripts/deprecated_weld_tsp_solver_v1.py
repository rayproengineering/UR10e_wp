#!/usr/bin/env python3

from distutils.file_util import move_file
from formatter import AbstractFormatter
import pyvista as pv
import  numpy as np
import networkx as nx
from networkx.algorithms import approximation
import scipy
from scipy.interpolate import make_smoothing_spline
from copy import deepcopy
import copy

import rospy
from trac_ik_python.trac_ik import IK
import moveit_commander
import geometry_msgs.msg as geometry_msgs
import sys
import moveit_msgs.msg

import shape_msgs
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import moveit.core.collision_detection

import trajectory_msgs.msg
import std_msgs.msg

import matplotlib as mpl
import matplotlib.pyplot as plt
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.msg import RobotState
import actionlib
import actionlib_msgs
import sensor_msgs.msg
from sensor_msgs.msg import JointState

import Weld_Select
from Weld_Select import Create_Welds
from Weld_Select import *
import LKH_solver
from LKH_solver import *
import MIP_solver
from MIP_solver import *
from MIP_solver import MIP_solver_edge_constraints
import math

import vlc

import edge_classes
from edge_classes import *

import helper_functions_n_objects
from helper_functions_n_objects import *





class TSP_solution:
    #*This is the higher level planner for the welder, it accepts the part-calculates costs between desired welds and finds a tour and then passes that to the IK_SOLVER class to execute it (! for the most part some of the movement code may be written in this level but may be relocated into the lower class)
    def __init__(self,IK_SOLVER=None):
        #*The IK_SOLVER passed in contains all the move it functionality which is utilized to communicate with the robot
        
        self.IK_SOLVER=IK_SOLVER

        self.control_distance=0.015
        #*This designates how close the weld tip gets to the work-piece^
        #*This are geometric constants of the original part we welded, they aided in selecting a control height for movements between tac and welding lines initially
        #TODO: It would be usefull to develop a better path finding algorithm between the points then geometric guess and checking such as RRT*
        long1=(6-(1/16))*0.0254
        long2=(6-(1/8))*0.0254
        short=(1/8)*0.0254
        plate_height=(1/8)*0.0254
        slot_height=(3)*0.0254
        self.path_height_offset=3*0.0254

        self.dwell_distance=2*np.sqrt(long2**2)/2
        #*When Dwells are initiated in a solution this is the distance that it moves away
        self.special_lengths=[long1,long2,short,plate_height,slot_height]
        self.edge_object_list=None
        # self.get_geometric_type()
        # #checked^
        # self.lock_geometric_type()

    def run_solution(self,Weld_Mesh=None,plotter=None,tac_points=None,weld_lines=None,Transformation=None):
        #*IF YOU ARE TRYING TO WORK THROUGH UNDERSTANDING MY CODE YOU SHOULD PROBABLY START HERE
        
        
        
        #*This is the solution, to run the solution you need to pass in the Weld Mesh, the weld lines and the tac points. the plotter is for plotting the solution and debugging. The transformation is the location of the weld mesh in space



        #*This bit of code was used to gather data speciffically compiling and comparing the expirmental and calculated joint angles/poses
        # self.cart_points_true=[]
        # self.cart_points_ex=[]
        # self.JA_calc=[]
        # self.JA_calc_to_cart=[]
        # self.JA_ex=[]
        # self.JA_ex_to_cart=[]
        # self.total_edge_object_list=[]
        

        self.T=Transformation
        self.plotter=plotter
        self.pose_default=None
        
        #* I need to review this meaning\|/
        self.replace_weld_edges=False


        #*This determines the amount of interpolation on the paths for movement
        self.tour_resolution=200
        self.tour_resolution_welds=5000
        #*How smooth the line is i.e. how much will it miss control points by in order to have continous derivatives
        self.smoothnesscoef=30
        
        self.Weld_Mesh_Surface=Weld_Mesh
        #*Create a dictionary containing the tac_points refrencing them by number
        self.node_dict=self.create_node_dict(tac_points)
        self.desired_points=tac_points
        self.desired_weld_lines=weld_lines

        #*Constants
        self.height_scaling_short=self.special_lengths[3]+self.path_height_offset
        self.height_scaling_long=self.special_lengths[4]+self.path_height_offset
        self.length_scaling=(np.sqrt(self.special_lengths[0]**2+self.
        special_lengths[0]**2))/2

        print('desired points')
        print(self.desired_points)
        #* Create a fully connected nodal graph from the node dictonaries keys
        self.Graph = nx.complete_graph(self.node_dict.keys())

        for n in range(len(self.desired_points)):
            #*Store the cordinates passed through the transform in the graph object for each node
            self.Graph.nodes[n]['cord']=self.Transform(self.desired_points[n])
        # nx.set_node_attributes(self.Graph,self.desired_points,'cord')
        #check if your geometric checker works real quick
        print('Current Configuration:')
        print(self.configuration_type(self.IK_SOLVER.move_group.get_current_joint_values()))
        print('Current Joint angles degrees:')
        print(np.array(self.IK_SOLVER.move_group.get_current_joint_values())*180/np.pi)
        print('Joint Names:')
        print(self.IK_SOLVER.move_group.get_joints())

        #*These booleans turn off and on how parts of the program is executed. i.e. tacking=True means a tack will be done at the end of each edge
        self.weld_tour=False
        self.tacking=True
        self.get_geometric_type()
        #*This checks for a geometric  configuration of the robot that can get to the farthest point on the part AND the closest it will kill the program if it can not be found
        #checked^
        self.lock_geometric_type()
        #*This locks that geometric configutation so that the robot remains in it throughout the welding proccess

        self.IK_SOLVER.add_weld_mesh()
        #*Self explanatory^
        #I thinkkkkk I fixed this, added shoulder lift constraints
        self.first_point()
        #*This moves the robot to what could be the first point in the tour

        #*Compute the inverse kinematics of each nodes point
        self.compute_points_IK()
        # #test this.....^ works
        self.compute_edges()
        #*Connect these points with edges storing splines between points passing through control points^

        # #we have been here
        self.solve_tac_tour(Graph=self.Graph,runs=10,trials=1000)
        self.tour=np.load('tour_tac_current.npy')
        self.tacking=True
        self.reorder_tour()
        #*re orders the solution for the current point
        self.proccess_tour()
        #*Post proccesing to make a continous joint angle solution
        # self.Send_to_robot()
        # self.Send_to_robot_measured()
        #*This second send to robot was for measuring data

        self.tacking=False

        print('Succeeded at tacking, initializing weld lines')
        self.weld_dict=self.create_weld_dict(self.desired_weld_lines)
        #*Creates a similar dictionary that containts the start and end point of each linear weld line
        
        self.add_weld_nodes()
        #*Add these new weld nodes to a networkx graph object
        
        self.weld_tour=True
        #*Boolean that turns on welding for the weld lines
        self.solve_weld_tour()
        #*This solves the weld tour^  most of the time we just load a pre-saved solution below
        np.save('weld_tour',self.tour)
        self.tour=np.load('weld_tour.npy')
        # print(self.tour)
        self.reorder_tour()
        #*Same as above
        # self.tour_edges=list(zip(self.tour,self.tour[1:]))
        # input(self.tour_edges)
        # self.plot_tour_3D()
        #*This will show the solution in pyvista^
        self.edge_object_list=None
        #*Reset the edge object list for proccessing the weld tour
        self.proccess_tour()
        self.Send_to_robot()


        #* Both of these were for data collection\|/
        # self.Send_to_robot_measured()
        # self.save_trajectory_data()

    def create_node_dict(self,point_list=None,key_list=None):
        #* This is just a method that generates the neccessary dictonaries based on the provided nodal data
        node_dict={}
        
        if type(point_list)==type(None):
            #*If the points are not provided get them from the stored cordinates in the graph
            point_list=[]
            for n in range(len(self.Graph.nodes)):
                point=self.Graph.nodes[n]['cord']
                point_list.append(point)
        else:
            iterable=range(len(point_list))

        if type(key_list)!=type(None):
            iterable=key_list
        else:
            iterable=range(len(point_list))
            #!I think there is an issue with this method as iterable is not even used will revist
        for n in range(len(point_list)):
            node_dict[n]=tuple(point_list[n])
        return node_dict

    def create_weld_dict(self,weld_list):
        #*This is very similar to the 'create_node_dict' method above but it stores the start and end point
        # print('weld list:')
        # print(weld_list)
        weld_dict={}
        weld_index=list(self.node_dict.keys())[-1]+1
        #*This is important becaause it adds NEW weld nodes starting with the next highest index following the tack nodes that were utilized in the tacking problem
        for n in range(len(weld_list)):
            #*Add each of these new nodes to a independent dictionary for line look up
            weld_dict[weld_index+n]=[tuple(weld_list[n][0]),tuple(weld_list[n][1])]
            #*Notice I just nested an array inside of an array
        return weld_dict
    
    def add_weld_nodes(self):
        #*The method to add the weld nodes to the networkx graph object
        point_dic={tuple(self.desired_points[n]):n for n in range(len(self.desired_points))}
        #*This dictonary is very similar to the node dictonary but reversed a given point is the key and returns the corresponding node in the graph
        # print('point_dic')
        # print(point_dic)
        self.point_dic=point_dic
        # self.pointss=[]
        for n in list(self.weld_dict.keys()):
            #*Loop through the given keys in the dictionary containing each set of points for each weld line
            self.Graph.add_node(n)
            #*Add each of these 'new' nodes to the graph
            self.Graph.add_edge(n,point_dic[self.weld_dict[n][0]])
            self.Graph.add_edge(n,point_dic[self.weld_dict[n][1]])
            #*Add limited connections for each of these nodes to force the path of the weld


            #* This next section I am basically generating the paths and inverse kinematics for each of these new edge paths.

            normal_a=self.Graph.nodes[point_dic[self.weld_dict[n][0]]]['Normals']
            normal_b=self.Graph.nodes[point_dic[self.weld_dict[n][1]]]['Normals']
            #*Get the normals that were already calculated in   the tacking solution, remeber these normals are basically calculated with an internal pyvista algorithm that just gives the arrow pointing away from the mesh at any point
            #TODO: This methodoligy mentioned is one of the top things to motify, edit and improve in relation to a better path planning algorithm
            
            #*So since we need to force the solution to travel through the weld nodes we take the weld line and bisect it with a node that the Traveling SalesMan has to go through
            normals=(normal_a+normal_b)/2
            normals=np.array(normals)/np.linalg.norm(normals)
            #*This middle node then has the orientation of the average between these two start and finish normal
            #TODO: Consider chaning this

            point_a=tuple(np.array(self.weld_dict[n][0]))
            point_b=tuple(np.array(self.weld_dict[n][1]))
            #*Refrence the start and end point from the weld line dictionary
            
            point=self.Transform(((np.array(point_a)+np.array(point_b))/2))
            #*Average these points and pass them through the transform.
            self.Graph.nodes[n]['cord']=tuple(point)
            #*Store these cordinates in the graph
            #TODO: A lot of my data pulls, could be improved. For example I store that data but I am unsure if I ever even refrence the stored 'cord' value after storing it or how I refrence it. This should be cleaned up
            normals_passed=deepcopy(normals)
            normals_passed=normals_passed*-1
            normals_passed[-1]=0            
            normals_passed=list(normals_passed)


            seed_state=self.Graph.nodes[point_dic[self.weld_dict[n][0]]]['Point_IK']
            #*Clearly the seed_state should be the solved inverse kinematics for that start

            joint_IK=self.IK_SOLVER.get_IK(self.Graph.nodes[n]['cord'],normals_passed,seed_state=seed_state)
            #*Get the inverse kinematics for the middle point i.e. the weld node
            self.Graph.nodes[n]['points_IK']=joint_IK
            #*Store that in the graph
            
            for k in self.weld_dict[n]:
                #*We use this loop to generate the edge ojbects
                p1=self.Transform(k)
                #*^ Like why didnt I refrence the stored 'cord' here
                p2=self.Graph.nodes[n]['cord']
                #*Get the points for this edge
                normal1=self.Graph.nodes[point_dic[k]]["Normals"]
                normal2=normals

                if not(self.control_distance==0):
                    #*Adjust the points by the tip to workpiece distance
                    p1=tuple(np.array(p1)+(np.array(normal1)/np.linalg.norm(normal1))*self.control_distance)
                    normal2_add=np.array(normal2)/np.linalg.norm(normal2)
                    normal2_add[-1]=normal1[-1]
                    p2=tuple(np.array(p2)+normal2_add*self.control_distance)
                
                #*Generate the edge with control points and start and stop normals
                edge=edge_line(np.array(euclidean_linspace(p1,p2,10)),deepcopy(normal1),deepcopy(normal2))
                edge.type='weld'
                
                edge.make_smooth_line(Smoothness_Coefficient=self.smoothnesscoef,Interpolation_Resolution=self.tour_resolution,Weight_Ratio=None)
                edge.Path_order(point_dic[k],n)

                edge.compute_joint_distances(self.IK_SOLVER,seed_state=joint_IK)
                nx.set_edge_attributes(self.Graph,{(n,point_dic[k]):edge},'edge')  
                nx.set_edge_attributes(self.Graph,{(n,point_dic[k]):edge.Cost_Function},'weight')
                #*Store the weight of these edges
                
    def plot_solution(self):
        #* Yeah this just plots the total solution angles
        t1=np.linspace(0,1,len(self.TSP_angles))
        fig, axs = plt.subplots(6)
        fig.suptitle('Joint Angle Solutions for each point')

        axs[0].scatter(t1,self.TSP_angles[:,0]*180/np.pi)
        axs[0].set_title('Q0_ik')

        axs[1].scatter(t1,self.TSP_angles[:,1]*180/np.pi)
        axs[1].set_title('Q1_ik')

        axs[2].scatter(t1,self.TSP_angles[:,2]*180/np.pi)
        axs[2].set_title('Q2_ik')

        axs[3].scatter(t1,self.TSP_angles[:,3]*180/np.pi)
        axs[3].set_title('Q3_ik')

        axs[4].scatter(t1,self.TSP_angles[:,4]*180/np.pi)
        axs[4].set_title('Q4_ik')

        axs[5].scatter(t1,self.TSP_angles[:,5]*180/np.pi)
        axs[5].set_title('Q5_ikpp')
        
        plt.show()
        # exit()

    def solve_weld_tour(self):
        #*THis is a custom MIP (mixed integer program) to solve the welding order based in python-mip, it is very similar to the way the traditional tranveling salesman problem is programmed:  https://docs.python-mip.com/en/latest/examples.html#the-traveling-salesman-problem
        #* It is easier to understanf this in slides, I will make some
        from itertools import product
        from sys import stdout as out
        from mip import Model, xsum, minimize, BINARY, INTEGER, CONTINUOUS, constants
        
        model= Model()
        V=set(range(len(self.weld_dict.keys())))
        n=len(self.weld_dict.keys())
        nodes=list(self.weld_dict.keys())
        # binary variables indicating if arc (i,j) is used on the route or not
        x = [[[model.add_var(var_type=BINARY) for l in range(4)]for j in V] for i in V]

        # continuous variable to prevent subtours: each city will have a
        # different sequential id in the planned route except the first one
        y = [model.add_var() for i in V]
        # constraint : leave each city only once
        for i in V:
            model += xsum(x[i][j][l]for l in range(4) for j in V - {i}) == 1

        # constraint : enter each city only once
        for i in V:
            model += xsum(x[j][i][l] for l in range(4) for j in V - {i}) == 1

        # subtour elimination

        for (i, j) in product(V - {0}, V - {0}):
            if i != j:
                model += y[i] - (n+1)*(x[i][j][0]+x[i][j][1]+x[i][j][2]+x[i][j][3]) >= y[j]-n

        self.all_shortest_paths=[]
        self.weld_graph=nx.complete_graph(self.weld_dict.keys())

        nx.set_edge_attributes(self.weld_graph,None,"Shortest_Paths")

        nx.set_edge_attributes(self.weld_graph,None,"Shortest_Path_Weights")

        size=len(self.weld_graph.nodes)+1
        weight_Ahsymetric=np.zeros((len(self.weld_graph.nodes),len(self.weld_graph.nodes),4))
        paths_Ahsymetric=np.empty((len(self.weld_graph.nodes),len(self.weld_graph.nodes)),dtype=list)
        default_shortest=np.zeros((len(self.weld_graph.nodes),len(self.weld_graph.nodes)))

        for (u,v) in self.weld_graph.edges:
            #get the connecting points:
            u_connect=self.weld_dict[u]
            v_connect=self.weld_dict[v]
            # g1 ,weight='weight'
            u_toto_shortest=nx.shortest_path(self.Graph,source=self.point_dic[u_connect[0]],target=self.point_dic[v_connect[0]])
            u_tofrom_shortest=nx.shortest_path(self.Graph,source=self.point_dic[u_connect[0]],target=self.point_dic[v_connect[1]])
            u_fromto_shortest=nx.shortest_path(self.Graph,source=self.point_dic[u_connect[1]],target=self.point_dic[v_connect[0]])
            u_fromfrom_shortest=nx.shortest_path(self.Graph,source=self.point_dic[u_connect[1]],target=self.point_dic[v_connect[1]])
            #g2
            v_toto_shortest=nx.shortest_path(self.Graph,source=self.point_dic[v_connect[0]],target=self.point_dic[u_connect[0]])
            v_tofrom_shortest=nx.shortest_path(self.Graph,source=self.point_dic[v_connect[0]],target=self.point_dic[u_connect[1]])
            v_fromto_shortest=nx.shortest_path(self.Graph,source=self.point_dic[v_connect[1]],target=self.point_dic[u_connect[0]])
            v_fromfrom_shortest=nx.shortest_path(self.Graph,source=self.point_dic[v_connect[1]],target=self.point_dic[u_connect[1]])

            Paths=[u_toto_shortest,u_tofrom_shortest,u_fromto_shortest,u_fromfrom_shortest,v_toto_shortest,v_tofrom_shortest,v_fromto_shortest,v_fromfrom_shortest]
            paths_Ahsymetric[u-36][v-36]=deepcopy(Paths[0:4])
            paths_Ahsymetric[v-36][u-36]=deepcopy(Paths[4:])
            # print(Paths)
            for n in range(len(Paths)):
                # print(n)
                # print(Paths[n])
                Path_weight_def=0
                for i in range(len(Paths[n][0:-1])):
                    # print(i)
                    Path_weight_def+=self.Graph.edges[Paths[n][i],Paths[n][i+1]]['weight']
                if n > 3:
                    # print(self.Graph.edges[Paths[n][-1],u])
                    # print(self.Graph.edges[v,Paths[n][0]])
                    Path_weight_def+=self.Graph.edges[Paths[n][-1],u]['weight']+self.Graph.edges[v,Paths[n][0]]['weight']
                    weight_Ahsymetric[v-36][u-36][n-4]=deepcopy(Path_weight_def)
                else:
                    # print(self.Graph.edges[Paths[n][-1],v])
                    # print(self.Graph.edges[u,Paths[n][0]])
                    Path_weight_def+=self.Graph.edges[Paths[n][-1],v]['weight']+self.Graph.edges[u,Paths[n][0]]['weight']
                    weight_Ahsymetric[u-36][v-36][n]=deepcopy(Path_weight_def)
                # input('Continue?:')
            
        default_relations=np.zeros((len(self.weld_graph.nodes),len(self.weld_graph.nodes),len(self.weld_graph.nodes),4))

        for i in V:
            for j in V-{i}:
                for k in V-{i}-{j}:
                    for p in range(4):
                        for q in range(4):
                            if paths_Ahsymetric[i][j][p][-1]==paths_Ahsymetric[j][k][q][0]:
                                    model += x[i][j][p]+x[j][k][q] <= 1

        model.objective=minimize(xsum(weight_Ahsymetric[i][j][l]*((x[i][j][l])) for l in range(4) for i in V for j in V))
        model.optimize(max_seconds=60)
        # checking if a solution was found
        solution=[]
        solution_bare=[]
        if model.num_solutions:
            out.write('route with total distance %g found: %s'
                    % (model.objective_value, nodes[0]))
            solution.append(nodes[0])
            solution_bare.append(nodes[0])
            nc = (0,0)
            while True:
                nc = [(i,j) for i in V for j in range(4) if x[nc[0]][i][j].x >= 0.99][0]
                out.write(' -> %s' % nodes[nc[0]])
                next_nodes=paths_Ahsymetric[solution[-1]-36][nodes[nc[0]]-36][nc[-1]]
                solution+=next_nodes
                solution.append(nodes[nc[0]])
                solution_bare.append(nodes[nc[0]])

                if nc[0] == 0:
                    break
            out.write('\n')

        solution=solution[:-1]
        print('solution')
        print(solution)
        print('solution bare')
        print(solution_bare)
        input('good?')
        print(len(solution))
        self.tour=solution[0:-1]
        np.save('tour_weld_current',self.tour)
            
    def solve_tac_tour(self,Graph,runs,trials):
        #*THis tour is solved simply with a LKH python plugin and a symetric weight matrix representing the cost between any two nodes i,j
        A=nx.adjacency_matrix(Graph, weight='weight')
        A=np.array(A.toarray())
        Sequential=np.matrix.flatten(A)
        Sequential=np.sort(Sequential)
        Sequential=np.block([[Sequential]])
        Sequential=np.reshape(Sequential,(len(Sequential[0]),1))
        Sequential=scipy.spatial.distance.pdist(Sequential)
        minval = np.min(Sequential[np.nonzero(Sequential)])
        A=A/minval
        #*I divided A by the smallest non-zero number to scale the costs to integer values as that is what LKH is able to interpret
        tour=LKH_solver.solve_tour_tsp(A,runs=runs,trials=trials)        
        self.tour=tour
        # np.save('tour_tac_current',tour)
        # print(tour)
    
    def reorder_tour(self):
        #*This just reorders the tour to start with the closest point
        current_nodes=np.array([self.Transform(n) for n in list(self.node_dict.values())])
        current_node_tree=scipy.spatial.KDTree(current_nodes)
        current_position=self.IK_SOLVER.move_group.get_current_pose().pose
        current_position=[current_position.position.x,current_position.position.y,current_position.position.z]
        cpi=current_node_tree.query(current_position)[1]
        # np.save('cpi_weld_current',cpi)
        # if self.tacking==True:
        #     cpi=np.load('cpi_weld_current.npy')
        for n in range(len(self.tour)):
            if self.tour[n]==cpi:
                self.tour=list(self.tour[n:])+list(self.tour[0:n])
                print('Path re order')
                break
        self.tour.append(self.tour[0])
        self.tour_edges=list(zip(self.tour,self.tour[1:]))
        input(self.tour_edges)


    def Transform(self,point,normal=False):
        #*This just applies the linear transform to any point or line/normal
        if normal==False:
            return np.matmul(self.T,np.append(np.array(point),[1]))[0:3]
        else:
            return np.matmul(self.T[0:3,0:3],np.array(point))


    def proccess_tour(self):
        #*This is a large method with a lot of subfunctions that I either thought were useful, or currently are utilized to find a complete, continous tour that does not violate the joint limits of the robotic arm
        def absolute_angle_array(array):
            #*This goes through an array of angles and gets rid of multiples of 2pi between consecutive angles
            for n in range(len(array)-1):
                if abs(array[n+1]-array[n])>0.8*np.pi:
                    array[n+1]+=round(abs(array[n+1]-array[n])/np.pi)*np.pi*-1*np.sign((array[n+1]-array[n]))
            return array
        
        def absolute_angle_edge(joint):
            #*This goes through an each edge's angles for a specified joint and gets rid of multiples of 2pi between consecutive angles
            edge=self.edge_object_list[0]
            edge.angles[:,joint]=absolute_angle_array(edge.angles[:,joint])
            for n in range(1,len(self.edge_object_list)):
                edge=self.edge_object_list[n]
                edge_prev=self.edge_object_list[n-1]
                edge_delta=edge.angles[0,joint]-edge_prev.angles[-1,joint]
                if abs(edge_delta)>0.8*np.pi:
                    edge.angles[:,joint]+=round(abs(edge_delta)/np.pi)*-1*np.pi*np.sign(edge_delta)
                edge.angles[:,joint]=absolute_angle_array(edge.angles[:,joint])

        def redo_edge_angles():
            #*repopulate the edges angles based of the stored TSP angles -may be depreciated
            self.tour_resolution=len(self.edge_object_list[0].angles)
            for n in range(len(self.edge_object_list)):
                edge=self.edge_object_list[n]
                edge.angles=list(self.TSP_angles[(n*(self.tour_resolution)):(((n+1)*(self.tour_resolution)))])
        
        def populate_tsp_angles(joint=None):
            #*Populate each edges angles to the TSP_angle array stored in object for a specified joint
            if joint==None:
                self.TSP_angles=[]
                for x in self.edge_object_list:
                    self.TSP_angles=self.TSP_angles+list(x.angles)
                self.TSP_angles=np.array(self.TSP_angles)
            else:
                new_angles=[]
                for x in self.edge_object_list:
                    new_angles=new_angles+list(x.angles[:,joint])
                self.TSP_angles[:,joint]=new_angles

        def handle_edge_flip(edge):
            #*flips an edges rotation i.e. cw->ccw or ccw->cw
            edge.flip_rotation_ee()
            angles=deepcopy(edge.angles[0])
            edge.compute_joint_distances(self.IK_SOLVER,seed_state=angles)
            populate_tsp_angles()
            self.TSP_angles[:,5]=absolute_angle_array(self.TSP_angles[:,5])
            redo_edge_angles()

        def handle_edge_flip_edge(edge):
            #*same as above but does not repopulate the stored TSP angles- these were origanally used to link the whole solution together and analyize it but since move away from
            edge.flip_rotation_ee()
            angles=deepcopy(edge.angles[0])
            edge.compute_joint_distances(self.IK_SOLVER,seed_state=angles)
            absolute_angle_edge(5)
            
        def flip_previous_edges(edge_index,sign,Flip_none_repeated=False,minimum_index=0):
            #*algorithm that goes to either a minimum or maximum joint value in some edge and flips all the previous edges rotation direction to avoid it happening
            if sign>0:
                #*sign dictates the desired flip direction
                value_check=np.min(self.TSP_angles[:,5])
            else:
                value_check=np.max(self.TSP_angles[:,5])
            
            for n in range(edge_index):
                ei=edge_index-(n+1)
                # print(ei)
                edge=self.edge_object_list[ei]
                if (edge.Rotatable and (Flip_none_repeated or edge.repeated)) and abs(value_check)>=2*np.pi and ei>=minimum_index:
                    ac=edge.total_angle_change()
                    if sign*ac<0:
                        handle_edge_flip_edge(edge)
                        populate_tsp_angles()
                        if sign>0:
                            value_check=np.min(self.TSP_angles[:,5])
                        else:
                            value_check=np.max(self.TSP_angles[:,5])
                             
        def flip_forward_edges_greedy(start_index=0,Flip_none_repeated=True):
            #*start at the first edge looks at each next edge and flips rotation directions to stay in boounds
            skip_n=0
            for n in range(len(self.edge_object_list)-start_index):

                if skip_n==0:      
                    print(n)
                    ei=n-start_index
                    edge=self.edge_object_list[ei]
                    if (edge.Rotatable and (Flip_none_repeated or edge.repeated)):
                        max=np.max(edge.angles[:,5])
                        min=np.min(edge.angles[:,5])
                        ac=edge.total_angle_change()
                        if abs(max)>abs(min) and abs(min-ac)<2*np.pi:
                            if ac>0:
                                handle_edge_flip_edge(edge)
                        elif abs(min)>abs(max):
                            if ac<0 and abs(max-ac)<2*np.pi:
                                handle_edge_flip_edge(edge)
                    elif edge.type=='weld' and n+3<len(self.edge_object_list)-1:
                        if self.edge_object_list[n+1].type=='weld' and\
                        self.edge_object_list[n+2].type=='weld' and\
                        self.edge_object_list[n+3].type=='weld':
                            #get the total rotation
                            total_rotation=edge.total_angle_change()+self.edge_object_list[n+1].total_angle_change()\
                            +self.edge_object_list[n+2].total_angle_change()+self.edge_object_list[n+3].total_angle_change()
                            if abs(edge.angles[0,5]-total_rotation)<abs(edge.angles[0,5]+total_rotation):
                                # print('k loop')
                                for k in range(n,n+4):
                                    # print(k)
                                    self.edge_object_list[k].flip_path()
                                    angles=deepcopy(self.edge_object_list[k].angles[0])
                                    # self.edge_object_list[k].compute_joint_distances(self.IK_SOLVER,seed_state=angles)
                                # print('k loop exit')
                                if n!=0:
                                    self.edge_object_list=self.edge_object_list[0:n]+self.edge_object_list[n+3:n-1:-1]+self.edge_object_list[n+4:]
                                elif n==0:
                                    self.edge_object_list=self.edge_object_list[n+3::-1]+self.edge_object_list[n+4:]
                                
                                skip_n=3
                else:
                    skip_n=skip_n-1
            populate_tsp_angles()
        
        def remove_reset_dwells():
            #*removes stacked reset dwells depreciated
            go_on=True
            count=0
            while go_on and count<10:
                count+=1
                parse_on=True
                for n in range(len(self.edge_object_list)):
                    if parse_on==True:
                        edge=self.edge_object_list[n]
                        if edge.type=='dwell':
                            if n<len(self.edge_object_list)-2:
                                next_edge=edge=self.edge_object_list[n+1]
                                bool= abs(next_edge.angles[0,5]-edge.total_angle_change())<abs(next_edge.angles[0,5])
                                forward_edges=1
                                for k in range(n+1,len(self.edge_object_list)):
                                    nn_edge=self.edge_object_list[k]
                                    if nn_edge.type=='dwell':
                                        forward_edges+=1
                                    else:
                                        break
                                if forward_edges>1:
                                    # print(len(self.edge_object_list))
                                    self.edge_object_list=self.edge_object_list[0:n]+self.edge_object_list[n+forward_edges:]
                                    # print(n)
                                    # print(forward_edges)
                                    # print(len(self.edge_object_list))
                                    # exit()
                                    # absolute_angle_edge(5)
                                    # populate_tsp_angles()
                                    parse_on=False

                                elif bool:
                                    self.edge_object_list=self.edge_object_list[0:n]+self.edge_object_list[n+1:]
                                    absolute_angle_edge(5)
                                    populate_tsp_angles()
                                    parse_on=False

                            elif n==len(self.edge_object_list)-1:
                                absolute_angle_edge(5)
                                populate_tsp_angles()
                                go_on=False

        def add_reset_dwell_edge():
            #*add a reset dwell before the joint violations to try and keep the solution in bounds
            #*There is a complicated operation in here specific to the original point in that if it is returning to a start point after a string of welds going in a circle it can flip this order to minimize angle change
            #add the dwell right before it goes out of bounds......idiot
            start_index=0
            while start_index<len(self.edge_object_list)-2:
                for n in range(start_index,len(self.edge_object_list)-1):
                    # print(n)
                    edge=self.edge_object_list[n+1]
                    edge_prev=self.edge_object_list[n]
                    [max_angle,max_angle_id,min_angle,min_angle_id,edge_index_max,edge_index_min]=get_max_min(edge)
                    if max_angle>2*np.pi and min_angle<-2*np.pi:
                        print('This edge can not currently be solved with a dwell')
                    elif max_angle>2*np.pi or min_angle<-2*np.pi:
                        print('ADDING DWELL')
                        normal_faux=[0,0,1]
                        normal1=deepcopy(edge.normal1)
                        p1=tuple(edge.Path[0])
                        p2=(np.array(p1)+(np.array(normal_faux)/np.linalg.norm(normal_faux))*self.dwell_distance)
                        p2[-1]=self.height_scaling_long
                        p2=tuple(p2)
                        control_points=np.array(euclidean_linspace(p1,p2,5)+euclidean_linspace(p2,p1,5))
                        dwell=dwell_edge(control_points,normal1,normal1)
                        dwell.make_smooth_line(Smoothness_Coefficient=self.smoothnesscoef,Interpolation_Resolution=self.tour_resolution,Weight_Ratio=None)
                        if max_angle>2*np.pi:
                            dwell.compute_joint_distances(self.IK_SOLVER,sign=-1,seed_state=edge_prev.angles[-1,:])
                        elif min_angle<-2*np.pi:
                            dwell.compute_joint_distances(self.IK_SOLVER,sign=1,seed_state=edge_prev.angles[-1,:])
                        self.edge_object_list=self.edge_object_list[0:n+1]+[dwell]+self.edge_object_list[n+1:]
                        absolute_angle_edge(5)
                        populate_tsp_angles()
                        start_index=deepcopy(n)
                        break
                    if n==len(self.edge_object_list)-2:
                        start_index=len(self.edge_object_list)-2
            
        def get_higher_resolution_tour(tour_resolution=None):
            #*manipulates the tours resolution, at first all lines were done at the same resolution and this was pretty neccessary but I found a way to change the resolution of more important movements like the weld lines and keep less important ones like the tack lines lower
            if tour_resolution==None:
                tour_resolution=self.tour_resolution
            else:
                self.tour_resolution=tour_resolution
            for n in range(len(self.edge_object_list)):
                edge=self.edge_object_list[n]
                edge.make_smooth_line(Smoothness_Coefficient=self.smoothnesscoef,Interpolation_Resolution=tour_resolution,Weight_Ratio=None)
                edge.make_orientation_vectors()
                angles=deepcopy(edge.angles[0])
                edge.compute_joint_distances(self.IK_SOLVER,seed_state=angles)
                
        def replace_weld_edges():
            #*Initially each weld line is made up of two edge objects this and combines the weld edges into one edge and makes them the child class for the actual weld proceadure.
            self.replace_weld_edges=True
            new_edge_object_list=[]
            skip_edge=False
            for n in range(len(self.edge_object_list)):
                edge=self.edge_object_list[n]
                if skip_edge==True:
                    skip_edge=False
                else:
                    if edge.type=='weld' and n<len(self.edge_object_list)-1:
                        next_edge=self.edge_object_list[n+1]
                        if next_edge.type=='weld':
                            # avg_point=(np.array(edge.Path[-1])+np.array(next_edge.Path[0]))/2
                            # p1=edge.Path[0]
                            # p2=next_edge.Path[-1]
                            u=edge.PathDirection[0]
                            v=next_edge.PathDirection[1]
                            # p1,p2=self.Transform(self.node_dict[u]),self.Transform(self.node_dict[v])
                            p1,p2=self.node_dict[u],self.node_dict[v]
                            
                            normal1=self.Graph.nodes[u]["Normals"]
                            normal2=self.Graph.nodes[v]["Normals"]
                            # control_points=np.array(euclidean_linspace(p1,avg_point,10)+euclidean_linspace(avg_point,p2,10)[1:])
                            control_points=np.array(euclidean_linspace(p1,p2,20))
                            # control_points=np.array(euclidean_linspace(point_1,point_2,5))
                            # edge=edge_line(control_points,deepcopy(normal_1),deepcopy(normal_2))
                            # normal1=edgnormal1
                            # normal2=next_edge.normal2
                            # new_edge=weld_edge_double_corner_turn(control_points=control_points,normal1=normal1,normal2=normal2,control_distance=self.control_distance)
                            new_edge=weld_edge_double_corner_turn(control_points=control_points,normal1=normal1,normal2=normal2,control_distance=self.control_distance)
                            
                            new_edge.type='weld'
                            new_edge_object_list.append(new_edge)
                            skip_edge=True
                            new_edge.make_smooth_line(Smoothness_Coefficient=self.smoothnesscoef,Interpolation_Resolution=self.tour_resolution_welds,Weight_Ratio=None)
                            new_edge.Path_order(edge.PathDirection[0],next_edge.PathDirection[1])
                            new_edge.make_orientation_vectors()
                            new_edge.compute_joint_distances(IK_object=self.IK_SOLVER,seed_state=edge.angles[0])
                    else:
                        new_edge_object_list.append(edge)
            self.edge_object_list=new_edge_object_list

        def correct_joint_angles_to_current(joint):
            #*Set the joint angle arrays to start at the current joint values
            curr=list(self.IK_SOLVER.move_group.get_current_joint_values())
            if not(math.isclose(curr[joint]*np.pi/180,self.edge_object_list[0].angles[0,joint]*np.pi/180,abs_tol=10**-3)):
                value=round(abs(curr[joint]-self.edge_object_list[0].angles[0,joint])/np.pi)*np.pi*np.sign((curr[0]-self.edge_object_list[0].angles[0,joint]))
                for n in range(len(self.edge_object_list)):
                    edge=self.edge_object_list[n]
                    edge.angles[:,joint]=edge.angles[:,joint]+value

        def adjust_by_pi(integer,joint=None):
            #*add or minus some amount of pi to an edges joint angles
            if joint==None:
                joint=5
            for n in range(len(self.edge_object_list)):
                edge=self.edge_object_list[n]
                edge.angles[:,joint]=edge.angles[:,joint]+np.pi*integer
            populate_tsp_angles(joint)
        
        def create_precise_tour():
            #*add the start point of each edge onto the end point of the last edge
            for n in range(len(self.edge_object_list)-1):
                edge=self.edge_object_list[n]
                next_edge=self.edge_object_list[n+1]
                edge.add_edge_end_point(next_edge)
        
        def get_max_min(edge=None):
            #*get max and min edge angle values that indicate the validity of the solution after absolute valuing it
            if edge==None:
                max_angle=np.max(self.TSP_angles[:,5])
                max_angle_id=np.argmax(self.TSP_angles[:,5])
                min_angle=np.min(self.TSP_angles[:,5])
                min_angle_id=np.argmin(self.TSP_angles[:,5])
                edge_index_max=int((max_angle_id-(max_angle_id%self.tour_resolution))/self.tour_resolution)
                edge_index_min=int((min_angle_id-(min_angle_id%self.tour_resolution))/self.tour_resolution)
                edge_index_max=round((max_angle_id/self.tour_resolution)-0.5)
                edge_index_min=round((min_angle_id/self.tour_resolution)-0.5)
                return [max_angle,max_angle_id,min_angle,min_angle_id,edge_index_max,edge_index_min]
            else:
                max_angle=np.max(edge.angles[:,5])
                max_angle_id=np.argmax(edge.angles[:,5])
                min_angle=np.min(edge.angles[:,5])
                min_angle_id=np.argmin(edge.angles[:,5])
                edge_index_max=None
                edge_index_min=None
                edge_index_max=None
                edge_index_min=None
                return [max_angle,max_angle_id,min_angle,min_angle_id,edge_index_max,edge_index_min]

        
        def run_solver():
            #*a solver pretty much garuentteed to find a continous solution with the greedy algorithm and adding reset dwells
            [max_angle,max_angle_id,min_angle,min_angle_id,edge_index_max,edge_index_min]=get_max_min()
            integer_adjust=np.floor(abs(self.edge_object_list[0].angles[0,5])/(2*np.pi))*-1*np.sign(self.edge_object_list[0].angles[0,5])
            print(integer_adjust)

            if integer_adjust!=0:
                adjust_by_pi(integer_adjust)
            
            if abs(self.edge_object_list[0].angles[0,5]+(2*-1*np.pi*np.sign(self.edge_object_list[0].angles[0,5])))\
                <abs(self.edge_object_list[0].angles[0,5]):
                adjust_by_pi(2*-1*np.sign(self.edge_object_list[0].angles[0,5]))
                
            [max_angle,max_angle_id,min_angle,min_angle_id,edge_index_max,edge_index_min]=get_max_min()
            counter=0

            while (max_angle>np.pi*2 or min_angle<-np.pi*2) and counter<2:
                counter+=1
                flip_forward_edges_greedy()
                [max_angle,max_angle_id,min_angle,min_angle_id,edge_index_max,edge_index_min]=get_max_min()

            
            if max_angle>2*np.pi or min_angle<-2*np.pi:    
                print('reset dwell call')
                add_reset_dwell_edge()

            for n in range(6):
                absolute_angle_edge(n)
            for n in range(5):
                correct_joint_angles_to_current(n)
            

        def get_edge_object_list():
            #*create an edge object list storing all the unique edges- making duplicates for repeated edges
            self.node_dict=self.create_node_dict()
            occurance=[]
            double_edges=[]
            for n in range(len(self.tour_edges)):
                edge=self.tour_edges[n]
                index=[[x,y] for x, y in enumerate(self.tour_edges[:n]) if (y==edge or (y[1],y[0])==edge)]
                if index==[]:
                    occurance.append(0)
                else:
                    stored_edge=self.Graph.edges[self.tour_edges[n]]['edge']
                    stored_edge.repeated=True
                    self.Graph.edges[self.tour_edges[n]]['edge']=[stored_edge,deepcopy(stored_edge)]
                    occurance.append(len(index))
                    double_edges.append(edge)
            
            self.occurance=occurance
            self.double_edges=double_edges
            edge_object_list=[]
            
            for n in range(len(self.tour_edges)):
                edge_tuple=self.tour_edges[n]
                double_bool=edge_tuple in double_edges or (edge_tuple[1],edge_tuple[0]) in double_edges
                if double_bool:
                    edge=self.Graph.edges[self.tour_edges[n]]['edge'][occurance[n]]
                else:
                    print(self.tour_edges[n])
                    print(self.Graph.edges)
                    edge=self.Graph.edges[self.tour_edges[n]]['edge']
                
                if not(edge.PathDirection[0]==self.tour_edges[n][0]):
                        print(edge.PathDirection)
                        print(self.tour_edges[n][0])
                        print('EDGE FLIP')
                        edge.flip_path()
                
                edge_object_list.append(edge)

            self.edge_object_list=edge_object_list

        def special_plot():
            #*plot that distingushes edge parts of the solution
            # t1=np.linspace(0,1,len(self.TSP_angles))
            fig = plt
            colors=['b','r']
            for n in range(len(self.edge_object_list)):
               t=np.linspace(n,n+1,len(self.edge_object_list[n].angles))
               fig.scatter(t,self.edge_object_list[n].angles[:,5]*180/np.pi,color=colors[n%2])
               fig.text(np.average(t),np.average(self.edge_object_list[n].angles[:,5]*180/np.pi-10),self.edge_object_list[n].type)
            plt.show()
        
        if self.edge_object_list==None:
            get_edge_object_list()
        # special_plot()
        
        # self.plot_tour_3D()
        
        if self.weld_tour==True:
            replace_weld_edges()
            # get_higher_resolution_tour(tour_resolution=1557)

            self.plot_tour_3D()
        
        # special_plot()
        
        
        for n in range(6):
            absolute_angle_edge(n)
        for n in range(5):
            correct_joint_angles_to_current(n)

        # special_plot()

        # original_edge_object_list=deepcopy(self.edge_object_list)
        points_reordered=False
        if self.weld_tour==True:
            if not(self.edge_object_list[0].type=='weld'):
                self.edge_object_list=self.edge_object_list[1:]
                points_reordered=True
            if not(self.edge_object_list[-1].type=='weld'):
                self.edge_object_list=self.edge_object_list[:-1]
        populate_tsp_angles()
        [max_angle,max_angle_id,min_angle,min_angle_id,edge_index_max,edge_index_min]=get_max_min()

        if max_angle>2*np.pi or min_angle<-2*np.pi:
            run_solver()

        
        create_precise_tour()
        #TODO:Is this still needed?

        for n in range(6):
            absolute_angle_edge(n)
        for n in range(5):
            correct_joint_angles_to_current(n)
        
        # special_plot()
        self.plot_solution()
        

        if points_reordered==True:
            #*This moves the robot to the first point
            input('Press Enter To Go to First Point:')
            trial_limit=10
            print('Attempting to reach first point in:'+str(trial_limit)+" Trials")
            c=0
            bool=False
            while c<trial_limit and bool==False:
                try:
                    print("Trial Number: "+str(c))
                    c+=1
                    bool=self.correct_first_point()
                except:
                    print('Trial Failed Retrying')
            input('Go on?:')
    
    def special_plot_tour_3D(self):
        pass




    def plot_tour_3D(self):
            #*This generates the 3D solution plot in pyvista over the weld mesh
            blue = np.array([12 / 256, 238 / 256, 246 / 256, 1.0])
            black = np.array([11 / 256, 11 / 256, 11 / 256, 1.0])
            grey = np.array([189 / 256, 189 / 256, 189 / 256, 1.0])
            yellow = np.array([255 / 256, 247 / 256, 0 / 256, 1.0])
            red = np.array([1.0, 0.0, 0.0, 1.0])
            
            Transformed_Mesh=self.Weld_Mesh_Surface.transform(self.T)
            self.plotter.add_mesh(Transformed_Mesh, opacity=0.10,color=True)
            
            
            tour_path=[]
            orientation_vectors=[]
            print(self.tour_edges)
            # angles=[]
            for n in range(len(self.edge_object_list)):
                edge=self.edge_object_list[n]
                orientation_vectors=orientation_vectors+edge.orientation_vectors
                # angles=angles+list(edge.angles)
                lines= pv.lines_from_points(edge.Path)
                self.plotter.add_mesh(lines,line_width=5,color=black)
                
                #*This is code that creates and displays the orientation vectors at each point, depending on when this is called in the overall code it may throw an error
                #TODO:Review this and make sure it is functional
                # for j in range(len(edge.Path)):
                #     orientation_vector=edge.orientation_vectors[j]
                #     vector_position=np.array(edge.Path[j])-np.array(orientation_vector)/100
                #     arrow=pv.Arrow(vector_position,np.array(orientation_vector),scale=np.linalg.norm(orientation_vector)/100,)
                #     self.plotter.add_mesh(arrow,color=blue)
            
            all_points=np.array(list(self.node_dict.values())) 
            poly = pv.PolyData(np.array(all_points))
            poly["My Labels"] = [f"{i}" for i in range(poly.n_points)]
            self.plotter.add_point_labels(poly, "My Labels", point_size=30, font_size=40)
            self.plotter.show(auto_close=True)
    
    def correct_first_joint_angle(self,joint=None):
        #*Earlier I mentioned some of the lower level control was present in the TSP solution, but rather this is done by refrencing the IK Solver object witg the move it functionality
        #*This method basically gets the robot into the starting position of the first weld line
        if joint==None:
            #*The joint to match by default is the last joint, as this likely needs rotated by +/- 2pi
            joint=5
        
        current_position=self.IK_SOLVER.move_group.get_current_pose().pose
        current_position=[current_position.position.x,current_position.position.y,current_position.position.z]
        #*Get current position from the robot
        
        waypoints = []
        curr=list(self.IK_SOLVER.move_group.get_current_joint_values())
        #*Get the current joint angles 

        if not(math.isclose(curr[joint]*np.pi/180,self.edge_object_list[0].angles[0,joint]*np.pi/180,abs_tol=10**-3)):
            #*Okay here we check if the joints angle matches the starting angle and if not we get the current position 
            current_position=self.IK_SOLVER.move_group.get_current_pose().pose
            current_position=[current_position.position.x,current_position.position.y,current_position.position.z]

            if current_position[-1]<self.closest_point[-1]+self.normals_closest[-1]*self.control_distance+self.special_lengths[-1]+self.special_lengths[-2]:
                #*We check if the current position clears the vertical limits of the part, i.e. is the TCP above the part by some safety value
                waypoints = []
                wpose = self.IK_SOLVER.move_group.get_current_pose().pose
                waypoints.append(deepcopy(wpose))
                pose_goal=wpose
                pose_goal.position.z = self.closest_point[-1]+self.normals_closest[-1]*self.control_distance+self.special_lengths[-1]+self.special_lengths[-2]
                waypoints.append(deepcopy(pose_goal))
                self.IK_SOLVER.move_group.limit_max_cartesian_link_speed(0.1,link_name='wrist_3_link')
                (plan, fraction) = self.IK_SOLVER.move_group.compute_cartesian_path(
                waypoints, 0.005, 0.0  # waypoints to follow  # eef_step
                )  # jump_threshold
                self.check_plan(plan)
                #*Check to make sure the plan does anything crazy, and kill the program if it does^
                self.IK_SOLVER.move_group.execute(plan,wait=True)
                #*Execute the plan if it passed the check
                # rospy.sleep(3)
                #*You will see a lot of sleeps and inputs commented out, because they are useful for debugging the live program with the robot sometimes
            # input('Continue?')


            #*Since we are now above the part we correct that joint and tell the robot to move to it
            self.IK_SOLVER.move_group.clear_pose_targets()
            curr=list(self.IK_SOLVER.move_group.get_current_joint_values())
            joint_goal = deepcopy(curr)
            joint_goal[5] = self.edge_object_list[0].angles[0,5]
            self.IK_SOLVER.move_group.go(joint_goal, wait=True)
            # self.IK_SOLVER.move_group.clear_pose_targets()
            # rospy.sleep(3)
            self.IK_SOLVER.move_group.clear_pose_targets()

            # rospy.sleep(3)
        #*We then go above the first refrenced point in the edge object list that containts each edge in the final solution
        waypoints = []
        wpose = self.IK_SOLVER.move_group.get_current_pose().pose
        waypoints.append(copy.deepcopy(wpose))
        pose_goal=wpose
        # print(self.edge_object_list[0].Path[0][1])
        pose_goal.position.x = self.edge_object_list[0].Path[0][0]
        pose_goal.position.y = self.edge_object_list[0].Path[0][1]
        #*notice how we ony refrence the x and y position
        # pose_goal.position.z = self.edge_object_list[0].Path[0][2]
        self.IK_SOLVER.move_group.limit_max_cartesian_link_speed(0.1,link_name='wrist_3_link')
        waypoints.append(deepcopy(pose_goal))
        
        (plan, fraction) = self.IK_SOLVER.move_group.compute_cartesian_path(
            waypoints, 0.005, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold
        
        self.check_plan(plan)
        #*Check the plan generated from move it

        input('Press Enter to Move above first point:')
        self.IK_SOLVER.move_group.execute(plan,wait=True)
        rospy.sleep(3)

        self.IK_SOLVER.move_group.clear_pose_targets()
        curr=list(self.IK_SOLVER.move_group.get_current_joint_values())
        joint_goal = deepcopy(curr)
        joint_goal[5] = self.edge_object_list[0].angles[0,5]
        self.IK_SOLVER.move_group.go(joint_goal, wait=True)
        # self.IK_SOLVER.move_group.clear_pose_targets()
        # rospy.sleep(3)
        self.IK_SOLVER.move_group.clear_pose_targets()



        waypoints = []
        wpose = self.IK_SOLVER.move_group.get_current_pose().pose
        waypoints.append(copy.deepcopy(wpose))
        pose_goal=wpose
        # print(self.edge_object_list[0].Path[0][1])
        pose_goal.position.x = self.edge_object_list[0].Path[0][0]
        pose_goal.position.y = self.edge_object_list[0].Path[0][1]
        pose_goal.position.z = self.edge_object_list[0].Path[0][2]
        #*See how we refrence all three cords here x,y,z
        self.IK_SOLVER.move_group.limit_max_cartesian_link_speed(0.1,link_name='wrist_3_link')
        waypoints.append(deepcopy(pose_goal))
        
        (plan, fraction) = self.IK_SOLVER.move_group.compute_cartesian_path(
            waypoints, 0.005, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold
        
        self.check_plan(plan)
        #*After going above the first point generate the plan to drop down from above it

        input('Press Enter to drop down to first point:')
        self.IK_SOLVER.move_group.execute(plan,wait=True)
        rospy.sleep(3)
        self.IK_SOLVER.move_group.clear_pose_targets()


    
    def check_plan(self,plan):
        #*This checks a plan from move it and checks wether it will violate the joint constraints
        curr=list(self.IK_SOLVER.move_group.get_current_joint_values())
        #*Take the current joint values
        # for n in range(0,5):
            #*This range can be altered but it basically throws away a trajectory if there is a joint angle change greater than 90 degrees
            # if abs(plan.joint_trajectory.points[-1].positions[n]-curr[n])>1*np.pi:
            #     self.IK_SOLVER.move_group.stop()
            #     raise Exception('WARNING BAD ALIGNMENT TRAJECTORY EXITING')
        
        
            # print(n)
        if not(math.isclose(plan.joint_trajectory.points[-1].positions[4],curr[4],abs_tol=10**-1)):
            #*Before moving into the first point the last joints value should remain constant
            self.IK_SOLVER.move_group.stop()
            raise Exception('WARNING BAD ALIGNMENT TRAJECTORY EXITING')


    def Send_to_robot(self):
        #*This sends each edge in the edge object list to the robot
        self.correct_first_joint_angle()
        #*First we have to get the robot into the first position^
        curr=list(self.IK_SOLVER.move_group.get_current_joint_values())
        for n in range(len(curr)):
            if not(math.isclose(curr[n]*np.pi/180,self.edge_object_list[0].angles[0][n]*np.pi/180,abs_tol=10**-2)):
                print('WARNING STARTING ANGLES DO NOT MATCH CURRENT ANGLES')
                print(self.edge_object_list[0].angles[0][n])
                print(self.IK_SOLVER.move_group.get_current_joint_values())
                self.IK_SOLVER.move_group.stop()
                #*If the first point of the edge does not match the current point kill the program
                exit()
        # input('Press Enter to Send Trajectory:')
        for n in range(len(self.edge_object_list)):
            # weld_it=input('Press y to weld Trajector,Press n to just run Trajectory to Send Trajectory:')
            weld_it=input('Press Enter to Send Trajectory:')
            # weld_it=True
            #*weld it can be used as an additional query to turn off and on the welding at each weld line
            # if weld_it=='y':
            #     weld_it=False
            # else:
            #     weld_it=False
            edge=self.edge_object_list[n]

            self.IK_SOLVER.move_group.go(edge.angles[0],wait=True)
            #*^Get the edge and send it \|/ to the robot using the sigmoind function for acceleration control
            self.IK_SOLVER.send_edge_to_moveit_scurve(edge,tacking=self.tacking,weld_it=True)
            # rospy.sleep(0.2)

    def Send_to_robot_measured(self):
        #*This is something I used to collect a bunch of joint angle data
        self.correct_first_joint_angle()
        curr=list(self.IK_SOLVER.move_group.get_current_joint_values())
        for n in range(len(curr)):
            if not(math.isclose(curr[n]*np.pi/180,self.edge_object_list[0].angles[0][n]*np.pi/180,abs_tol=10**-2)):
                print('WARNING STARTING ANGLES DO NOT MATCH CURRENT ANGLES')
                print(self.edge_object_list[0].angles[0][n])
                print(self.IK_SOLVER.move_group.get_current_joint_values())
                self.IK_SOLVER.move_group.stop()
                exit()

        input('Press enter to continue:')
        self.cart_points_true.append(self.edge_object_list[0].Path[0])
        pose=self.IK_SOLVER.move_group.get_current_pose().pose.position
        self.cart_points_ex.append([pose.x,pose.y,pose.z])
        self.JA_calc.append(self.edge_object_list[0].angles[0])
        self.JA_calc_to_cart.append(self.IK_SOLVER.get_fk(self.edge_object_list[0].angles[0]))
        self.JA_ex.append(list(self.IK_SOLVER.move_group.get_current_joint_values()))
        self.JA_ex_to_cart.append(self.IK_SOLVER.get_fk(list(self.IK_SOLVER.move_group.get_current_joint_values())))
        self.total_edge_object_list.append(self.edge_object_list)
        for n in range(len(self.edge_object_list)):
            edge=self.edge_object_list[n]
            # input('Press enter to continue:')
            self.IK_SOLVER.send_edge_to_moveit_scurve(edge,tacking=self.tacking,weld_it=False)
            rospy.sleep(0.2)
            
            self.cart_points_true.append(edge.Path[-1])
            pose=self.IK_SOLVER.move_group.get_current_pose().pose.position
            self.cart_points_ex.append([pose.x,pose.y,pose.z])
            self.JA_calc.append(edge.angles[-1])
            self.JA_calc_to_cart.append(self.IK_SOLVER.get_fk(edge.angles[-1]))
            self.JA_ex.append(list(self.IK_SOLVER.move_group.get_current_joint_values()))
            self.JA_ex_to_cart.append(self.IK_SOLVER.get_fk(list(self.IK_SOLVER.move_group.get_current_joint_values())))            

    def save_trajectory_data(self):
        data_dict={
                'cart_points_true':self.cart_points_true,
                'cart_points_ex':self.cart_points_ex,
                'JA_calc':self.JA_calc,
                'JA_calc_to_cart':self.JA_calc_to_cart,
                'JA_ex':self.JA_ex,
                'JA_ex_to_cart':self.JA_ex_to_cart,
                'total_edge_object_list':self.total_edge_object_list
        }
        np.save('measured_data_3_with_pause_nonzeroed',data_dict)

    
    def get_geometric_type(self):
        #*This finds the geometric type, short or long that can get to every point on the weld mesh, if it can not it will kill the program
        self.IK_SOLVER.IK_SOLVER._solve_type= "Distance"
        #*Prioritize distance over speed for the solution
        
        origin=np.array([[0,0,0]])
        self.adjusted_desired_points=np.array([self.Transform(n) for n in self.desired_points])
        #*Pass all the desired points through the transform to the weld mesh
        #TODO: Can I just refrence 'cord' here
        print(self.adjusted_desired_points)
        
        x=scipy.spatial.distance.cdist(origin,self.adjusted_desired_points,'euclidean')
        #*^Calculate the distances to each node from the current point
        #why this seed state?
        seed_state=list(self.IK_SOLVER.move_group.get_current_joint_values())
        
        farthest_point=self.desired_points[np.argmax(x)]
        closest_point=self.desired_points[np.argmin(x)]
        #*Find the closest and furthest point
        
        normals_closest=self.compute_normal(closest_point)
        normals_farthest=self.compute_normal(farthest_point)
        #*Get the normals of these points
        closest_points_angles=self.IK_SOLVER.get_IK(self.Transform(closest_point),list([-normals_closest[0],-normals_closest[1],0]),seed_state=seed_state)
        farthest_points_angles=self.IK_SOLVER.get_IK(self.Transform(farthest_point),list([-normals_farthest[0],-normals_farthest[1],0]),seed_state=seed_state)
        #*Get the IK angles for them
        close_config=self.configuration_type(closest_points_angles)
        long_config=self.configuration_type(farthest_points_angles)
        #*Find their configuration types
        if long_config==close_config:
            #*IF they match lock into that configuration
            #hopefully they match
            print('Configuration Types Match')
            self.configuration=close_config
            print('Config')
            print(close_config)
            

        else:
            #*If not try to find a combination that does
            print('Configuration Types DO Not Match Revalutating')
            closest_points_angles=self.IK_SOLVER.get_IK(self.Transform(closest_point),list([-normals_closest[0],-normals_closest[1],0]),seed_state=farthest_points_angles)
            close_config=self.configuration_type(closest_points_angles)
            if long_config==close_config:
                #try to get long config
                print('Configuration Types Match')
                self.configuration=close_config
                print('LONG')
                
            else:
                print('Configuration Types DO Not Match Revalutating')
                closest_points_angles=self.IK_SOLVER.get_IK(self.Transform(closest_point),list([-normals_closest[0],-normals_closest[1],0]),seed_state=closest_points_angles)
                farthest_points_angles=self.IK_SOLVER.get_IK(self.Transform(farthest_point),list([-normals_farthest[0],-normals_farthest[1],0]),seed_state=closest_points_angles)
                close_config=self.configuration_type(closest_points_angles)
                long_config=self.configuration_type(farthest_points_angles)
                if long_config==close_config:
                    #this is short config
                    print('Configuration Types Match')
                    self.configuration=close_config
                    print('SHORT')
                    
                else:
                    print('Can not match configuration types')
                    exit()

    def configuration_type(self,angles):
        #*Take the angles and return what configuration type it is
        if type(angles)==type(None):
            print(angles)
            return 3
        else:
            if 0<angles[3]<np.pi or -np.pi>angles[3]>-2*np.pi:
                #configuration short
                return 0
            
            elif np.pi<angles[3]<2*np.pi or 0>angles[3]>-np.pi:
                #configuration long
                return 1
            
            else:
                #configuration indeterminante
                return 2
        
    def lock_geometric_type(self):
        #*This code locks the robot into the determined geometric configuration stored
        def rotational_bounds(angle):
            #*Generates +/- rotational bounds for a angle
            quad=angle/np.pi
            b1=np.ceil(quad)*np.pi
            b2=np.floor(quad)*np.pi
            #*Rounds the value up and down by pi
            bounds=np.array([b1,b2])
            bounds=np.sort(bounds)
            return bounds
        def lock_wrist_one(joints):
            #*Locks the first wrist in a range
            bounds=rotational_bounds(joints[3])
            if self.configuration_type(joints)==self.configuration:
                #Lock this current configuration
                upper_bounds[3]=bounds[1]
                lower_bounds[3]=bounds[0]
            else:
                #Lock 180 from the current configuration
                if bounds[0]>=0:
                    bounds[0]=bounds[0]-np.pi
                    bounds[1]=bounds[1]-np.pi
                else:
                    bounds[0]=bounds[0]+np.pi
                    bounds[1]=bounds[1]+np.pi
                upper_bounds[3]=bounds[1]
                lower_bounds[3]=bounds[0]
        
        joints=self.IK_SOLVER.move_group.get_current_joint_values()
        lower_bounds=[-2*np.pi]*6
        upper_bounds=[2*np.pi]*6
        lower_bounds[1]=-np.pi
        upper_bounds[1]=0

        shoulder_lift_joint_constraint=moveit_msgs.msg.JointConstraint()
        shoulder_lift_joint_constraint.joint_name=JOINT_NAMES[1]
        shoulder_lift_joint_constraint.position=-np.pi/2
        shoulder_lift_joint_constraint.tolerance_above=np.pi/2
        shoulder_lift_joint_constraint.tolerance_below=np.pi/2
        shoulder_lift_joint_constraint.weight=1
        
        elbow_joint_constraint=moveit_msgs.msg.JointConstraint()
        elbow_joint_constraint.joint_name=JOINT_NAMES[2]


        if joints[2]>0:
            print('positive elbow joint')

            elbow_joint_constraint.position=np.pi/2
            elbow_joint_constraint.tolerance_above=np.pi/2
            elbow_joint_constraint.tolerance_below=np.pi/2
            lower_bounds[2]=elbow_joint_constraint.position-elbow_joint_constraint.tolerance_below
            upper_bounds[2]=elbow_joint_constraint.position+elbow_joint_constraint.tolerance_above
            
            lock_wrist_one(joints)

        elif joints[2]<0:
            print('negative ebow joint')

            elbow_joint_constraint.position=-np.pi/2
            elbow_joint_constraint.tolerance_above=np.pi/2
            elbow_joint_constraint.tolerance_below=np.pi/2
            lower_bounds[2]=elbow_joint_constraint.position-elbow_joint_constraint.tolerance_below
            upper_bounds[2]=elbow_joint_constraint.position+elbow_joint_constraint.tolerance_above
            lock_wrist_one(joints)

        else:
            print('indeterminante elbow joint Roman has not programmed this')
        elbow_joint_constraint.weight=1

        wrist_1_constraint=moveit_msgs.msg.JointConstraint()
        wrist_1_constraint.joint_name=JOINT_NAMES[3]
        self.wrist_1_average=np.average([lower_bounds[3],upper_bounds[3]])
        wrist_1_constraint.position=np.average([lower_bounds[3],upper_bounds[3]])
        wrist_1_constraint.tolerance_above=np.pi/2
        wrist_1_constraint.tolerance_below=np.pi/2
        wrist_1_constraint.weight=1

        constraints=self.IK_SOLVER.move_group.get_path_constraints()
        constraints.joint_constraints=[shoulder_lift_joint_constraint,elbow_joint_constraint,wrist_1_constraint]
        self.IK_SOLVER.move_group.set_path_constraints(constraints)
        self.IK_SOLVER.IK_SOLVER.set_joint_limits(lower_bounds, upper_bounds)
        self.upper_bounds=upper_bounds
        self.lower_bounds=lower_bounds

        # self.IK_SOLVER.add_weld_mesh()

    def correct_first_point(self):
        #*Very similar to correct first angle
        #TODO:See if this function is really neccessary and eliminate if not
        self.IK_SOLVER.move_group.clear_pose_targets()
        # self.closest_point=self.edge_object_list[0,:]
        
        current_position=self.IK_SOLVER.move_group.get_current_pose().pose
        current_position=[current_position.position.x,current_position.position.y,current_position.position.z]

        if current_position[-1]<self.closest_point[-1]+self.normals_closest[-1]*self.control_distance+self.special_lengths[-1]+self.special_lengths[-2]:
            waypoints = []
            wpose = self.IK_SOLVER.move_group.get_current_pose().pose
            waypoints.append(deepcopy(wpose))
            pose_goal=wpose
            pose_goal.position.z = self.closest_point[-1]+self.normals_closest[-1]*self.control_distance+self.special_lengths[-1]+self.special_lengths[-2]
            waypoints.append(deepcopy(pose_goal))
            self.IK_SOLVER.move_group.limit_max_cartesian_link_speed(0.1,link_name='wrist_3_link')
            (plan, fraction) = self.IK_SOLVER.move_group.compute_cartesian_path(
            waypoints, 0.005, 0.0  # waypoints to follow  # eef_step
            )  # jump_threshold
            self.check_plan(plan)
            self.IK_SOLVER.move_group.execute(plan,wait=True)
            # rospy.sleep(3)
            self.IK_SOLVER.move_group.clear_pose_targets()
        
        waypoints = []
        wpose = self.IK_SOLVER.move_group.get_current_pose().pose
        waypoints.append(copy.deepcopy(wpose))
        pose_goal=wpose
        pose_goal.position.x = self.edge_object_list[0].Path[0][0]
        pose_goal.position.y = self.edge_object_list[0].Path[0][1]
        pose_goal.position.z = self.edge_object_list[0].Path[0][2]+self.special_lengths[-1]+self.special_lengths[-2]
        waypoints.append(deepcopy(pose_goal))
        self.IK_SOLVER.move_group.limit_max_cartesian_link_speed(0.1,link_name='wrist_3_link')
        (plan, fraction) = self.IK_SOLVER.move_group.compute_cartesian_path(
        waypoints, 0.005, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold
        self.check_plan(plan)
        bool=self.IK_SOLVER.move_group.execute(plan,wait=True)
        # rospy.sleep(3)
        self.IK_SOLVER.move_group.clear_pose_targets()
        return bool


    def first_point(self):
        #*When the program starts this moves to what should be but may not remain, the first point which is also the closest point
        nx.set_node_attributes(self.Graph,None,"Normals")
        nx.set_node_attributes(self.Graph,None,"Control_Points")
        nx.set_node_attributes(self.Graph,None,"Point_IK")
        #*This sets up the attributes to be stored in the graph object from networkx

        desired_point_tree=scipy.spatial.KDTree(self.adjusted_desired_points)
        self.IK_SOLVER.move_group.set_planning_time(30)
        self.IK_SOLVER.move_group.allow_looking(True)
        self.IK_SOLVER.move_group.allow_replanning(True)
        current_position=self.IK_SOLVER.move_group.get_current_pose().pose
        current_position=[current_position.position.x,current_position.position.y,current_position.position.z]
   
            

        # print(self.desired_points)
        # print(desired_point_tree)
        cpi=desired_point_tree.query(current_position)[1]
        closest_point=self.desired_points[cpi]
        normals_closest=self.compute_normal(closest_point)
        #*This finds the closest point from the scipy spatial KD tree to the current position
        # print('Normals closest')
        # print(normals_closest)
        # input('Continue?')
        closest_point=self.Transform(closest_point)
        self.closest_point=closest_point
        self.normals_closest=normals_closest

        if current_position[-1]<closest_point[-1]+normals_closest[2]*self.control_distance+self.special_lengths[-1]+self.special_lengths[-2]:
            waypoints = []
            wpose = self.IK_SOLVER.move_group.get_current_pose().pose
            waypoints.append(copy.deepcopy(wpose))
            pose_goal=wpose
            pose_goal.position.z = closest_point[2]+normals_closest[2]*self.control_distance+self.special_lengths[-1]+self.special_lengths[-2]
            waypoints.append(deepcopy(pose_goal))
            self.IK_SOLVER.move_group.limit_max_cartesian_link_speed(0.1,link_name='wrist_3_link')
            (plan, fraction) = self.IK_SOLVER.move_group.compute_cartesian_path(
            waypoints, 0.005, 0.0  # waypoints to follow  # eef_step
            )  # jump_threshold
            self.check_plan(plan)
            self.IK_SOLVER.move_group.execute(plan,wait=True)
            #*Go up if not above the part

        waypoints = []
        wpose = self.IK_SOLVER.move_group.get_current_pose().pose
        waypoints.append(copy.deepcopy(wpose))
        pose_goal=wpose
        
        r=self.IK_SOLVER.get_rotation_from_vectors(self.IK_SOLVER.aligned_tool0_vector,list([-normals_closest[0],-normals_closest[1],0]))
        quat=r.as_quat()
        pose_goal.orientation.x = quat[0]
        pose_goal.orientation.y = quat[1]
        pose_goal.orientation.z = quat[2]
        pose_goal.orientation.w = quat[3]
        
        pose_goal.position.x = closest_point[0]+normals_closest[0]*self.control_distance
        pose_goal.position.y = closest_point[1]+normals_closest[1]*self.control_distance
        pose_goal.position.z = closest_point[2]+normals_closest[2]*self.control_distance+self.special_lengths[-1]+self.special_lengths[-2]
        # pose_goal.position.z = closest_point[2]+normals_closest[2]*self.control_distance+self.special_lengths[-1]+self.special_lengths[-2]
        self.IK_SOLVER.move_group.limit_max_cartesian_link_speed(0.1,link_name='wrist_3_link')
        waypoints.append(deepcopy(pose_goal))
        # pose_goal.position.z = closest_point[2]+normals_closest[2]*self.control_distance+self.special_lengths[-1]+self.special_lengths[-2]
        # waypoints.append(deepcopy(pose_goal))
        (plan, fraction) = self.IK_SOLVER.move_group.compute_cartesian_path(
            waypoints, 0.005, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold
        self.check_plan(plan)
        bool=self.IK_SOLVER.move_group.execute(plan,wait=True)
        rospy.sleep(3)

        self.Graph.nodes[cpi]["Normals"]=normals_closest
        self.Graph.nodes[cpi]["Point_IK"]=list(self.IK_SOLVER.move_group.get_current_joint_values())
        #*Store the useful computed data in the graph

    def compute_points_IK(self):
        #*This just computes and stores the IK of each node
        self.IK_SOLVER.IK_SOLVER._solve_type= "Distance"
        self.IK_SOLVER.IK_SOLVER._timeout=0.005*5
        seed_state=list(self.IK_SOLVER.move_group.get_current_joint_values())
        seed_state[3]=self.wrist_1_average
        for u in self.Graph.nodes:
            # print(self.Graph.nodes[u]["Normals"])
            if type(self.Graph.nodes[u]["Normals"])==type(None):
                self.Graph.nodes[u]["Normals"]=self.compute_normal(self.node_dict[u])
            self.Graph.nodes[u]["Control_Points"]=self.compute_control_points(self.Graph.nodes[u]["Normals"],self.Transform(self.node_dict[u])+self.control_distance*self.Graph.nodes[u]["Normals"])
            if self.Graph.nodes[u]["Point_IK"]==type(None):
                joint_IK=self.IK_SOLVER.get_IK(self.Transform(self.node_dict[u])+self.control_distance*self.Graph.nodes[u]["Normals"],list([-self.Graph.nodes[u]["Normals"][0],-self.Graph.nodes[u]["Normals"][1],0]),seed_state=seed_state)
                    
                if type(joint_IK)==type(None):
                    print('Warning no joint angles for a point')
                self.Graph.nodes[u]["Point_IK"]=joint_IK
        # self.IK_SOLVER.IK_SOLVER._timeout=0.005
      
    def compute_edges(self):
        #*This computes the edges
        self.IK_SOLVER.IK_SOLVER._solve_type= "Distance"

        for (u,v) in self.Graph.edges:
            #*loop through the set of all edges in the graph
            edge=self.compute_edge_line(u,v)
            #*generate the edge^
            nx.set_edge_attributes(self.Graph,{(u,v):edge},'edge')            
            nx.set_edge_attributes(self.Graph,{(u,v):edge.Cost_Function},'weight')
            
            #!This is depreciated from when I first started to include joint angles in the cost\|/
            # if self.Include_Joint_Metrics==True:
            #     nx.set_edge_attributes(self.Graph,{(u,v):edge.Cost_Function},'weight')
            # else:
            #     nx.set_edge_attributes(self.Graph,{(u,v):edge.Path_Distance},'weight')

        #*This is something I did to compute power point images
        # # for u in self.Graph.nodes:
        #     # if u !=5:
        # edge=self.Graph.edges[[5,0]]['edge']
        # lines=pv.lines_from_points(edge.Path)
        # # lines=pv.lines_from_points(edge.controlpoints)
        # self.plotter.add_mesh(lines,line_width=5,color='black')
        # points=np.array(list(edge.controlpoints))
        # poly = pv.PolyData(np.array(points))
        # self.plotter.add_mesh(poly,color='blue',point_size=10)
        # Transformed_Mesh=self.Weld_Mesh_Surface.transform(self.T)
        # self.plotter.add_mesh(Transformed_Mesh, opacity=0.9,color=True)
        # self.plotter.show()
            
            





    def compute_edge_line(self,u,v):
        #*Method to call the edge class and pass the appropriate data to generate the object


        #*Normals pulled from each point
        normal1=self.Graph.nodes[u]["Normals"]
        normal2=self.Graph.nodes[v]["Normals"]
        controlpoints1=self.Graph.nodes[u]["Control_Points"]
        controlpoints2=self.Graph.nodes[v]["Control_Points"]
        
        p1,p2=self.Transform(self.node_dict[u]),self.Transform(self.node_dict[v])
        #*pulling each point from the dictionary and passing them through the transform
        #TODO: you could just pull them from the 'cord' in the graph if it asigned here.
        pdist=np.linalg.norm(np.array(p2)-np.array(p1))
        #*This pdist was originally asigned in order to make use of known geometric properties of the first part in implimentation
        #TODO: Write code that no longer relies on this
            
        if not(self.control_distance==0):
            p1=tuple(np.array(p1)+(np.array(normal1)/np.linalg.norm(normal1))*self.control_distance)
            p2=tuple(np.array(p2)+(np.array(normal2)/np.linalg.norm(normal2))*self.control_distance)
        
        
        isdiagonal=math.isclose(pdist,np.sqrt(self.special_lengths[1]**2+self.special_lengths[1]**2),abs_tol=10**-5)
        #*See here I am checking if the current distance between the two nodes in the edge match any of the special distances for the original part
        if (math.isclose(pdist,self.special_lengths[0],abs_tol=10**-5) or 
            math.isclose(pdist,self.special_lengths[1],abs_tol=10**-5) or
            isdiagonal):
            #SHORT
            #*^ Short implies that you can use a lower path height as it does not need to clear some 'ribs' of the part
            edge=edge_line(np.array(euclidean_linspace(p1,controlpoints1[0],5)+euclidean_linspace(controlpoints1[0],controlpoints2[0],10)[1:-1]+euclidean_linspace(controlpoints2[0],p2,5)),deepcopy(normal1),deepcopy(normal2))
            edge.make_smooth_line(Smoothness_Coefficient=self.smoothnesscoef,Interpolation_Resolution=self.tour_resolution,Weight_Ratio=None)
            edge.Path_order(u,v)
            if isdiagonal:
                #*Diagonal originally implied that it was from interior grid corner to another interior grid angle at the diagonal
                edge.Rotatable=True
                #*This means that the edge can be flipped in how it turns from the initial normal to the final if it results in not breaking the joint limits of the last joint
            edge.interior=True

        else:
            #LONG
            #*The long edge type implies to use the taller height scalling
            edge=edge_line(np.array(euclidean_linspace(p1,controlpoints1[1],5)+euclidean_linspace(controlpoints1[1],controlpoints2[1],10)[1:-1]+euclidean_linspace(controlpoints2[1],p2,5)),deepcopy(normal1),deepcopy(normal2))
            edge.make_smooth_line(Smoothness_Coefficient=self.smoothnesscoef,Interpolation_Resolution=self.tour_resolution,Weight_Ratio=None)
            edge.Path_order(u,v)
            edge.Rotatable=True
        
        
        edge.compute_joint_distances(self.IK_SOLVER,seed_state=self.Graph.nodes[u]["Point_IK"])
        #*Compute the IK along that edge line

        return edge
    
    def compute_normal(self,P1):
        #*This is almost the center principle of how the code currently works it glitches out pyvista by asking the cells along a line from the point to the point then sums the normals at this point for all surface cells and takes the average to find the normal out of each corner
        #TODO: Replace this methodology with something superior
        P1=self.Weld_Mesh_Surface.find_closest_point(P1)
        P1=self.Weld_Mesh_Surface.points[P1]
        cells=self.Weld_Mesh_Surface.find_cells_along_line(P1,P1)
        tac_normals=np.sum(self.Weld_Mesh_Surface.cell_normals[cells],0)

        #*This garuantees that the 'normal' is 45 degrees vertical from the horizontal x,y plane. because the magnitude of the vector in the xy plane and in the z plane are both 1 before being divided by the final total norm
        tac_normals[0:2]=(tac_normals[0:2]/np.linalg.norm(tac_normals[0:2]))
        tac_normals[2]=1
        
        tac_normals=tac_normals/np.linalg.norm(tac_normals)
        # print(tac_normals)
        tac_normals=self.Transform(tac_normals,normal=True)
        # print(self.T)
        # print(self.T[0:3,0:3])
        # print(tac_normals)

        #*Some shit I was doing to generate images for power point \|/
        # Transformed_Mesh=self.Weld_Mesh_Surface.transform(self.T)
        # self.plotter.add_mesh(self.Weld_Mesh_Surface, opacity=1,color=True)
        # all_points=np.array(list(P1)) 
        # poly = pv.PolyData(np.array(all_points))
        # poly["My Labels"] = [f"{i}" for i in range(poly.n_points)]
        # self.plotter.add_point_labels(poly, "My Labels", point_size=30, font_size=40)
        # # orientation_vector=edge.orientation_vectors[j]
        # # vector_position=np.array(edge.Path[j])-np.array(orientation_vector)/100
        # arrow=pv.Arrow(P1,np.array(tac_normals),scale=1/25)
        # self.plotter.add_mesh(arrow,color='blue')
        # self.plotter.show()
        # # self.plotter.add_point(P1)
        # exit()
        
        return (tac_normals)

    
    def compute_control_points(self,normal,P1):
        #*This accepts the normal and the point and then returns two sets of control points one for tall scalling and one for short scaling
        controlpoint_long=np.array([normal[0]*self.length_scaling,normal[1]*self.length_scaling,normal[2]*self.height_scaling_long])+np.array(P1)
        controlpoint_short=np.array([normal[0]*self.length_scaling,normal[1]*self.length_scaling,normal[2]*self.height_scaling_short])+np.array(P1)
        
        controlpoints=[tuple(controlpoint_short),tuple(controlpoint_long)]




        #*Some shit I was doing to generate images for power point \|/
        # Transformed_Mesh=self.Weld_Mesh_Surface.transform(self.T)
        # self.plotter.add_mesh(Transformed_Mesh, opacity=1,color=True)
        # all_points=np.array(list(controlpoint_short))
        # long_points=np.array(list(controlpoint_long))
        # poly = pv.PolyData(np.array(all_points))
        # self.plotter.add_mesh(all_points,color='red',point_size=30)
        # self.plotter.add_mesh(long_points,color='black',point_size=30)
        

        # # poly["My Labels"] = [f"{i}" for i in range(poly.n_points)]
        # # self.plotter.add_point_labels(poly, "My Labels", point_size=30, font_size=40)
        # # # orientation_vector=edge.orientation_vectors[j]
        # # # vector_position=np.array(edge.Path[j])-np.array(orientation_vector)/100
        # arrow=pv.Arrow(P1,np.array(normal),scale=1/25)
        # self.plotter.add_mesh(arrow,color='blue')
        # self.plotter.show()
        # # # self.plotter.add_point(P1)
        # exit()
        
        return controlpoints
        


class Inverse_Kinematics_Solver:
    #*This is the class that contains all of the move it functionality and the IK solver
    def __init__(self,weld_on=False,sound_on=False):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_node',
                anonymous=False)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        #*Initialize the move it node and all the move it commander objects^
        self.aligned_tool0_vector=np.array([0,1,0])
        #*This is basically noting that the tool is pointed along the ee y axis
        
        group_name = "manipulator"
        move_group=moveit_commander.MoveGroupCommander(group_name)
        #*^ create the move group refrence
        self.move_group = move_group
        #*Set up the IK solver and defualt it to Speed
        self.IK_SOLVER = IK("base_link",
                "tool0",solve_type="Speed")
        
        # self.weld_velocity=0.1
        # self.transport_velocity=0.005*2*2*2*2*2*2*2*2*2
        self.transport_velocity=0.5*3
        self.weld_velocity=0.06
        self.interior_transport_velocity=self.transport_velocity/2
        self.tacking_time=1200
        #*These are the parameters that influence the robots speed and welding speed etc.^^^
        self.pose_default=self.set_pose(Transformation=np.eye(4),set_as_default=False)
        #TODO: why did I do this here????^^^ I belive this is properly done up a level in the TCP class
        #*defualt pose of the weld mesh*^
        self.weld_on=weld_on

        #*Sounds and topic publishers \|/
        if weld_on:
            print('Iniitializing Arduino Communiation:')
            self.create_publishers()
        self.sound_on=sound_on
        if sound_on:
            self.working_sound = vlc.MediaPlayer("working.mp3")
            self.working_sound.audio_set_volume(200)
            self.tacking_sound = vlc.MediaPlayer('tacking.mp3')
            self.tacking_sound.audio_set_volume(150)

    def get_fk(self,joint_values):
        #*This gets the forward kinematics for any pose and returns the point but not currently the orientatio
        rospy.wait_for_service('compute_fk')
        gpf=moveit_msgs.srv.GetPositionFK()
        fk=rospy.ServiceProxy('compute_fk',gpf)
        header=std_msgs.msg.Header()
        print('header')
        print(header)
        header.frame_id='base_link'
        fk_link_names=self.robot.get_link_names()
        robot_state=self.robot.get_current_state()
        robot_state.joint_state.position=joint_values
        sol=fk(header,fk_link_names,robot_state)
        sol=sol.pose_stamped[-1].pose.position
        Point=[sol.x,sol.y,sol.z]
        return Point

        
    
    def return_read_point(self,point=None,fake_cord=None,fake_rotation=None,noise_scale=None,return_rotation=False):
        #*This just returns the current read point from the robot but can also accept fake rotations and cords etc. for testing
        if fake_cord==None and fake_rotation==None:
            pose= self.move_group.get_current_pose().pose
            position= [pose.position.x,pose.position.y,pose.position.z]
            orientation= [pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
            R=scipy.spatial.transform.Rotation.from_quat(orientation)
            # print(R.as_matrix())
            measurement_delta=(50.86-39.18)/1000
            point=measurement_delta*np.matmul(R.as_matrix(),[0,np.sqrt(2)/2,np.sqrt(2)/2])+np.array(position)
            # point=np.array(position)
            # print('vector offset:')
            # print(measurement_delta*np.matmul(R.as_matrix(),[0,np.sqrt(2)/2,np.sqrt(2)/2]))
            if return_rotation==True:
                return point,orientation
            else:
                return point
        else:
            position= np.matmul(fake_rotation,np.array(point))+fake_cord
            if noise_scale!=None:
                position+=np.random.rand(3)*noise_scale
            return position
        
    def set_pose(self,Transformation,set_as_default=True):
        #*I belive this set the pose for the weld mesh
        pose=geometry_msgs.PoseStamped()
        pose.pose.position.x=Transformation[0,3]
        pose.pose.position.y=Transformation[1,3]
        pose.pose.position.z=Transformation[2,3]
        # print(Transformation[0:3,0:3])
        r=scipy.spatial.transform.Rotation.from_matrix(Transformation[0:3,0:3])
        quat=r.as_quat()
        header=std_msgs.msg.Header()
        header.frame_id="base_link"
        header.seq=int(1)

        header.stamp.secs=rospy.Time.now().secs
        header.stamp.nsecs=rospy.Time.now().nsecs
        pose.pose.orientation.x=quat[0]
        pose.pose.orientation.y=quat[1]
        pose.pose.orientation.z=quat[2]
        pose.pose.orientation.w=quat[3]
        pose.header=header
        if set_as_default==True:
            self.pose_default=pose
        else:
            return pose
        
        
    def create_publishers(self):
        self.toggle_weld=rospy.Publisher('/enable_weld',std_msgs.msg.Empty,queue_size=10)
        self.jog=rospy.Publisher('/jog',std_msgs.msg.Empty,queue_size=10)
        self.purge=rospy.Publisher('/purge',std_msgs.msg.Empty,queue_size=10)
        self.mode=rospy.Publisher('/mode',std_msgs.msg.Empty,queue_size=10)
        self.weld_set=rospy.Publisher('/weld_voltage_set',std_msgs.msg.Int64,queue_size=10)
        self.wire_set=rospy.Publisher('/wire_feed_set',std_msgs.msg.Int64,queue_size=10)
        self.tack=rospy.Publisher('/tack',std_msgs.msg.Float64,queue_size=10)
    
    def add_weld_mesh(self):
        #*Add the weld mesh with the pose defualt positioning it in the work space
        self.scene.add_mesh(name='weld_mesh', pose=self.pose_default, filename='Weld_Mesh_sangam_roman_edit.stl')
        x=self.scene.get_objects()
        self.scene.add_object(x['weld_mesh'])
        moveit_commander.ApplyPlanningSceneRequest(x['weld_mesh'])
        #*Ive used this exit so it exits the program very quickly after adding the mesh so I can verify its position without the robot moving
        # exit()
        
    def get_rotation_from_vectors(self,v1,v2):
        #*uhhh what am I doing here
        #*assumming both of these vectors are in the x-y plane this finds the rotation between them we use this to position our welding assuming always that the global z is aligned with the -z of the body or ee
        k=np.cross(v1,v2)
        p=scipy.spatial.transform.Rotation.from_rotvec((np.pi*np.array([0,1,0])))
        if tuple(k)==tuple(np.zeros(3)):
            r=p
        else:
            k=k/np.linalg.norm(k)
            theta=np.arccos(np.dot(v1,v2)/(np.linalg.norm(v1)*np.linalg.norm(v2)))
            r=scipy.spatial.transform.Rotation.from_rotvec((theta*np.array(k)))
            r=r*p
        #*We return this rotation
        return r
    
    def get_IK(self,point,orientation_vector,seed_state=None):
        #*This is the method to get and return the inverse kinematics based on the orientation vector and point passed in
        if type(seed_state)==type(None):
            seed_state=self.move_group.get_current_joint_values()
        x=point[0]
        y=point[1]
        z=point[2]
        print(orientation_vector)

        r=self.get_rotation_from_vectors(self.aligned_tool0_vector,orientation_vector)
        #*get the rotation from the aligned tool vector and the desired tool vector
        #TODO: what if we didnt want to stay in the xy plane, unlikely but how can we make this program more robust
        quat=r.as_quat()

        sol = (self.IK_SOLVER.get_ik(seed_state,
                                x, y, z,
                                quat[0], quat[1], quat[2], quat[3])
                                )
        
        if not(sol==None):
            angles=list(sol)
        else:
            angles=None
        #*Return the angles

        return angles
    
    def send_edge_to_moveit_scurve(self,edge,tacking,weld_it):
        #*Just pass in the edge and the options for tacking and weld_it

        #*This sends the code to move it and spaces the joint angles via a sigmound curve

        #TODO: I think theres some mixed up functionality in here
        def print_edge():
            #*THis prints the edges position time and velocity over time
            fig, axs = plt.subplots(6,3)
            fig.suptitle('Joint Angle Solutions for each point')

            axs[0,0].scatter(time,Angles[:,0]*180/np.pi)
            axs[0,0].set_title('Q0_ikp')
            axs[0,1].scatter(time,Velocities[:,0]*180/np.pi)
            axs[0,1].set_title('Q0_ikv')
            axs[0,2].scatter(time,Accelerations[:,0]*180/np.pi)
            axs[0,2].set_title('Q0_ika')


            axs[1,0].scatter(time,Angles[:,1]*180/np.pi)
            axs[1,0].set_title('Q1_ik')
            axs[1,1].scatter(time,Velocities[:,1]*180/np.pi)
            axs[1,1].set_title('Q1_ikv')
            axs[1,2].scatter(time,Accelerations[:,1]*180/np.pi)
            axs[1,2].set_title('Q1_ika')


            axs[2,0].scatter(time,Angles[:,2]*180/np.pi)
            axs[2,0].set_title('Q2_ik')
            axs[2,1].scatter(time,Velocities[:,2]*180/np.pi)
            axs[2,1].set_title('Q2_ikv')
            axs[2,2].scatter(time,Accelerations[:,2]*180/np.pi)
            axs[2,2].set_title('Q2_ika')

            axs[3,0].scatter(time,Angles[:,3]*180/np.pi)
            axs[3,0].set_title('Q3_ik')
            axs[3,1].scatter(time,Velocities[:,3]*180/np.pi)
            axs[3,1].set_title('Q3_ikv')
            axs[3,2].scatter(time,Accelerations[:,3]*180/np.pi)
            axs[3,2].set_title('Q3_ika')

            axs[4,0].scatter(time,Angles[:,4]*180/np.pi)
            axs[4,0].set_title('Q4_ik')
            axs[4,1].scatter(time,Velocities[:,4]*180/np.pi)
            axs[4,1].set_title('Q4_ikv')
            axs[4,2].scatter(time,Accelerations[:,4]*180/np.pi)
            axs[4,2].set_title('Q4_ika')


            axs[5,0].scatter(time,Angles[:,5]*180/np.pi)
            axs[5,0].set_title('Q5_ikpp')
            axs[5,1].scatter(time,Velocities[:,5]*180/np.pi)
            axs[5,1].set_title('Q5_ikv')
            axs[5,2].scatter(time,Accelerations[:,5]*180/np.pi)
            axs[5,2].set_title('Q5_ika')

            plt.show()

        def weld_edge():
            #*Executes a weld edge
            if self.sound_on==True:
                rospy.sleep(0.1)
                self.working_sound.play()
                rospy.sleep(1)
            
            if self.weld_on==True and weld_it:
                self.toggle_weld.publish()
            path.joint_trajectory.header.stamp.secs=rospy.Time.now().secs
            path.joint_trajectory.header.stamp.nsecs=rospy.Time.now().nsecs
            bool=self.move_group.execute(path,wait=True)
            # print(bool)
            if self.weld_on==True and weld_it:
                self.toggle_weld.publish()
            if bool==False:
                print(bool)
                exit()
                #* This corrects the current angles if the path fails due to them not matching the start but I think id rather kill the program if this happens
                correct_angles()

        def tack_edge():
            #*Executes a tack edge
            path.joint_trajectory.header.stamp.secs=rospy.Time.now().secs
            path.joint_trajectory.header.stamp.nsecs=rospy.Time.now().nsecs
            bool=self.move_group.execute(path,wait=True)
            print(bool)
            if self.sound_on==True:
                rospy.sleep(0.1)
                self.tacking_sound.play()
                rospy.sleep(1)

            if self.weld_on==True and tacking==True and bool and weld_it:

                self.tack.publish(self.tacking_time)
                rospy.sleep(2)
            elif bool==False:
                print(bool)
                exit()
                correct_angles()
                #* This corrects the current angles if the path fails due to them not matching the start but I think id rather kill the program if this happens

        def correct_angles():
            #*Moves the robot to the starting joint angles
            curr=list(self.move_group.get_current_joint_values())
            for n in range(len(curr)):
                if not(math.isclose(curr[n]*np.pi/180,edge.angles[0][n]*np.pi/180,abs_tol=10**-3)):
                    #*Sanity check
                    print('WARNING STARTING ANGLES DO NOT MATCH CURRENT ANGLES_IK_SOLVER')
                    print(edge.angles[0])
                    print(self.move_group.get_current_joint_values())
                    exit()
            # curr=list(self.move_group.get_current_joint_values())
            joint_goal = deepcopy(curr)
            for n in range(len(curr)):
                joint_goal[n] =edge.angles[0,n]
            self.move_group.go(joint_goal, wait=True)
            print('reseting joint angles')
            rospy.sleep(3)
            execute_edge()

        def execute_edge():
            #*Executes the edge based on its type
            if edge.type=='weld':
                weld_edge()
            else:  
                tack_edge()

        def time_path_distance():
            #*This times the path based on a average velocity and the path distance
            if edge.type=='weld':
                max_desired_velocity=self.weld_velocity
            else:
                if edge.interior==True:
                    max_desired_velocity=self.interior_transport_velocity
                else:
                    max_desired_velocity=self.transport_velocity
            
            scurve=s_curve(b=30,c=1)
            x=np.linspace(0,1,len(edge.Path))
            velocity_ratio=max_desired_velocity/scurve.ds(0.5)
            Path_Distance=scurve.s(1)-scurve.s(0)
            scurve_avg_velocity=Path_Distance/1
            average_velocity=scurve_avg_velocity*velocity_ratio
            if edge.type=='weld':
                time_step_est=edge.Path_Distance_Direct/average_velocity
                #*Times itself along the welded line not the ee position for the welds
            else:
                time_step_est=edge.Path_Distance/average_velocity
            # print(time_step_est)
            velocity_magnitude=[]
            acceleration_magnitude=[]
            time=[]
            
            for n in range(len(Angles)):
                t=x[n]
                time.append(t*time_step_est)
                velocity_magnitude.append(scurve.ds(t)*velocity_ratio)
                acceleration_magnitude.append(scurve.dds(t)*velocity_ratio)
            return time,velocity_magnitude,acceleration_magnitude

        def time_joint_distance(desired_time_span=None,desired_angular_velocity=None):
            #*This times the path based on an average joint velocity and joint distance
            if desired_time_span==None and desired_angular_velocity==None:
                scurve=s_curve(b=30,c=1)
                desired_time_span=10
                time_step_est=desired_time_span
                Path_Distance=scurve.s(1)-scurve.s(0)
                ratio=scurve.ds(0.5)/(Path_Distance/1)
                max_desired_velocity=ratio*(edge.Cost_Function/time_step_est)
                velocity_ratio=max_desired_velocity/scurve.ds(0.5)
            elif desired_time_span==None:
                scurve=s_curve(b=30,c=1)
                max_desired_velocity=desired_angular_velocity
                velocity_ratio=max_desired_velocity/scurve.ds(0.5)
                Path_Distance=scurve.s(1)-scurve.s(0)
                scurve_avg_velocity=Path_Distance/1
                average_velocity=scurve_avg_velocity*velocity_ratio
                time_step_est=edge.Cost_Function/average_velocity

            velocity_magnitude=[]
            acceleration_magnitude=[]
            time=[]
            x=np.linspace(0,1,len(edge.angles))
            for n in range(len(Angles)):
                t=x[n]
                time.append(t*time_step_est)
                velocity_magnitude.append(scurve.ds(t)*velocity_ratio)
                acceleration_magnitude.append(scurve.dds(t)*velocity_ratio)
            return time,velocity_magnitude,acceleration_magnitude

        def get_VA():
            #*Calculates the joint velocity and accelerations based on the time arrau and positions
            desired_joint_velocities=[]
            for n in range(len(Angles)):
                if n==0:
                    VAF=(np.array(Angles[n+1])-np.array(Angles[n]))/(time[n+1]-time[n])
                    desired_joint_velocities.append(VAF)
                elif n==len(Angles)-1:
                    VAB=(np.array(Angles[n])-np.array(Angles[n-1]))/(time[n]-time[n-1])
                    # VAB=np.array([0,0,0,0,0,0])
                    desired_joint_velocities.append(VAB)
                else:
                    VAF=(np.array(Angles[n+1])-np.array(Angles[n]))/(time[n+1]-time[n])
                    VAB=(np.array(Angles[n])-np.array(Angles[n-1]))/(time[n]-time[n-1])
                    VAA=(VAF+VAB)/2
                    desired_joint_velocities.append(VAA)
                    
            desired_joint_accelerations=[]

            for n in range(len(Angles)):
                if n==0:
                    AAF=(np.array(desired_joint_velocities[n+1])-np.array(desired_joint_velocities[n]))/(time[n+1]-time[n])
                    desired_joint_accelerations.append(AAF)
                elif n==len(Angles)-1:
                    AAB=(np.array(desired_joint_velocities[n])-np.array(desired_joint_velocities[n-1]))/(time[n]-time[n-1])
                    # AAB=np.array([0,0,0,0,0,0])
                    desired_joint_accelerations.append(AAB)
                else:
                    AAF=(np.array(desired_joint_velocities[n+1])-np.array(desired_joint_velocities[n]))/(time[n+1]-time[n])
                    AAB=(np.array(desired_joint_velocities[n])-np.array(desired_joint_velocities[n-1]))/(time[n]-time[n-1])
                    AAA=(AAF+AAB)/2
                    desired_joint_accelerations.append(AAA)
            return desired_joint_velocities,desired_joint_accelerations
        
        
        Angles=edge.angles
        Path=edge.Path

        #*Read the type of edge
        if edge.type=='dwell':
            time,velocity_magnitude,acceleration_magnitude=time_joint_distance()
            tacking=False
        else:
            time,velocity_magnitude,acceleration_magnitude=time_path_distance()
        
        desired_joint_velocities,desired_joint_accelerations=get_VA()
        #* Get the velocities and accelerations
        Velocities=np.array(desired_joint_velocities)
        Accelerations=np.array(desired_joint_accelerations)
        
        rtj=trajectory_msgs.msg.JointTrajectory()
        header=std_msgs.msg.Header()
        header.frame_id="world"
        header.seq=int(1)

        rtj.joint_names=JOINT_NAMES
        points=[]

        for n in range(len(time)):
            Joint_Point=trajectory_msgs.msg.JointTrajectoryPoint()
            Joint_Point.positions=Angles[n]
            Joint_Point.time_from_start=rospy.Duration.from_sec(time[n])
            Joint_Point.velocities=Velocities[n]
            Joint_Point.accelerations=Accelerations[n]
            points.append(deepcopy(Joint_Point))
                
        state=RobotState()
        joint_state=JointState()
        joint_state.header = std_msgs.msg.Header()
        header.frame_id="world"
        header.seq=int(1)
        joint_state.name=JOINT_NAMES
        joint_state.position=list(self.move_group.get_current_joint_values())
        state.joint_state=joint_state
        rtj.points=points

        rtj.header=header
        path=RobotTrajectory(joint_trajectory=rtj)

        #*Execute the trajectory
        execute_edge()



