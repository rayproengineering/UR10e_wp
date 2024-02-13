#!/usr/bin/env python3

import copy
import math
import sys
import itertools
from copy import deepcopy
from distutils.file_util import move_file
from formatter import AbstractFormatter

import actionlib
import actionlib_msgs
import geometry_msgs.msg as geometry_msgs
import LKH_solver
import matplotlib as mpl
import matplotlib.pyplot as plt

import MIP_solver
import moveit.core.collision_detection
import moveit_commander
import moveit_msgs.msg
import networkx as nx
import numpy as np
import pyvista as pv
import rospy
import scipy
import sensor_msgs.msg
import shape_msgs
import std_msgs.msg
import trajectory_msgs.msg
import vlc
import Weld_Select
# from grid_class_v1 import *

import edge_classes
from edge_classes import *

from LKH_solver import *
from MIP_solver import *
from MIP_solver import MIP_solver_edge_constraints
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotState, RobotTrajectory
from networkx.algorithms import approximation
from scipy.interpolate import make_smoothing_spline
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trac_ik_python.trac_ik import IK
from Weld_Select import *
from Weld_Select import Create_Welds

import helper_functions_n_objects
from helper_functions_n_objects import *

#*These can be accessed from anywhere in the file
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

class condensed_solver():
    def __init__(self,Weld_Mesh=None,Z_offset=None):
        
        # self.end_effector_joint_limits=np.array([-226.62,237.65])*(np.pi/180)
        self.end_effector_joint_limits=np.array([-226.62,300])*(np.pi/180)
        # self.end_effector_joint_limits=np.array([-300,300])*(np.pi/180)
        # print(self.end_effector_joint_limits)
        self.aeejl=np.average(self.end_effector_joint_limits)
        
        if type(Weld_Mesh)!=type(None):
            self.Weld_Mesh_Surface=Weld_Mesh
            self.bounds=Weld_Mesh.GetBounds()
            self.Z_offset=abs(self.bounds[-1]-self.bounds[-2])*1.2
        elif type(Z_offset)!=type(None):
            print('No Weld Mesh Provided Using a Provided height offset of: '+str(Z_offset)+' meters')
            self.Z_offset=Z_offset
        else:
            self.Z_offset=0.18415
            print('No Weld Mesh or Height Offset Provided, defualting to a Height Offset of: '+str(self.Z_offset))

        self.control_distance=0.02
        self.weld_on=True

        self.final_resolution_per_joint_length=250
        self.final_resolution_per_path_length=1000
        self.min_final_resolution=200
        self.smoothness_coefficient_weld=2
        self.smoothness_coefficient_reorientatie=5
        self.weldb=3
        self.transportb=7
        
        self.tour_resolution_welds=20
        # self.reorientation_cost_vector=[1,0.83,0.57,0.33,0.19,0.10]
        # self.reorientation_cost_vector=[1,1,1,1,1,1]
        # alpha=0.8
        # self.reorientation_cost_vector=[10.0**alpha,8.3**alpha,5.7**alpha,3.3**alpha,1.9**alpha,1.0**alpha]
        self.reorientation_cost_vector=[10,10,10,5,2.5,1]
        self.reorientation_angular_rate=np.pi/2

        self.fig1=None
        self.fig2=None
        self.fig3=None
        

        #*DICTIONARY MANAGEMENT
        self.point_normal_joint_angle_dict={}
        self.normal_dict=None
        self.current_edge=None
        # plt.ion()
        self.clearence_point_dict={}
        self.adjust_by_normal_dict=None
        self.adjusted_point_dict=None
        self.node_dict=None


        # self.control_distance=0.05
        #*These are the joint limits I am using to further constrict the end effector
        #*This code is all pretty much strip mined from grid class 
    
    def set_plotter(self,plotter):
        self.plotter=plotter

    def cycle_test_ee_limits(self,cycles):
        joint_goal = self.move_group.get_current_joint_values()
        for n in range(0,cycles):
            if n%2==0:
                joint_goal[5]=self.end_effector_joint_limits[-1]
            else:
                joint_goal[5]=self.end_effector_joint_limits[0]
            self.move_group.go(joint_goal, wait=True)
            self.move_group.stop()

    def set_weld_goals(self,weld_lines=[],tack_points=[]):
        self.tack_points=tack_points
        self.weld_lines=weld_lines

    def set_transform(self,Transformation):
        self.T=Transformation

    def pre_compile_weld_tour_points(self):
        tour_points=None
        tour_normals=None
        for n in self.tour_stripped:
            x=int(np.where(np.array(self.tour)==n)[0][0])
            node_ahead=x+1
            node_behind=x-1
            print('x')
            print(x)
            print(self.node_dict)
            print(self.tour)
            x_plus=x+1
            if x_plus>=len(self.tour):
                x_plus=0+((x_plus)-len(self.tour))
            print('x_plus')
            print(x_plus)
            print(self.tour[x_plus])
            print(self.tour[x-1])
            
            # input('good?')
            # print(self.tour(x+1))
            # print(x+1)
            
            point_a=self.node_dict[self.tour[x-1]]
            point_b=self.node_dict[self.tour[x_plus]]
            normal_a=self.get_desired_point_normal(tuple(point_a))
            normal_b=self.get_desired_point_normal(tuple(point_b))
            point_a_adjusted=self.get_adjusted_point(point_key=tuple(point_a),normal=normal_a)
            point_b_adjusted=self.get_adjusted_point(point_key=tuple(point_b),normal=normal_b)
            
            if type(tour_points)==type(None):
                tour_points=[]
                tour_normals=[]
                tour_points.append([point_a_adjusted,point_b_adjusted])
                tour_normals.append([normal_a,normal_b])
            else:

                point_delta=np.array(tour_points[-1][1])-np.array(point_a_adjusted)
                point_delta=np.dot(point_delta,point_delta)
                print(tour_points)
                print(point_a_adjusted)
                print('considered zero?')
                print(math.isclose(point_delta,0.0,abs_tol=10**-3))
                input(point_delta)
                
                if math.isclose(point_delta,0.0,abs_tol=10**-3):
                    
                    tour_points.append([point_a_adjusted,point_b_adjusted])
                    tour_normals.append([normal_a,normal_b])
                    # tour_points.append(point_b_adjusted)
                    # tour_normals.append(normal_b)
                    # tour_points.append(point_a_adjusted)
                    # tour_points.append(point_b_adjusted)
                    # tour_normals.append(normal_a)
                    # tour_normals.append(normal_b)
                else:
                    # tour_points.append(point_a_adjusted)
                    tour_points.append([tour_points[-1][1],self.get_safety_points(tour_points[-1][1],tuple(tour_normals[-1][1]))])
                    tour_points.append([self.get_safety_points(tour_points[-1][1],tuple(tour_normals[-1][1])),self.get_safety_points(point_a_adjusted,tuple(normal_a))])
                    tour_points.append([self.get_safety_points(point_a_adjusted,tuple(normal_a)),point_a_adjusted])
                    tour_normals.append([tour_normals[-1][1],tour_normals[-1][1]])
                    tour_normals.append([tour_normals[-1][1],normal_a])
                    tour_normals.append([normal_a,normal_a])
                    
                    tour_points.append([point_a_adjusted,point_b_adjusted])
                    tour_normals.append([normal_a,normal_b])
                    


                    
        return tour_points,tour_normals

    def pre_compile_tack_tour_points(self):
        tour_points=[]
        tour_normals=[]

        for n in range(len(self.tour)-1):
            point_a=self.node_dict[self.tour[n]]
            point_b=self.node_dict[self.tour[n+1]]
            normal_a=self.get_desired_point_normal(tuple(point_a))
            normal_b=self.get_desired_point_normal(tuple(point_b))
            point_a_adjusted=self.get_adjusted_point(point_key=tuple(point_a),normal=normal_a)
            point_b_adjusted=self.get_adjusted_point(point_key=tuple(point_b),normal=normal_b)
            
            tour_points.append([point_a_adjusted,self.get_safety_points(point_a_adjusted,tuple(normal_a))])
            tour_points.append([self.get_safety_points(point_a_adjusted,tuple(normal_a)),self.get_safety_points(point_b_adjusted,tuple(normal_b))])
            tour_points.append([self.get_safety_points(point_b_adjusted,tuple(normal_b)),point_b_adjusted])
            
            tour_normals.append([normal_a,normal_a])
            tour_normals.append([normal_a,normal_b])
            tour_normals.append([normal_b,normal_b])

        return tour_points,tour_normals

    def pre_plot_solution(self,weld=True,show_every_point=False):
        #*This generates the 3D solution plot in pyvista over the weld mesh
        blue = np.array([12 / 256, 238 / 256, 246 / 256, 1.0])
        black = np.array([11 / 256, 11 / 256, 11 / 256, 1.0])
        grey = np.array([189 / 256, 189 / 256, 189 / 256, 1.0])
        yellow = np.array([255 / 256, 247 / 256, 0 / 256, 1.0])
        red = np.array([1.0, 0.0, 0.0, 1.0])
        

        self.plotter.clear()

        robot_location=np.array([0,0,0])
        label=['Center Of Robot Base']
        self.plotter.add_point_labels(robot_location,label,font_size=10,point_size=10,point_color='red')
        # all_points=np.array(list(self.node_dict.values())) 
        # poly = pv.PolyData(robot_location)
        
        # poly["My Labels"] = [f"{i}" for i in range(poly.n_points)]
        
        # self.plotter.add_point_labels(poly, "My Labels", point_size=30, font_size=40)
        mesh=deepcopy(self.Weld_Mesh_Surface)
        
        Transformed_Mesh=mesh.transform(self.T)
        
        self.plotter.add_mesh(Transformed_Mesh, opacity=0.10,color=True)
        if weld==True:
            tour_points,tour_normals=self.pre_compile_weld_tour_points()
        else:
            tour_points,tour_normals=self.pre_compile_tack_tour_points()
        # self.plotter.add_axes_at_origin(line_width=0.5)
        axi_l=0.25
        axi=[[0,0,0],[axi_l,0,0]],[[0,0,0],[0,axi_l,0]],[[0,0,0],[0,0,axi_l]]
        for n in axi:
            lines= pv.lines_from_points(n)
            self.plotter.add_mesh(lines,line_width=1,color=grey)


        # self.plotter.add_points(np.array(tour_points))
        label=['0']
        arrow=pv.Arrow((tour_points[0][0]-np.array(-tour_normals[0][0]*self.control_distance)),np.array(-tour_normals[0][0])*self.control_distance,scale=np.linalg.norm(tour_normals[0][0])*self.control_distance*2)
        self.plotter.add_point_labels(tour_points[0][0],label,font_size=30,point_size=10,point_color='blue')
        self.plotter.add_mesh(arrow,color=blue)
        
        written_tuples=[tuple(tour_points[0][0])]

        for n in range(len(tour_points)):
            if show_every_point==True:
                label=[str(n+1)]
                self.plotter.add_point_labels(tour_points[n][-1],label,font_size=30,point_size=10,point_color='blue')
            elif not(tuple(tour_points[n][-1]) in written_tuples):
                written_tuples.append(tuple(tour_points[n][-1]))
                label=[str(len(written_tuples)-1)]
                self.plotter.add_point_labels(tour_points[n][-1],label,font_size=30,point_size=10,point_color='blue')
            
            lines= pv.lines_from_points(tour_points[n])
            self.plotter.add_mesh(lines,line_width=5,color=black)
            arrow=pv.Arrow((tour_points[n][-1]-np.array(-tour_normals[n][-1])*self.control_distance),np.array(-tour_normals[n][-1])*self.control_distance,scale=np.linalg.norm(tour_normals[n][-1])*self.control_distance*2)
            self.plotter.add_mesh(arrow,color=blue)
        
        # # for n in 
        # lines= pv.lines_from_points(tour_points)
        # self.plotter.add_mesh(lines,line_width=5,color=black)
        # # # tour_path=[]
        # # orientation_vectors=[]
        # # print(self.tour_edges)
        # # # angles=[]
        # for n in range(len(self.edge_object_list)):
        #     edge=self.edge_object_list[n]
        #     orientation_vectors=orientation_vectors+edge.orientation_vectors
        #     # angles=angles+list(edge.angles)
        #     lines= pv.lines_from_points(edge.Path)
        #     self.plotter.add_mesh(lines,line_width=5,color=black)                
        #     #*This is code that creates and displays the orientation vectors at each point, depending on when this is called in the overall code it may throw an error
        #     #TODO:Review this and make sure it is functional
        #     # for j in range(len(edge.Path)):
        #     #     orientation_vector=edge.orientation_vectors[j]
        #     #     vector_position=np.array(edge.Path[j])-np.array(orientation_vector)/100
        #     #     arrow=pv.Arrow(vector_position,np.array(orientation_vector),scale=np.linalg.norm(orientation_vector)/100,)
        #     #     self.plotter.add_mesh(arrow,color=blue)
            
        # all_points=np.array(list(self.node_dict.values())) 
        # poly = pv.PolyData(np.array(all_points))
        # poly["My Labels"] = [f"{i}" for i in range(poly.n_points)]
        # self.plotter.add_point_labels(poly, "My Labels", point_size=30, font_size=40)
        self.plotter.show(auto_close=False)

    def run_solution(self,Tack=True,Weld=True,solution_type='tsp'):
        self.configuration=self.configuration_type(self.move_group.get_current_joint_values())
        self.lock_geometric_type(self.end_effector_joint_limits)

        if Tack==True:
            if solution_type=='tsp':
                self.run_tsp_tack()
                self.tour_count=None
                self.pre_plot_solution(weld=False)
                
                self.execute_tack_points(get_next_tack=self.tour_tack_get)
            else:
                self.execute_tack_points()
        if Weld==True:
            if solution_type=='tsp':
                self.run_tsp_weld()
                self.tour_count=None
                self.execute_weld_lines(get_next_weld=self.tour_weld_get)
            else:
                self.execute_weld_lines()
                # input('compare tsp')
                # self.run_tsp_weld()
    #DICTONARY MANAGEMENT\|/:

    def get_adjusted_point(self,point_key,normal=None,transform_normal=False,transform_point=True,adjusted_point_dict=None):
        
        if adjusted_point_dict==None:
            if type(self.adjusted_point_dict)==type(None):
                self.adjusted_point_dict={}
            adjusted_point_dict=self.adjusted_point_dict
        
        if type(normal)==type(None):
            normal=self.normal_dict[point_key]

        if transform_normal==True:
            normal=self.Transform(normal,normal=True)
        # if transform_point==True:
        if point_key in list(adjusted_point_dict.keys()):
            adjusted_point=adjusted_point_dict[point_key]
        else:
            adjusted_point=np.array(deepcopy(point_key))
            
            if transform_point:
                adjusted_point=self.Transform(adjusted_point)
            
            #* Add to the non-normal dictionary here
            adjusted_point=self.adjust_by_normal(point=adjusted_point,normal=normal,sign=1,scaling=None)

            adjusted_point_dict[point_key]=deepcopy(adjusted_point)

        return adjusted_point
    
    def get_safety_points(self,point,normal=[0,0,0]):
        
        if tuple(point) in list(self.clearence_point_dict.keys()):
            sp=self.clearence_point_dict[tuple(point)]
        
        else:
            sp=np.array(deepcopy(point))
            sp[-1]= self.Transform([0,0,0])[-1]+self.Z_offset+normal[-1]*self.control_distance

            # sp[0]+=normal[0]*self.control_distance
            # sp[1]+=normal[1]*self.control_distance

            self.clearence_point_dict[tuple(point)]=deepcopy(sp)
        
        return sp

    def get_desired_point_normal(self,point):
        if type(self.normal_dict)==type(None):
            self.normal_dict={}
        
        if point in list(self.normal_dict.keys()):
            normal=self.normal_dict[point]
                    
        else:
            normal=self.compute_normal(point)
            self.normal_dict[point]=normal

        return normal

    def get_general_point_normal(self,point):
        if not(point in list(self.point_normal_joint_angle_dict.keys())):
            if (point in list(self.normal_dict.keys())):
                normal=self.normal_dict[point]
                self.point_normal_joint_angle_dict[point]={tuple(normal):None}
                return normal
            else:
                return None
        else:
            normals=self.point_normal_joint_angle_dict[point]
            return normals
                    
    def get_point_normal_angles(self,point,normal):
        
        def call_angles():
            angles=None
            while type(angles)==type(None):
                angles=self.get_IK(point=point,orientation_vector=[-normal[0],-normal[1],0])
            return angles
        
        if not(tuple(point) in list(self.point_normal_joint_angle_dict.keys())): 
            angles=call_angles()
            self.point_normal_joint_angle_dict[tuple(point)]={tuple(normal):deepcopy(angles)}
        if not(tuple(normal) in self.point_normal_joint_angle_dict[tuple(point)].keys()) or type(self.point_normal_joint_angle_dict[tuple(point)][tuple(normal)])==type(None):
            angles=call_angles()
            self.point_normal_joint_angle_dict[tuple(point)].update({tuple(normal):deepcopy(angles)})
        else:
            angles=self.point_normal_joint_angle_dict[tuple(point)][tuple(normal)]
        
        return angles

    def adjust_by_normal(self,point,normal,sign,scaling=None):
        #*Assumed normal and point are in same frame of refrence!
        
        if self.adjust_by_normal_dict==None:
            self.adjust_by_normal_dict={}

        if scaling==None:
            scaling=self.control_distance

        if tuple(point) in list(self.adjust_by_normal_dict.keys()):
            if tuple(normal) in list(self.adjust_by_normal_dict[tuple(point)]):
                if np.sign(sign) in list(self.adjust_by_normal_dict[tuple(point)][tuple(normal)]):
                    return self.adjusted_point_dict[tuple(point)][tuple(normal)][np.sign(sign)]

        adjusted_point=deepcopy(point)
        adjusted_point[0]+=normal[0]*scaling*np.sign(sign)
        adjusted_point[1]+=normal[1]*scaling*np.sign(sign)
        adjusted_point[2]+=normal[2]*scaling*np.sign(sign)

        self.adjusted_point_dict[tuple(point)]={tuple(normal):{np.sign(sign):adjusted_point}}
        self.adjusted_point_dict[tuple(adjusted_point)]={tuple(normal):{-np.sign(sign):point}}

        return adjusted_point

    #*DICTIONARY MANAGEMENT

    #*IMPLIMENT SOLVERS BELLOW: WORK IN PROGRESS
    def tour_tack_get(self,tack_points):
        if type(self.tour_count)==type(None):
            self.tour_count=0
            self.reorder_tour_tack()
            self.live_tacs=deepcopy(tack_points)
        else:
            self.tour_count+=1

        # normals,closest_point=,tack_points=get_next_tack(tack_points)
        # normals=[self.get_desired_point_normal(tuple(n)) for n in current_tack_points]


        # adjusted_tack_points=[]
        # for n in range(len(current_tack_points)):
        #     adjusted_tack_points.append(self.get_adjusted_point(point_key=tuple(current_tack_points[n]),normal=normals[n]))
        # closest_point_index=self.tour[self.tour_count]
        clostest_point=self.live_tacs[self.tour[self.tour_count]]
        closest_normal=self.get_desired_point_normal(tuple(clostest_point))
        # x=scipy.spatial.distance.cdist(np.array([point]),np.array(adjusted_tack_points),'euclidean')
        # closest_point_index=np.argmin(x)
        closest_point_adjusted=self.get_adjusted_point(point_key=tuple(clostest_point),normal=closest_normal)
        # closest_normal=normals[closest_point_index]

        # print(current_tack_points)
        tack_points.remove(clostest_point)
        # tack_points.remove()

        return closest_normal,closest_point_adjusted,tack_points

        # return normals,closest_point,tack_points

        # get_next_tack(tack_points)

    def tour_weld_get(self,current_weld_lines,point=None):
        if type(self.tour_count)==type(None):
            self.solution_type='tsp'
            self.tour_count=0
            self.live_welds=deepcopy(current_weld_lines)
            
            
            
            self.reorder_tour_weld()

            self.pre_plot_solution(weld=True,show_every_point=True)
            
            self.repeat=0
            wpose = self.move_group.get_current_pose().pose
            self.last_point=[wpose.position.x,wpose.position.y,wpose.position.z]
            self.point_que=[]
            self.weld_tid_wi={}
        
        
        # else:
        #     self.tour_count+=1
        
        if type(point)!=type(None):
            print('POINT PASSED IN')
            print(point)
            print('^Thats the point again but inside tour weld get')
            
            weld_id=(self.tour_count)+self.repeat
            self.repeat+=1
            # *self.repeat
            self.point_que.append(weld_id)
            # distance=np.linalg.norm(np.array(self.last_ending_point)-np.array(point))    
        else:
            self.repeat=0
            self.point_que=[]
            weld_id=deepcopy(self.tour_count)
            self.point_que.append(weld_id)
            self.tour_count+=1
            
            wpose = self.move_group.get_current_pose().pose
            point=[wpose.position.x,wpose.position.y,wpose.position.z]
            point=np.array(point)


        # print('weld_id')
        # print(weld_id)
        # input('where is this in the tour')
        
        # all_points=np.array(current_weld_lines)
        # all_points=all_points.reshape(all_points.shape[0]*all_points.shape[1],all_points.shape[2])
        # normals=[self.get_desired_point_normal(tuple(n)) for n in all_points]
        # adjusted_weld_points=[]
        
        # for n in range(len(all_points)):
        #     adjusted_weld_points.append(self.get_adjusted_point(point_key=tuple(all_points[n]),normal=normals[n]))

        

        # current_weld_adjusted=deepcopy([adjusted_weld_points[closest_point_index],adjusted_weld_points[(closest_point_index+1+(-2*remainder))]])
        
        # current_normals=[normals[closest_point_index],normals[(closest_point_index+1+(-2*remainder))]]
        # if repeat==0:
        
        print(len(current_weld_lines))

        last_pile=deepcopy(current_weld_lines)
        print(len(current_weld_lines))
        
        if np.array(current_weld_lines).size==0 or weld_id>=len(self.tour_stripped):
            return  None,None,None,None,None,np.inf,[]
        
        current_weld_lines=current_weld_lines[0:-1]
        
        current_weld_id=self.tour_stripped[weld_id]        

        # else:
        print(self.tour_stripped)
        print(current_weld_id)
        print('hai')
        weld=self.weld_dict[current_weld_id]
        total_id=np.where(np.array(self.tour)==self.tour_stripped[weld_id])[0][0]
        
        self.weld_tid_wi.update({weld_id:total_id})
                
        if self.point_dic[tuple(weld[0])]!=self.tour[(total_id-1)]:
            weld=[weld[-1],weld[0]]
        
        # next_weld_id=self.tour_stripped[weld_id+1]

        
        # print(self.tour_stripped[weld_id])
        # print(weld_id)
        # print(next_weld_id)
        # input('bada boom')

        # next_total_id=np.where(np.array(self.tour)==next_weld_id)[0][0]
        # self.weld_tid_wi.update({next_weld_id:next_total_id})
        
        # print(next_total_id-1)
        # input('what you think')



        # next_weld=deepcopy(self.weld_dict[next_weld_id])
        # if self.point_dic[tuple(next_weld[0])]!=self.tour[(next_total_id-1)]:
        #     next_weld=[next_weld[-1],next_weld[0]]
        
        # point_d=self.get_adjusted_point(point_key=tuple(next_weld[0]))
        
        # if self.point_dic[tuple(next_weld[0])]!=self.tour[(next_total_id-1)]:
        #             point_d=self.get_adjusted_point(point_key=tuple(next_weld[-1]))
        
        normal_a=self.get_desired_point_normal(tuple(weld[0]))
        normal_c=self.get_desired_point_normal(tuple(weld[-1]))
        # input('normals')
        # input(normal_a)
        # input(normal_c)
        
        # normal_b=(normal_a+normal_c)/2
        # normal_b=np.array(normal_b)/np.linalg.norm(normal_b)
        #     #*This middle node then has the orientation of the average between these two start and finish normal
        #     #TODO: Consider chaning this
        #     #*CHECK
        point_a=self.get_adjusted_point(point_key=tuple(weld[0]),normal=normal_a)
        point_c=self.get_adjusted_point(point_key=tuple(weld[-1]),normal=normal_c)
        
        
        #* Minus Normal
        wp1=self.adjust_by_normal(point=point_a,normal=normal_a,sign=-1)
        wp2=self.adjust_by_normal(point=point_c,normal=normal_c,sign=-1)

        control_points=np.array(euclidean_linspace(wp1,wp2,100))
        p1=point_a
        p2=point_c
        # self.last_ending_point=point_c
        normal1=normal_a
        normal2=normal_c
        
        # distance=np.linalg.norm(point_a-point)
        # print('come on man')

        # x=scipy.spatial.distance.cdist(np.array([point]),np.array(next_weld),'euclidean')
        # distance=np.min(x)
        # input(point)
        # input([p1,p2])
        # print(distance)
        #return 
        print('distance')
        distance=np.linalg.norm(p1-point)
        

        # if self.repeat!=0:
        #     if 

        # input(distance)
        # distance=0

        self.last_point=deepcopy(p2)
        
        return p1,p2,normal1,normal2,control_points,distance,last_pile
    
    def run_tsp_tack(self):
        self.node_dict={}
        G=self.generate_graph_simple()
        self.solve_tac_tour(G,runs=10,trials=1000)
        self.networkx_plot(G)

    def run_tsp_weld(self):
        self.node_dict={}
        # weld_cost_function
        G=self.generate_graph_simple(add_all_nodes=False,cost_function=self.weld_cost_function)
        wG,weld_dict,point_dic=self.add_weld_nodes(G)
        
        self.solve_weld_tour(G=wG,weld_dict=weld_dict,point_dic=point_dic)
        self.networkx_plot(wG,weld=True)
        # self.pre_compute_tour_points()
        

    def generate_graph_simple(self,add_all_nodes=False,cost_function=None,cost_type='joint',weld_lines=True,tack_points=True,variable_to_set=None):
        if type(cost_function)==type(None):
            cost_function=self.tack_cost_function
        
        
        #*Make this function as robust as you need currently
        #*This computes the edges connecting all tack and weld points
        
        # print('tack points:')
        # print(self.tack_points)
        # print('weld points:')
        # print(self.weld_lines)
        
        if weld_lines and tack_points:
            all_points=[n.tolist() for (i,n) in self.weld_lines]+[n.tolist() for n in self.tack_points]
        elif weld_lines:
            all_points=[n.tolist() for (i,n) in self.weld_lines]
        elif tack_points:
            all_points=[n.tolist() for n in self.tack_points]
        else:
            print('No Points Given for Graph Generation')
            return None

        all_points=np.unique(all_points,axis=0)
        
        G=nx.Graph()
        dnodes={n:tuple(all_points[n]) for n in range(len(all_points[:]))}
        
        self.node_dict=deepcopy(dnodes)

        print(dnodes)

        dset=set(list(dnodes.keys()))
        print('dset')
        print(dset)
        # dset=dnodes[:]
        G.add_nodes_from(dnodes)

        # print(G.nodes())
        # print(dset)
        for u in itertools.combinations(dset,r=2):
            print(u)
        # input('check this')
        
        # for (u,v) in itertools.product(dset,dset,repeat=0):
        for u in itertools.combinations(dset,r=2):
            print(u)
            
            #*Write the algorithm for the weights lol
            #*We need the points associated with each other node and cost and what not
            #*First look up the normal.....
            
            normals=[]
            safety_points=[]
            adjusted_points=[]
            angles=[]
            
            for node in u:
                point=dnodes[node]
                print('printing point:'+str(point))
                #*FIRST NORMAL CALL HERE
                # normals=[self.get_desired_point_normal(tuple(n)) for n in current_tack_points]
                # adjusted_tack_points=[]
                # for n in range(len(current_tack_points)):
                

                normal=self.get_desired_point_normal(point)
                adjusted_points.append(self.get_adjusted_point(point_key=tuple(point),normal=normal))

                end_ee_ik=self.get_point_normal_angles(adjusted_points[-1],normal)
                angles.append(deepcopy(end_ee_ik))
                sp=self.get_safety_points(adjusted_points[-1],tuple(normal))
                safety_points.append(sp)
                end_ee_ik=self.get_point_normal_angles(sp,normal)
                angles.append(deepcopy(end_ee_ik))
            
            #*CHECK THIS:
            joint_values=[angles[0],angles[1],angles[3],angles[2]]
            # input(joint_values)
            if cost_type=='joint':
                cost_array=joint_values
            else:
                cost_array=[adjusted_points[0],safety_points[0],safety_points[1],adjusted_points[-1]]

            if add_all_nodes==True:

                # G.add_nodes_from({:safety_points})
                for n in range(len(safety_points)):
                    node_int=int(list(dnodes.keys())[-1]+1)
                    if n==0:
                        dnodes.update({node_int:safety_points[0]})
                        #*Gotta update the way you add edges and dictionary
                        G.add_edge(u[0],node_int)

                        nx.set_edge_attributes(G,{(u[0],node_int):self.tack_cost_function(positions=cost_array[0:2],cost_type=cost_type)},'weight')

                    elif n==len(safety_points)-1:
                        dnodes.update({node_int:safety_points[0]})
                        G.add_edge(node_int,u[-1])
                        
                        nx.set_edge_attributes(G,{(node_int,u[-1]):self.tack_cost_function(positions=cost_array[1:3],cost_type=cost_type)},'weight')

                    else:
                        dnodes.update({node_int:safety_points[n]})
                        dnodes.update({node_int+1:safety_points[n+1]})
                        G.add_edge(node_int,node_int+1)
                        
                        nx.set_edge_attributes(G,{(node_int,node_int+1):self.tack_cost_function(positions=cost_array[2:],cost_type=cost_type)},'weight')

                
            else:
                G.add_edge(u[0],u[1])
                nx.set_edge_attributes(G,{(u[0],u[1]):self.tack_cost_function(positions=cost_array,cost_type=cost_type)},'weight')

        return G
    
    def weld_cost_function(self,positions=[],cost_type='joint',joint_scaling=[1,1,1,1,1,1]):
        if cost_type!='joint':
            return self.tack_cost_function(positions,cost_type=cost_type,joint_scaling=joint_scaling)
        else:
            positions=np.array(positions)
            for n in range(6):
                positions[:,n]=self.absolute_angle_array(np.array(positions[:,n]))
            angles=positions

            for n in range(len(angles)-1):
                if math.isclose(angles[n+1,5],angles[n,5],abs_tol=10*-2):
                    delta_angles=np.array(angles[n+1])-np.array(angles[n])
                    delta_angles=delta_angles*np.array(joint_scaling)
                    cost+=np.linalg.norm(delta_angles)
                else:
                    TA=angles[n+1,5]-angles[n,5]
                    if TA>0 and (angles[n+1,5]-2*np.pi)>self.end_effector_joint_limits[0]:
                        angles[n+1:,5]+=-2*np.pi
                    if TA<0 and (angles[n+1,5]+2*np.pi)<self.end_effector_joint_limits[-1]:
                        angles[n+1:,5]+=2*np.pi
    
    def tack_cost_function(self,positions=[],cost_type='joint',joint_scaling=[1,1,1,1,1,1]):
        cost=0
               
        if cost_type=='joint':
                
            positions=np.array(positions)
            print(positions)

            for n in range(6):
                positions[:,n]=self.absolute_angle_array(np.array(positions[:,n]))
            
            angles=positions

            for n in range(len(angles)-1):
                delta_angles=np.array(angles[n+1])-np.array(angles[n])
                delta_angles=delta_angles*np.array(joint_scaling)
                cost+=np.linalg.norm(delta_angles)

            return cost

        elif cost_type=='cart':

            for n in range(len(positions)-1):
                delta_position=np.array(positions[n+1])-np.array(positions[n])
                cost+=np.linalg.norm(delta_angles)
            
            return cost
    def networkx_plot(self,G,weld=False):
        points={}
        labels={}
        node_list=[]
        node_colors=[]
        node_size=[]
        print(G)
        print(G.nodes())
        # input(self.node_dict.keys())
        # input(self.weld_dict.keys())
        if weld==True:
            for n in range(list(self.weld_dict.keys())[-1]+1):
                if n<list(self.node_dict.keys())[-1]+1:
                    key=list(self.node_dict.keys())[n]
                    # input(key)
                    points[key]=self.node_dict[key][0:2]
                    labels[key]='dw '+str(key)
                    node_list.append(key)
                    node_size.append(20)
                    node_colors.append("tab:red")
                else:
                    key=list(self.weld_dict.keys())[(n-1)-list(self.node_dict.keys())[-1]]
                    labels[key]=('v '+str(key))
                    points[key]=(np.array(self.weld_dict[key][0][0:2])+np.array(self.weld_dict[key][1][0:2]))/2
                    node_list.append(key)
                    node_size.append(10)
                    node_colors.append("tab:blue")         
        else:
            for n in range(len(self.node_dict.keys())):
                key=list(self.node_dict.keys())[n]
                labels[key]='tack '+str(key)
                print(key)
                points[key]=self.node_dict[key][0:2]
                node_list.append(key)
                node_colors.append("tab:red")
        
        print(points)
        print(points.keys())
        edge_list=[]
        edge_labels={}
        for u in G.edges:
            edge_list.append(tuple(u))
            print(u)
            edge_labels[u]='c_f'+str(u)+'='+str(np.round(G.edges[u]['weight'],decimals=2))

        if weld:
            label='Weld Graph with Joint Cost'
        else:
            label='Tack Graph with Joint Cost'

        nx.draw(G,pos=points,labels=labels,font_size=20,alpha=0.5,nodelist=node_list,node_color=node_colors,label=label)
        print(edge_labels)
        print(edge_list)
        nx.draw_networkx_edge_labels(G,pos=points,edge_labels=edge_labels,label_pos=0.25,alpha=0.5)
        # nx.draw_networkx_labels(G,pos=points,labels=labels,font_size=10,alpha=0.5)
        # input('drawn')
        plt.tight_layout()
        plt.axis("off")
        plt.show()
        input('drawb')

    def add_weld_nodes(self,G,weld_lines=None,cost_type='joint'):
        #* AHHH FUCK
        if type(weld_lines)==type(None):
            weld_lines=self.weld_lines

        # all_points=np.array(weld_lines)
        # all_points=all_points.reshape(all_points.shape[0]*all_points.shape[1],all_points.shape[2])
        
        # all_points=np.unique(all_points,axis=0)
        # print(all_points)


        if type(weld_lines)==type(np.array):
            weld_lines=weld_lines.tolist()
        #*Literally take the other graph add weld nodes between the required nodes and pass it to the old solver lol so easy
        #*The method to add the weld nodes to the networkx graph object

        # point_dic={tuple(self.desired_points[n]):n for n in range(len(self.desired_points))}
        #*This dictionary doesnt exist anymote in the object
        
        
        #*This dictonary is very similar to the node dictonary but reversed a given point is the key and returns the corresponding node in the graph
        # print('point_dic')
        # print(point_dic)
        # point_dic=point_dic
        # self.pointss=[]
        weld_dict={list(self.node_dict.keys())[-1]+(n+1):weld_lines[n] for n in range(len(weld_lines))}
        weld_ptd={tuple([tuple(list(weld_dict.values())[n][0]),tuple(list(weld_dict.values())[n][1])]):list(self.node_dict.keys())[n] for n in range(len(list(weld_dict.keys())))}
        weld_ptd.update({tuple([tuple(list(weld_dict.values())[n][1]),tuple(list(weld_dict.values())[n][0])]):list(self.node_dict.keys())[n] for n in range(len(list(weld_dict.keys())))})
        # weld_ptd={tuple(list(weld_dict.values())[n][0]):{tuple(list(weld_dict.values())[1]):list(self.node_dict.keys())[n]}}
        
        # input(weld_ptd)
        self.weld_ptd=weld_ptd
        # point_dic={}
        # for n in  
        point_dic={tuple(list(self.node_dict.values())[n]):list(self.node_dict.keys())[n] for n in range(len(list(self.node_dict.keys())))}
        
        # print('point_dic')
        # input(point_dic)
        # input(weld_dict)
        # G.add_nodes_from(weld_dict)
        for n in list(weld_dict.keys()):
            #*Loop through the given keys in the dictionary containing each set of points for each weld line
            G.add_node(n)
            #*Add each of these 'new' nodes to the graph
            # print('watch')
            # input(n)
            # input(weld_dict[n])
            # input(point_dic[tuple(weld_dict[n][0])])
            # input(point_dic[tuple(weld_dict[n][1])])
            
            G.add_edge(n,point_dic[tuple(weld_dict[n][0])])
            G.add_edge(n,point_dic[tuple(weld_dict[n][1])])
            #*Add limited connections for each of these nodes to force the path of the weld

            #* This next section I am basically generating the paths and inverse kinematics for each of these new edge paths.

            #*You need to get both normals......
            # print('weld dict of n:')
            # print(weld_dict[n][0])
            # print(weld_dict[n][1])
            
            # input(weld_dict[n])

            normal_a=self.get_desired_point_normal(tuple(weld_dict[n][0]))
            normal_c=self.get_desired_point_normal(tuple(weld_dict[n][1]))

            # normal_a=G.nodes[point_dic[weld_dict[n][0]]]['Normals']
            # normal_b=G.nodes[point_dic[weld_dict[n][1]]]['Normals']

            #*Get the normals that were already calculated in   the tacking solution, remeber these normals are basically calculated with an internal pyvista algorithm that just gives the arrow pointing away from the mesh at any point
            #TODO: This methodoligy mentioned is one of the top things to motify, edit and improve in relation to a better path planning algorithm
            
            #*So since we need to force the solution to travel through the weld nodes we take the weld line and bisect it with a node that the Traveling SalesMan has to go through
            normal_b=(normal_a+normal_c)/2
            normal_b=np.array(normal_b)/np.linalg.norm(normal_b)
            #*This middle node then has the orientation of the average between these two start and finish normal
            #TODO: Consider chaning this
            #*CHECK
            point_a=self.get_adjusted_point(point_key=tuple(weld_dict[n][0]),normal=normal_a)
            point_c=self.get_adjusted_point(point_key=tuple(weld_dict[n][1]),normal=normal_c)
            point_b=(np.array(point_a)+np.array(point_c))/2
            angles=[]

            end_ee_ik_a=self.get_point_normal_angles(point_a,normal_a)
            end_ee_ik_b=self.get_point_normal_angles(point_b,normal_b)
            end_ee_ik_c=self.get_point_normal_angles(point_c,normal_c)
            #*CHECK THIS:
            # input(joint_values)
            if cost_type=='joint':
                cost_array=[end_ee_ik_a,end_ee_ik_b,end_ee_ik_c]
            else:
                cost_array=[point_a,point_b,point_c]

            # G.add_edge(,)
            # G.add_edge(,)
            
            nx.set_edge_attributes(G,{(n,point_dic[tuple(weld_dict[n][0])]):self.tack_cost_function(positions=cost_array[0:2],cost_type=cost_type)},'weight')
            nx.set_edge_attributes(G,{(n,point_dic[tuple(weld_dict[n][1])]):self.tack_cost_function(positions=cost_array[1:],cost_type=cost_type)},'weight')

        return G,weld_dict,point_dic
        
    def solve_weld_tour(self,G,weld_dict,point_dic):
        #*THis is a custom MIP (mixed integer program) to solve the welding order based in python-mip, it is very similar to the way the traditional tranveling salesman problem is programmed:  https://docs.python-mip.com/en/latest/examples.html#the-traveling-salesman-problem
        #* It is easier to understanf this in slides, I will make some
        from itertools import product
        from sys import stdout as out
        from mip import Model, xsum, minimize, BINARY, INTEGER, CONTINUOUS, constants
        
        model= Model()
        V=set(range(len(weld_dict.keys())))
        n=len(weld_dict.keys())
        nodes=list(weld_dict.keys())
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

        all_shortest_paths=[]
        weld_graph=nx.complete_graph(weld_dict.keys())
        
        nx.set_edge_attributes(weld_graph,None,"Shortest_Paths")
        nx.set_edge_attributes(weld_graph,None,"Shortest_Path_Weights")

        # size=len(weld_graph.nodes)+1
        weight_Ahsymetric=np.zeros((len(weld_graph.nodes),len(weld_graph.nodes),4))
        paths_Ahsymetric=np.empty((len(weld_graph.nodes),len(weld_graph.nodes)),dtype=list)
        # default_shortest=np.zeros((len(weld_graph.nodes),len(self.weld_graph.nodes)))
        n_fi=list(self.node_dict.keys())[-1]+1
        # input(n_fi)
        # input(weld_dict.keys())
        # input(self.node_dict.keys())
        # print('edges')
        # input(G.edges)
        # input(weld_graph.edges)
        for (u,v) in weld_graph.edges:
            #get the connecting points:
            u_connect=weld_dict[u]
            v_connect=weld_dict[v]
            # g1 ,weight='weight'
            u_toto_shortest=nx.shortest_path(G,source=point_dic[tuple(u_connect[0])],target=point_dic[tuple(v_connect[0])])
            u_tofrom_shortest=nx.shortest_path(G,source=point_dic[tuple(u_connect[0])],target=point_dic[tuple(v_connect[1])])
            u_fromto_shortest=nx.shortest_path(G,source=point_dic[tuple(u_connect[1])],target=point_dic[tuple(v_connect[0])])
            u_fromfrom_shortest=nx.shortest_path(G,source=point_dic[tuple(u_connect[1])],target=point_dic[tuple(v_connect[1])])
            #g2
            v_toto_shortest=nx.shortest_path(G,source=point_dic[tuple(v_connect[0])],target=point_dic[tuple(u_connect[0])])
            v_tofrom_shortest=nx.shortest_path(G,source=point_dic[tuple(v_connect[0])],target=point_dic[tuple(u_connect[1])])
            v_fromto_shortest=nx.shortest_path(G,source=point_dic[tuple(v_connect[1])],target=point_dic[tuple(u_connect[0])])
            v_fromfrom_shortest=nx.shortest_path(G,source=point_dic[tuple(v_connect[1])],target=point_dic[tuple(u_connect[1])])

            Paths=[u_toto_shortest,u_tofrom_shortest,u_fromto_shortest,u_fromfrom_shortest,v_toto_shortest,v_tofrom_shortest,v_fromto_shortest,v_fromfrom_shortest]
            
            # input(Paths)
            # input(paths_Ahsymetric)
            # print(u-n_fi)
            # print(v-n_fi)
            # input('cech this')

            paths_Ahsymetric[u-n_fi][v-n_fi]=deepcopy(Paths[0:4])
            paths_Ahsymetric[v-n_fi][u-n_fi]=deepcopy(Paths[4:])
            # print(Paths)
            for n in range(len(Paths)):
                # print(n)
                # print(Paths[n])
                Path_weight_def=0
                for i in range(len(Paths[n][0:-1])):
                    # print(i)
                    Path_weight_def+=G.edges[Paths[n][i],Paths[n][i+1]]['weight']
                if n > 3:
                    # print(G.edges[Paths[n][-1],u])
                    # print(G.edges[v,Paths[n][0]])
                    Path_weight_def+=G.edges[Paths[n][-1],u]['weight']+G.edges[v,Paths[n][0]]['weight']
                    weight_Ahsymetric[v-n_fi][u-n_fi][n-4]=deepcopy(Path_weight_def)
                else:
                    # print(G.edges[Paths[n][-1],v])
                    # print(G.edges[u,Paths[n][0]])
                    Path_weight_def+=G.edges[Paths[n][-1],v]['weight']+G.edges[u,Paths[n][0]]['weight']
                    weight_Ahsymetric[u-n_fi][v-n_fi][n]=deepcopy(Path_weight_def)
                # input('Continue?:')
            
        # default_relations=np.zeros((len(self.weld_graph.nodes),len(self.weld_graph.nodes),len(self.weld_graph.nodes),4))

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
            print('hhmmmm')
            print(nodes)

            # out.write('route with total distance %g found: %s'
            #         % (model.objective_value, nodes[0]))
            print('route with total distance: '+str(model.objective_value))
            solution.append(nodes[0])
            solution_bare.append(nodes[0])
            nc = (0,0)
            while True:
                print('V')
                print(V)
                nc = [(i,j) for i in V for j in range(4) if x[nc[0]][i][j].x >= 0.99][0]
                print(nc)
                print('nc')
                # out.write(' -> %s' % nodes[nc[0]])
                next_nodes=paths_Ahsymetric[solution_bare[-1]-n_fi][nc[0]][nc[-1]]
                print('next nodes:')
                print(next_nodes)
                print('solution before')
                print(solution)
                solution+=list((deepcopy(next_nodes)))
                print(solution)
                # solution.append(nodes[nc[0]])
                solution_bare.append(nodes[nc[0]])
                solution.append(nodes[nc[0]])
                if nc[0] == 0:
                    break
            
            out.write('\n')
            
            self.tour=solution[0:-1]

            self.tour_stripped=solution_bare[0:-1]
            print('TOURS')
            print(self.tour_stripped)
            print(self.tour)
            self.weld_dict=weld_dict
            self.point_dic=point_dic
            # input('good?')
            
            # for n in range(len(self.tour)):
            #     if self.tour[n] in list(weld_dict.keys()):
            #         tour_stripped.append(self.tour[n])

            
            # self.weld_tour_stripped=tour_stripped
            # print('huh')
            # input(self.weld_tour_stripped)
            # # np.save('tour_weld_current',self.tour)
 
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
        print('tour')
        print(tour)
    
    #*these reorder tours for use in the execution:

    def reorder_tour_tack(self):
        #*This just reorders the tour to start with the closest point
        #*get these values right.....

        print(list(self.node_dict.values()))
        current_nodes=[self.get_adjusted_point(point_key=tuple(n)) for n in list(self.node_dict.values())]
        
        print(current_nodes)

        # normal=self.get_desired_point_normal(point)
        # adjusted_points.append(self.get_adjusted_point(point_key=tuple(point),normal=normal))

        # end_ee_ik=self.get_point_normal_angles(adjusted_points[-1],normal)
        # angles.append(deepcopy(end_ee_ik))
        # dnodes={n:tuple(all_points[n]) for n in range(len(all_points[:]))}
        
        # self.node_dict=deepcopy(dnodes)
        current_node_tree=scipy.spatial.KDTree(current_nodes)

        current_position=self.move_group.get_current_pose().pose

        current_position=[current_position.position.x,current_position.position.y,current_position.position.z]
        
        cpi=current_node_tree.query(current_position)[1]
        print('closested_point_index')
        print(cpi)

        # np.save('cpi_weld_current',cpi)
        # if self.tacking==True:
        #     cpi=np.load('cpi_weld_current.npy')

        for n in range(len(self.tour)):
            if self.tour[n]==cpi:
                self.tour=list(self.tour[n:])+list(self.tour[0:n])
                print('Path re order')
                print(self.tour)
                break
        
        # self.tour.append(self.tour[0])
        # self.tour_edges=list(zip(self.tour,self.tour[1:]))
        # input(self.tour_edges)

    def reorder_tour_weld(self):
        #*This just reorders the tour to start with the closest point
        #*get these values right.....

        current_weld,remainder,closest_weld_line_index=self.next_weld_greedy_on_line(self.live_welds,point=None,remove_point=False)
        
        cpi=closest_weld_line_index
        weldp1=current_weld[0]
        weldp2=current_weld[1]

        n1=self.point_dic[tuple(weldp1)]
        n2=self.point_dic[tuple(weldp2)]
        print(tuple([tuple(weldp1),tuple(weldp2)]))
        weld_number=list(self.weld_dict.keys())[self.weld_ptd[tuple([tuple(weldp1),tuple(weldp2)])]]
        print('weld numba')
        print(weld_number)
        print(self.tour_stripped)
        cpi=np.where(np.array(self.tour_stripped)==weld_number)[0][0]

        print('cpi')
        print(cpi)
        cpi=int(cpi)
        print('remainder:')
        print(remainder)
        if remainder!=0:
            if cpi==0:
                print(self.tour_stripped[cpi::-1])
                self.tour_stripped=self.tour_stripped[cpi::-1]+list(self.tour_stripped[-1:cpi:-1])
            elif cpi==len(self.tour_stripped)-1:
                self.tour_stripped=list(self.tour_stripped[cpi::-1])
            else:
                self.tour_stripped=list(list(self.tour_stripped[cpi::-1])+list(self.tour_stripped[-1:cpi:-1]))
            print('reordered stripped tour')
            print(self.tour_stripped)       
            
            if cpi==len(self.tour_stripped)-1:
                self.tour=[self.tour[-1]]+list(self.tour[-2::-1])
            elif cpi==0:
                self.tour=self.tour[1::-1]+list(self.tour[-1:cpi:-1])
            else:
                id=np.where(np.array(self.tour)==self.tour_stripped[0])[0][0]
                print(id)
                print(self.tour)
                self.tour=self.tour[(id+1)::-1]+self.tour[-1:(id+1):-1]
            print('reordred total tour1:')
            print(self.tour)
        else:
            if cpi==0:
                self.tour_stripped=list(self.tour_stripped[cpi:])
            if cpi==len(self.tour_stripped)-1:
                self.tour_stripped=list(self.tour_stripped[0:cpi+1])
            else:
                self.tour_stripped=list(self.tour_stripped[cpi:])+list(self.tour_stripped[0:cpi])
            print('reordered stripped tour')
            print(self.tour_stripped)
            print(cpi)

            if cpi==0:
                self.tour=[self.tour[-1]]+self.tour[0:]
            else:
                id=np.where(np.array(self.tour)==self.tour_stripped[0])[0][0]
                # if id-1==0:
                #     self.tour=[self.tour[(id-1)]]+self.tour[id:]+self.tour[:(id-1)]
                # else:
                self.tour=[self.tour[(id-1)]]+self.tour[id:]+self.tour[:(id-1)]
            print('reordred total tour2:')
            print(self.tour)

            
            # print('Path re order')
            # print(self.tour)
            
        # self.tour.append(self.tour[0])
        # self.tour_edges=list(zip(self.tour,self.tour[1:]))
        # input(self.tour_edges)

    #*IMPLIMENT SOLVERS ABOVE:

    #*MODIFY THE CODE BELLOW

    def time_path_parameratization(self,joint_angles,scaling_metric,method='joint_distance',b=10,c=1,cost=None):
        def time_path_distance(b,c,Joint_Angles,cost,desired_velocity=self.transport_velocity):
            scurve=s_curve(b=b,c=c)
            Path_Distance_scurve=scurve.s(1)-scurve.s(0)
            # ratio_scurvemax_to_scurvedistance=scurve.ds(0.5)/(Path_Distance_scurve)

            desired_lin_vel=scaling_metric
            desired_time_span=cost/desired_lin_vel
            time, joint_angles=compute_joint_n_time_arrays(scurve,Joint_Angles,desired_time_span,Path_Distance_scurve)
            # print('joint angles startr and finish')
            # print(joint_angles[0])
            # print(joint_angles[-1])
            # input('good?')
            
            
            # print('plotting')
            self.plot_single_set(np.array(joint_angles),np.array(time))         
            
            return time, joint_angles
        
        def compute_joint_n_time_arrays(scurve,Joint_Angles,desired_time_span,Path_Distance_scurve):
            joint_angles=[]
            time=[]
            x=np.linspace(0,1,len(Joint_Angles)*4)
            start_t=deepcopy(scurve.s(x[0]))
            
            for n in range(len(x)):
                t=x[n]
                ratio=((scurve.s(t)-start_t)/Path_Distance_scurve)
                index_avg=(ratio*(len(Joint_Angles)-1))
                indexf=int(np.ceil(index_avg))
                indexb=int(np.floor(index_avg))                
                JAF=Joint_Angles[indexf]
                
                JAB=Joint_Angles[indexb]
                JAD=np.array(JAF)-np.array(JAB)
                
                if indexf!=indexb:
                    WAJA=JAB+JAD*abs((index_avg-indexb)/(indexf-indexb))
                elif indexf==indexb:
                    WAJA=JAF
                
                joint_angles.append(deepcopy(WAJA))
                time.append(t*desired_time_span)
            
            # self.plot_single_set(np.array(joint_angles),np.array(time))

            return time, joint_angles
        
        def time_joint_distance(b,c,Joint_Angles,desired_time_span=None,desired_angular_velocity=None):
            #*This times the path based on an average joint velocity and joint distance
            scurve=s_curve(b=b,c=c)
            Path_Distance_scurve=scurve.s(1)-scurve.s(0)
            # ratio_scurvemax_to_scurvedistance=scurve.ds(0.5)/(Path_Distance_scurve)
            
            if desired_time_span!=None and desired_angular_velocity==None:
                angle_change=np.array(Joint_Angles[-1])-np.array(Joint_Angles[0])
                # scaled_change=np.dot(self.reorientation_cost_vector,angle_change)
                desired_avg_ang_vel=(np.linalg.norm(angle_change))/desired_time_span
            elif desired_time_span==None and desired_angular_velocity!=None:
                desired_avg_ang_vel=desired_angular_velocity
                angle_change=np.array(Joint_Angles[-1])-np.array(Joint_Angles[0])
                scaled_change=np.dot(self.reorientation_cost_vector,angle_change)
                desired_time_span=(np.linalg.norm(scaled_change))/desired_avg_ang_vel
            else:
                print('No conditions passed for the timing, exiting')
                exit()

            time, joint_angles=compute_joint_n_time_arrays(scurve,Joint_Angles,desired_time_span,Path_Distance_scurve)
            # print('plotting')
            self.plot_single_set(np.array(joint_angles),np.array(time))

            return time, joint_angles

        if method=='joint_distance':
            time,joint_angles=time_joint_distance(b,c,joint_angles,desired_time_span=None,desired_angular_velocity=scaling_metric)
        elif method=='time':
            time,joint_angles=time_joint_distance(b,c,joint_angles,desired_time_span=scaling_metric,desired_angular_velocity=None)
        elif method=='path_distance':
            time,joint_angles=time_path_distance(b,c,joint_angles,desired_velocity=scaling_metric,cost=cost)
        
        return time,joint_angles

    def plot_single_set(self,Angles,time):

        if type(self.fig1)==type(None):
            self.fig1, self.axs1 = plt.subplots(3,2)
            
        #     # self.fig1.ion()
        
        plt.rcParams.update({'font.size': 12})
        # fig, axs = plt.subplots(3,2)
        fig=self.fig1
        axs=self.axs1
        fig.suptitle('Joint Angles Across Time')
        fig.tight_layout()

        for n in axs:
            for k in n:
                k.clear()

        for n in axs:
            for k in n:
                k.set_ylabel('\u03B8(degrees)')
                k.set_xlabel('time(seconds)')
        # fig.clf()
        
        # input(dir(axs[0]))
        axs[0,0].scatter(time,Angles[:,0]*180/np.pi)
        axs[0,0].set_title('Q0')
       
        

        axs[1,0].scatter(time,Angles[:,1]*180/np.pi)
        axs[1,0].set_title('Q1')
        # axs[1].ylabel('\theta(degrees)')
        # axs[1].xlabel('time(seconds)')
        

        axs[2,0].scatter(time,Angles[:,2]*180/np.pi)
        axs[2,0].set_title('Q2')
        # axs[2].ylabel('\theta(degrees)')
        # axs[2].xlabel('time(seconds)')
        
        axs[0,1].scatter(time,Angles[:,3]*180/np.pi)
        axs[0,1].set_title('Q3')
        # axs[3].ylabel('\theta(degrees)')
        # axs[3].xlabel('time(seconds)')
        
        axs[1,1].scatter(time,Angles[:,4]*180/np.pi)
        axs[1,1].set_title('Q4')
        # axs[4].ylabel('\theta(degrees)')
        # axs[4].xlabel('time(seconds)')
        

        axs[2,1].scatter(time,Angles[:,5]*180/np.pi)
        axs[2,1].set_title('Q5')
        # axs[5].ylabel('\theta(degrees)')
        # axs[5].xlabel('time(seconds)')
        plt.pause(0.01)
        # plt.show()
        
    def get_VAJ(): 
        pass         
        #             desired_joint_velocities=[]
        #             for n in range(len(Joint_Angles)):
        #                 if n==0 or n==len(Joint_Angles)-1:
        #                     desired_joint_velocities.append([0,0,0,0,0,0])
        #                 else:
        #                     vrf=(np.array(Joint_Angles[n+1])-np.array(Joint_Angles[n]))/(time[n+1]-time[n])
        #                     vab=(np.array(Joint_Angles[n])-np.array(Joint_Angles[n-1]))/(time[n]-time[n-1])
        #                     vaa=(vrf+vab)/2
        #                     desired_joint_velocities.append(deepcopy(vaa))

        #             desired_joint_accelerations=[]

        #             for n in range(len(Joint_Angles)):
        #                 if n==0 or n==len(Joint_Angles)-1:
        #                     desired_joint_accelerations.append([0,0,0,0,0,0])
        #                 else:
        #                     arf=(np.array(desired_joint_velocities[n+1])-np.array(desired_joint_velocities[n]))/(time[n+1]-time[n])
        #                     aab=(np.array(desired_joint_velocities[n])-np.array(desired_joint_velocities[n-1]))/(time[n]-time[n-1])
        #                     aaa=(arf+aab)/2
        #                     desired_joint_accelerations.append(deepcopy(aaa))

        #             self.print_array(time=np.array(time),Angles=np.array(Joint_Angles),Velocities=np.array(desired_joint_velocities),Accelerations=np.array(desired_joint_accelerations))
        #             return desired_joint_velocities,desired_joint_accelerations
        
    def fit_scipy_spline_to_joint_angles(self,Joint_Angles,time,Smoothness_Coefficient=None,retime=False,cost=None,cost_type=None):
        def generate_splines(time,Joint_Angles):
            Joint_Angles=np.array(Joint_Angles)
            q0=Joint_Angles[:,0]
            q1=Joint_Angles[:,1]
            q2=Joint_Angles[:,2]
            q3=Joint_Angles[:,3]
            q4=Joint_Angles[:,4]
            q5=Joint_Angles[:,5]
            
            # weights=[1]*len(time)
            # weights[0]=1000
            # weights[-1]=1000k
            # weights[1]=500
            # weights[-2]=500
            # weights[2]=250
            # weights[-3]=250
            weights=np.linspace(10000,100,int(np.floor(len(Joint_Angles)/2))).tolist()+[1]*(len(Joint_Angles)%2)+np.linspace(100,10000,int(np.floor(len(Joint_Angles)/2))).tolist()
            weights[0]=1000*1000000
            weights[-1]=1000*1000000
            weights[1]=100*1000
            weights[-2]=100*1000
            

            splq0=scipy.interpolate.make_smoothing_spline(time,q0, w=weights, lam=Smoothness_Coefficient)
            splq1=scipy.interpolate.make_smoothing_spline(time,q1, w=weights, lam=Smoothness_Coefficient)
            splq2=scipy.interpolate.make_smoothing_spline(time,q2, w=weights, lam=Smoothness_Coefficient)
            splq3=scipy.interpolate.make_smoothing_spline(time,q3, w=weights, lam=Smoothness_Coefficient)
            splq4=scipy.interpolate.make_smoothing_spline(time,q4, w=weights, lam=Smoothness_Coefficient)
            splq5=scipy.interpolate.make_smoothing_spline(time,q5, w=weights, lam=Smoothness_Coefficient)
            
            return [splq0,splq1,splq2,splq3,splq4,splq5]

        def interpolate_joint_angles(splines,resolution):
            splinesdt=[n.derivative() for n in splines]
            splinesddt=[n.derivative(2) for n in splines]
            t_interpolate=np.linspace(time[0],time[-1],resolution)

            # if retime==True:
            #     splines,splinesdt,splinesddt=retime_trajectory(splines,splinesdt,splinesddt,t_interpolate,resolution)
            
            Joint_Angles=[[n(to) for n in splines]for to in t_interpolate]

            Joint_Velocities=[[n(to) for n in splinesdt]for to in t_interpolate]
            
            Joint_Accelerations=[[n(to) for n in splinesddt]for to in t_interpolate]
            
            return Joint_Angles,Joint_Velocities,Joint_Accelerations,t_interpolate
        
        def retime_trajectory(splines,splinesdt,splinesddt,t_interpolate,resolution):
            scurve=s_curve(b=10,c=1)
            average_velocity=0
            
            
            pass
        
        if type(Smoothness_Coefficient)==type(None):
            Smoothness_Coefficient=self.smoothness_coefficient_reorientatie

        splines=generate_splines(time,Joint_Angles)
        if type(cost)==type(None):
            resolution=self.min_final_resolution
        else:
            if cost_type=='path_distance':
                resolution=int(np.ceil(self.final_resolution_per_path_length*cost))
            elif cost_type=='joint_distance':
                resolution=int(np.ceil(self.final_resolution_per_joint_length*cost))
            else:
                resolution=self.min_final_resolution

        if resolution<self.min_final_resolution:
            resolution=self.min_final_resolution

        Joint_Angles,Joint_Velocities,Joint_Accelerations,time=interpolate_joint_angles(splines,resolution)
        return time,Joint_Angles,Joint_Velocities,Joint_Accelerations
        
    def fit_polynomials_to_joint_angles(self,Joint_Angles,time,cost=None,cost_type=None):
        #*polyfit
        num_of_constraints=len(Joint_Angles)+4
        poly_coef=[]
        desired_joint_velocities=[]
        for n in range(len(Joint_Angles)):
            if n==0 or n==len(Joint_Angles)-1:
                desired_joint_velocities.append([0,0,0,0,0,0])
            else:
                vrf=(np.array(Joint_Angles[n+1])-np.array(Joint_Angles[n]))/(time[n+1]-time[n])
                vab=(np.array(Joint_Angles[n])-np.array(Joint_Angles[n-1]))/(time[n]-time[n-1])
                vaa=(vrf+vab)/2
                desired_joint_velocities.append(deepcopy(vaa))

                desired_joint_accelerations=[]

        for n in range(len(Joint_Angles)):
            if n==0 or n==len(Joint_Angles)-1:
                desired_joint_accelerations.append([0,0,0,0,0,0])
            else:
                arf=(np.array(desired_joint_velocities[n+1])-np.array(desired_joint_velocities[n]))/(time[n+1]-time[n])
                aab=(np.array(desired_joint_velocities[n])-np.array(desired_joint_velocities[n-1]))/(time[n]-time[n-1])
                aaa=(arf+aab)/2
                desired_joint_accelerations.append(deepcopy(aaa))
                # self.print_array(time=np.array(time),Angles=np.array(Joint_Angles),Velocities=np.array(desired_joint_velocities),Accelerations=np.array(desired_joint_accelerations))
                # return desired_joint_velocities,desired_joint_accelerations
        
        for n in range(0,6):
            constraints=[]
            A=[]
            for k in range(len(time)):
                if k==0 or k==len(time)-1:
                    # if True:
                    constraints.append(Joint_Angles[k][n])
                    constraints.append(0)
                    constraints.append(0)
                    for l in range(0,3):
                        row=[]
                        # scales=[]
                        for b in range(num_of_constraints):
                            if b>=l:
                                scale=math.factorial(b)/math.factorial((b-l))
                                row.append((time[k]**(b-l))*scale)
                            else:
                                row.append(0)
                        A.append(deepcopy(row))
                else:
                    constraints.append(Joint_Angles[k][n])
                    # constraints.append(desired_joint_velocities[k][n])
                    for l in range(0,1):
                        row=[]
                        for b in range(num_of_constraints):
                            if b>=l:
                                scale=math.factorial(b)/math.factorial((b-l))
                                # scales.append(scale)
                                row.append((time[k]**(b-l))*scale)
                            else:
                                row.append(0)
                        A.append(deepcopy(row))
            A=np.array(A)
            # print('A')
            # print(np.size(A))
            # print(np.shape(A))
            constraints=np.array(constraints)
            # print('here')
            # print(A)
            # print('space')
            # print(constraints)
            coef=np.linalg.solve(A,constraints)
            # print(coef)
            # input('good?')
            poly_coef.append(coef)
        # print(np.array(poly_coef))
        # print(len(poly_coef))
        # input('good?')

        if type(cost)==type(None):
            resolution=self.min_final_resolution
        else:
            if cost_type=='path_distance':
                resolution=int(np.ceil(self.final_resolution_per_path_length*cost))
            elif cost_type=='joint_distance':
                resolution=int(np.ceil(self.final_resolution_per_joint_length*cost))
            else:
                resolution=self.min_final_resolution

        if resolution<self.min_final_resolution:
            resolution=self.min_final_resolution

        new_time=np.linspace(time[0],time[-1],resolution)
        new_angles=[]
        new_velocities=[]
        new_accelerations=[]

        scalep=[math.factorial(b)/math.factorial((b)) for b in range(0,num_of_constraints)]
        scalev=[math.factorial(b)/math.factorial((b-1)) for b in range(1,num_of_constraints)]
        scalea=[math.factorial(b)/math.factorial((b-2)) for b in range(2,num_of_constraints)]
                
        for n in range(len(new_time)):
            t=new_time[n]                    
            time_p=[t**k for k in range(num_of_constraints)]
            time_v=[t**d for d in range(num_of_constraints-1)]
            time_a=[t**r for r in range(num_of_constraints-2)]
            jp=[np.dot(time_p,(np.array(poly_coef[l][:])*np.array(scalep))) for l in range(0,6)]
            jv=[np.dot(time_v,(np.array(poly_coef[m][1:])*np.array(scalev))) for m in range(0,6)]
            ja=[np.dot(time_a,(np.array(poly_coef[p][2:])*np.array(scalea))) for p in range(0,6)]
            new_angles.append(jp)
            new_velocities.append(jv)
            new_accelerations.append(ja)
        
        # self.print_array(time=np.array(new_time),Angles=np.array(new_angles),Velocities=np.array(new_velocities),Accelerations=np.array(new_accelerations),message='reorientation_joint_angles')
        # exit()
        return new_time,new_angles,new_velocities,new_accelerations

    def send_joint_trajectory_to_move_it(self,path):
        path.joint_trajectory.header.stamp.secs=rospy.Time.now().secs
        path.joint_trajectory.header.stamp.nsecs=rospy.Time.now().nsecs
        bool=self.move_group.execute(path,wait=True)
        rospy.sleep(0.1)
        self.move_group.stop()
        print(bool)

    def create_joint_traj_data_type(self,time,joint_angles,joint_velocities,joint_accelerations):
        Velocities=np.array(joint_velocities)
        Accelerations=np.array(joint_accelerations)
            
        rtj=trajectory_msgs.msg.JointTrajectory()
        header=std_msgs.msg.Header()
        header.frame_id="world"
        header.seq=int(1)

        rtj.joint_names=JOINT_NAMES
        points=[]

        for n in range(len(time)):
            Joint_Point=trajectory_msgs.msg.JointTrajectoryPoint()
            Joint_Point.positions=joint_angles[n]
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
        return path

    def plan_path_constant_z_axis(self,p1,p2,n2,metric='joint_distance',points_per=3.404852841045446,ee_angle_corrector=None):
        
        if ee_angle_corrector==None:
            ee_angle_corrector=self.ee_angle_corrector
        
        # end_ee_ik=np.inf

        end_ee_ik=self.get_point_normal_angles(point=tuple(p2),normal=tuple(n2))
        
        
        #! lol hopefully this shows a performance improvement I know I probably just broke everything

        # if tuple(p2) in list(self.point_normal_joint_angle_dict.keys()):
        #     if tuple(n2) in list(self.point_normal_joint_angle_dict[tuple(p2)].keys()) and type(self.point_normal_joint_angle_dict[tuple(p2)][tuple(n2)])!=type(None):
        #         end_ee_ik=self.point_normal_joint_angle_dict[tuple(p2)][tuple(n2)]
        #     else:
        #         while end_ee_ik==np.inf:
        #             end_ee_ik=self.get_IK(point=p2,orientation_vector=[-n2[0],-n2[1],0])
        #         self.point_normal_joint_angle_dict[tuple(p2)].update({tuple(n2):list(end_ee_ik)})
        # else:
        #     while end_ee_ik==np.inf:
        #             end_ee_ik=self.get_IK(point=p2,orientation_vector=[-n2[0],-n2[1],0])
        #     self.point_normal_joint_angle_dict[tuple(p2)]={tuple(n2):list(end_ee_ik)}

        
        end_ee_angle=end_ee_ik[-1]
        end_ee_angle=ee_angle_corrector(end_ee_angle)
        end_ee_ik[-1]=end_ee_angle
        
        current_JA=self.move_group.get_current_joint_values()
        
        if metric=='joint_distance':
            distance=np.linalg.norm(np.dot(self.reorientation_cost_vector,np.array(end_ee_ik)-np.array(current_JA)))
            pointsperjointanglechange=points_per
            num_points=int(np.floor(distance*pointsperjointanglechange))+1
        elif metric=='path_distance':
            distance=np.linalg.norm(np.array(p2)-np.array(p1))
            if points_per==3.404852841045446:
                point_per_pathdistance=50
            else:
                points_per_pathdistance=points_per
                num_points=distance*point_per_pathdistance
        else:
            print('INVALID METRIC PROVIDED DEFAULTING TO JOINT DISTANCE')
            distance=np.linalg.norm(np.array(end_ee_ik)-np.array(current_JA))
            pointsperjointanglechange=3.404852841045446
            num_points=int(np.ceil(distance*pointsperjointanglechange))

        #*Create the control points array
            
        control_points=list(euclidean_linspace(p1,p2,num_points))        
        control_points=np.array(control_points)
        joint_array=[]

        joint_array.append(deepcopy(current_JA))
        # joint_array.append(deepcopy(current_JA))
        # joint_array.append(deepcopy(current_JA))
        # joint_array.append(deepcopy(current_JA))

        #*Create end effector joint array
        ee_angles=np.linspace(current_JA[-1],end_ee_angle,num_points)
        # print('ee angles')
        # print(ee_angles)
        
        print('enter1')
        for n in range(1,len(control_points)):
            angles=np.inf
            while angles==np.inf or type(angles)==type(None):
                angles=self.get_IK(point=control_points[n],orientation_vector=[-n2[0],-n2[1],0],seed_state=joint_array[-1])
            angles[-1]=ee_angles[n]
            if tuple(angles)!=tuple(joint_array[-1]):         
                joint_array.append(deepcopy(angles))
                # if n==len(control_points)-1:
                #     joint_array.append(deepcopy(angles))
                #     joint_array.append(deepcopy(angles))
                #     joint_array.append(deepcopy(angles))
        print('exit1')
        # joint_array.append(deepcopy(angles))
        return joint_array,distance

    #*MODIFY THE CODE ABOVE

    def ee_angle_corrector_to_current_weld(self,angle):
        pass

    def ee_angle_corrector_constant(self,angle):
        current_JA=self.move_group.get_current_joint_values()
        return current_JA[-1]

    def closest_to_current_ee_angle_corrector(self,angle):
        current_JA=self.move_group.get_current_joint_values()
        if math.isclose(angle,current_JA[-1],abs_tol=10**-1):
            return angle
        
        if abs((angle-2*np.pi)-current_JA[-1])<abs((angle+2*np.pi)-current_JA[-1]) and abs((angle-2*np.pi)-current_JA[-1])<abs((angle)-current_JA[-1]):
            angle=angle-2*np.pi
            return angle
        
        if abs((angle-2*np.pi)-current_JA[-1])>abs((angle+2*np.pi)-current_JA[-1]) and abs((angle+2*np.pi)-current_JA[-1])<abs((angle)-current_JA[-1]):
            angle=angle+2*np.pi
            return angle
        
        return angle

    def ee_angle_corrector(self,angle):        
        if self.current_edge==None:
            print('standard 1')
            if angle>self.aeejl and abs(angle-self.aeejl)>abs((angle-self.aeejl)-(2*np.pi)) and (angle-(2*np.pi))>self.end_effector_joint_limits[0] and (angle-(2*np.pi))<self.end_effector_joint_limits[-1]:
                angle+=(-2*np.pi)
            elif angle<self.aeejl and abs(self.aeejl-angle)>abs((self.aeejl-angle)+(2*np.pi)) and (angle+(2*np.pi))>self.end_effector_joint_limits[0] and (angle+(2*np.pi))<self.end_effector_joint_limits[-1]:
                angle+=(2*np.pi)
            return deepcopy(angle)
        else:
            TA=self.current_edge.total_angle_change()
            if TA>0 and (angle-2*np.pi)>self.end_effector_joint_limits[0]:
                angle+=-2*np.pi
            
            if TA<0 and (angle+2*np.pi)<self.end_effector_joint_limits[-1]:
                angle+=2*np.pi
            
            if angle>self.end_effector_joint_limits[-1] or angle<self.end_effector_joint_limits[0]:
                print('WARNING WRONG START ANGLE')
                print(angle)
                exit()
            
            return deepcopy(angle)

    def drop_down_to_point_moveit_planned(self,point,normals):
        waypoints=self.cart_traj_same_pose_wp(point=point,normals=normals)
        bool=False
        while bool==False:
            try:
                (plan, fraction) = self.move_group.compute_cartesian_path(
                waypoints, 0.005, 0.0  # waypoints to follow  # eef_step
                )  # jump_threshold
                # if fraction<1.0:
                #     raise Exception('WARNING Fraction:'+str(fraction))
                
                self.iterative_constant_joint_angle_check(plan=plan,raise_exepction=True)
                # change_limits=[np.pi/8,np.pi/4,np.pi/4,np.pi/2,0.1,abs(self.end_effector_joint_limits[0]-self.end_effector_joint_limits[-1])]
                change_limits=[0.2]*6
                self.iterative_joint_angle_check(plan=plan,change_limits=change_limits,start_joint_angles=list(self.move_group.get_current_joint_values()),raise_exepction=True)
                # input('good?')
                print('attempting drop down')
                bool=self.move_group.execute(plan,wait=True)
            except:
                print('failure retrying')

    def lift_up_constant_ee_angle_corrector(self,point,normal):
        #Lets assume the point is the current point
        #go luck up the point and shit.....
        
        
        #!point must be in the desired part transform
        p2=self.get_safety_points(tuple(point),normal=tuple(normal))
        # p2=point
        # p2[0]+=normal[0]*self.control_distance
        # p2[1]+=normal[1]*self.control_distance
        # p2[2]+=normal[2]*self.control_distance

        
        
        # if not(tuple(p2) in list(self.point_normal_joint_angle_dict.keys())):
        #     self.point_normal_joint_angle_dict[tuple(p2)]={tuple(normal):None}
        # Joint_Angles=self.get_point_normal_angles(tuple(point),tuple(normal))
        

        #*VALIDATE IN HERE\|/
        # print(point)
        # print(p2)
        # input('are those different')

        joint_array,distance=self.plan_path_constant_z_axis(p1=point,p2=p2,n2=normal,ee_angle_corrector=self.ee_angle_corrector_constant)
        
        # input(joint_array)
        #*Theres a better conditional then this I know it:
        if len(joint_array)>1:

            time,joint_array=self.time_path_parameratization(joint_angles=joint_array,scaling_metric=self.reorientation_angular_rate,method='joint_distance',b=self.transportb,c=1,cost=None)

            time,joint_angles,joint_velocities,joint_accelerations=self.fit_scipy_spline_to_joint_angles(joint_array,time,cost=distance,cost_type='joint_distance')

            plan=self.create_joint_traj_data_type(time,joint_angles,joint_velocities,joint_accelerations)

            if self.display_trajectories==True:
                display_trajectory = moveit_msgs.msg.DisplayTrajectory()
                display_trajectory.trajectory_start = self.robot.get_current_state()
                display_trajectory.trajectory.append(plan)
                self.display_trajectory_publisher.publish(display_trajectory)
            
            # input('ready?')
            self.send_joint_trajectory_to_move_it(plan)
            # input('sent')

    def drop_down_to_point_custom(self,normal,point):
        pose=self.move_group.get_current_pose().pose
        p2=point
        
        joint_array,distance=self.plan_path_constant_z_axis(p1=[pose.position.x,pose.position.y,pose.position.z],p2=p2,n2=normal,ee_angle_corrector=self.closest_to_current_ee_angle_corrector)
        
        if len(joint_array)>1:
            print('enter 2')
            time,joint_array=self.time_path_parameratization(joint_angles=joint_array,scaling_metric=self.reorientation_angular_rate,method='joint_distance',b=self.transportb,c=1,cost=None)
            print('exit 2')
            # time,joint_angles,joint_velocities,joint_accelerations=self.fit_polynomials_to_joint_angles(joint_array,time)
            # time,joint_angles,joint_velocities,joint_accelerations=self.fit_polynomials_to_joint_angles(joint_array,time)
            print('enter 3')
            time,joint_angles,joint_velocities,joint_accelerations=self.fit_scipy_spline_to_joint_angles(joint_array,time,cost=distance,cost_type='joint_distance')
            print('exit 3')
            # input(np.array(time))
            # input(np.array(joint_angles))
            # input(np.array(joint_velocities))
            # input(np.array(joint_accelerations))
            # input('good?')
            print('enter 4')
            # self.print_array(np.array(time),Angles=np.array(joint_angles),Velocities=np.array(joint_velocities),Accelerations=np.array(joint_accelerations),message='reorientation_joint_angles')
            print('exit 4')
            print('enter 5')
            plan=self.create_joint_traj_data_type(time,joint_angles,joint_velocities,joint_accelerations)
            print('exit 5')
            print('enter 6')
            self.velocity_acceleration_jerk_check(velocity=joint_velocities,acceleration=joint_accelerations,jerk=None,velocity_max=100,acceleration_max=1000,jerk_max=10000,raise_exepction=False)
            print('exit 6')
            # change_limits=[np.pi/8,np.pi/4,np.pi/4,np.pi/2,0.1,abs(self.end_effector_joint_limits[0]-self.end_effector_joint_limits[-1])]
            print('enter 7')
            change_limits=[0.2]*6
            self.iterative_joint_angle_check(plan,change_limits=change_limits,start_joint_angles=list(self.move_group.get_current_joint_values()),raise_exepction=False)
            print('exit 7')
            print('enter 8')
            if self.display_trajectories==True:
                display_trajectory = moveit_msgs.msg.DisplayTrajectory()
                display_trajectory.trajectory_start = self.robot.get_current_state()
                display_trajectory.trajectory.append(plan)
                # Publish
                self.display_trajectory_publisher.publish(display_trajectory)
                # input('good?')
            self.send_joint_trajectory_to_move_it(plan)
            print('exit 8')
            
        # input('continue?')

    
    
    def get_above_point(self,normal,point):
        current_pose=self.return_read_point(delta=False)
        if current_pose[-1]<self.Transform([0,0,0])[-1]+self.Z_offset+normal[-1]*self.control_distance:
            # # waypoints = []
            wpose = self.move_group.get_current_pose().pose
            
            # # wpose=current_pose
            # # waypoints.append(copy.deepcopy(wpose))
            
            # # wpose.position.z
            # # up_point=deepcopy(point)
            
            # # waypoints.append(deepcopy(pose_goal))
            # # self.IK_SOLVER.move_group.limit_max_cartesian_link_speed(0.1,link_name='wrist_3_link')
            up_point=[wpose.position.x,wpose.position.y,wpose.position.z]
            self.lift_up_constant_ee_angle_corrector(up_point,-normal)

            # bool=False
            # while bool==False:
            #     try:
            #         (plan, fraction) = self.move_group.compute_cartesian_path(
            #         waypoints, 0.005, 0.0  # waypoints to follow  # eef_step
            #         )  # jump_threshold
            #         # if fraction<1.0:
            #         #     raise Exception('WARNING Fraction:'+str(fraction))
            #         self.iterative_constant_joint_angle_check(plan=plan,raise_exepction=True)
            #         change_limits=[np.pi/8,np.pi/4,np.pi/4,np.pi/2,0.1,abs(self.end_effector_joint_limits[0]-self.end_effector_joint_limits[-1])]
            #         self.iterative_joint_angle_check(plan=plan,change_limits=change_limits,start_joint_angles=list(self.move_group.get_current_joint_values()),raise_exepction=True)
            #         # input('good?')
            #         bool=self.move_group.execute(plan,wait=True)
            #     except:
            #         print('retrying trajectory')
        # input('continue?')
        
        pose=self.move_group.get_current_pose().pose
        p2=self.get_safety_points(tuple(point),normal=normal)

        
        # print('normal')
        # print(normal)

        # self.JA_cart_aprox_bounded_plannar_orientation_v2(p1=[pose.position.x,pose.position.y,pose.position.z],p2=p2,n2=normal)
        joint_array,distance=self.plan_path_constant_z_axis(p1=[pose.position.x,pose.position.y,pose.position.z],p2=p2,n2=normal)
        if len(joint_array)>1:
            print('enter 2')
            time,joint_array=self.time_path_parameratization(joint_angles=joint_array,scaling_metric=self.reorientation_angular_rate,method='joint_distance',b=self.transportb,c=1,cost=None)
            print('exit 2')
            # time,joint_angles,joint_velocities,joint_accelerations=self.fit_polynomials_to_joint_angles(joint_array,time)
            # time,joint_angles,joint_velocities,joint_accelerations=self.fit_polynomials_to_joint_angles(joint_array,time)
            print('enter 3')
            time,joint_angles,joint_velocities,joint_accelerations=self.fit_scipy_spline_to_joint_angles(joint_array,time,cost=distance,cost_type='joint_distance')
            print('exit 3')
            # input(np.array(time))
            # input(np.array(joint_angles))
            # input(np.array(joint_velocities))
            # input(np.array(joint_accelerations))
            # input('good?')
            print('enter 4')
            # self.print_array(np.array(time),Angles=np.array(joint_angles),Velocities=np.array(joint_velocities),Accelerations=np.array(joint_accelerations),message='reorientation_joint_angles')
            print('exit 4')
            print('enter 5')
            plan=self.create_joint_traj_data_type(time,joint_angles,joint_velocities,joint_accelerations)
            print('exit 5')
            print('enter 6')
            self.velocity_acceleration_jerk_check(velocity=joint_velocities,acceleration=joint_accelerations,jerk=None,velocity_max=100,acceleration_max=1000,jerk_max=10000,raise_exepction=False)
            print('exit 6')
            # change_limits=[np.pi/8,np.pi/4,np.pi/4,np.pi/2,0.1,abs(self.end_effector_joint_limits[0]-self.end_effector_joint_limits[-1])]
            print('enter 7')
            change_limits=[0.2]*6
            self.iterative_joint_angle_check(plan,change_limits=change_limits,start_joint_angles=list(self.move_group.get_current_joint_values()),raise_exepction=False)
            print('exit 7')
            print('enter 8')
            if self.display_trajectories==True:
                display_trajectory = moveit_msgs.msg.DisplayTrajectory()
                display_trajectory.trajectory_start = self.robot.get_current_state()
                display_trajectory.trajectory.append(plan)
                # Publish
                self.display_trajectory_publisher.publish(display_trajectory)
                # input('good?')
            self.send_joint_trajectory_to_move_it(plan)
            print('exit 8')
            
        # input('continue?')

    def cart_traj_same_pose_wp(self,point,normals=[0,0,0]):
        waypoints = []
        wpose = self.move_group.get_current_pose().pose
        waypoints.append(copy.deepcopy(wpose))
        pose_goal=wpose
        desired_tack_point=point
        #*scale the normal to the correct size
        # print('Current norm')
        # print(np.linalg.norm(normals))
        # print('current vector:')
        # print(normals)
        # print('new vector')
        # normalized_normals=normals/np.linalg.norm(normals)
        # print(normalized_normals)
        # input('looks good?')
        
        normalized_normals=normals/np.linalg.norm(normals)
        
        pose_goal.position.x = desired_tack_point[0]+normals[0]*self.control_distance
        pose_goal.position.y = desired_tack_point[1]+normals[1]*self.control_distance
        pose_goal.position.z = desired_tack_point[2]+normals[2]*self.control_distance
            
        waypoints.append(deepcopy(pose_goal))
        return waypoints
        
    def next_tack_greedy_on_line(self,current_tack_points):
        wpose = self.move_group.get_current_pose().pose
        point=[wpose.position.x,wpose.position.y,wpose.position.z]
            # print(np.array([point]))
            # print(np.array(adjusted_tack_points))

        if type(self.T)!=type(None):
            Transform=False
        else:
            Transform=True      

        normals=[self.get_desired_point_normal(tuple(n)) for n in current_tack_points]


        adjusted_tack_points=[]
        for n in range(len(current_tack_points)):
            adjusted_tack_points.append(self.get_adjusted_point(point_key=tuple(current_tack_points[n]),normal=normals[n]))

        x=scipy.spatial.distance.cdist(np.array([point]),np.array(adjusted_tack_points),'euclidean')
        closest_point_index=np.argmin(x)
        closest_point_adjusted=deepcopy(adjusted_tack_points[closest_point_index])
        closest_normal=normals[closest_point_index]

        print(current_tack_points)
        current_tack_points.remove(deepcopy(current_tack_points[closest_point_index]))

        return closest_normal,closest_point_adjusted,current_tack_points

    def execute_tack_points(self,get_next_tack=None,tack_points=None):
        if type(tack_points)==type(None):
            tack_points=deepcopy(self.tack_points)
        else:
            tack_points=deepcopy(tack_points)
        if get_next_tack==None:
            get_next_tack=self.next_tack_greedy_on_line
        # print('tack points length:')
        # print(len(self.tack_points))
        
        if type(self.normal_dict)==type(None):
            self.normal_dict={}
        
        if type(tack_points)!=type(list):
            tack_points=tack_points.tolist()
        # self.adjusted_point_dict={}
        

        # if type(self.T)!=type(None):
        #     Transform=False
        # else:
        #     Transform=True        

        # for n in tack_points:
        #*SHOULD THESE BE ADJUSTED
        
        while len(tack_points)>0:
            normals,closest_point,tack_points=get_next_tack(tack_points)

            self.get_above_point(normals,closest_point)
            # self.drop_down_to_point(point=closest_point,normals=normals)
            
            self.drop_down_to_point_custom(normals,closest_point)
            
            if self.sound_on==True:
                rospy.sleep(0.1)
                self.tacking_sound.play()
                rospy.sleep(1)

            if self.weld_on==True:
                self.tack.publish(self.tacking_time)
                rospy.sleep(self.post_weld_delay)
       
    def correct_joint_angles_to_current(self,joint,edge,current=None):
        #*Set the joint angle arrays to start at the current joint values
        if type(current)==type(None):
            curr=list(self.move_group.get_current_joint_values())
            # print('read')
            # print(curr)
            
            # print('passed in')
        else:
            curr=current
            # print(current)

        if not(math.isclose(curr[joint]*np.pi/180,edge.angles[0,joint]*np.pi/180,abs_tol=10**-3)):

            value=round(abs(curr[joint]-edge.angles[0,joint])/np.pi)*np.pi*np.sign((curr[joint]-edge.angles[0,joint]))
            edge.angles[:,joint]=edge.angles[:,joint]+value
            #! Maybe Return Something
        

    def absolute_angle_array(self,array):
        #*This goes through an array of angles and gets rid of multiples of 2pi between consecutive angles
        # input(array)
        for n in range(len(array)-1):
            if abs(array[n+1]-array[n])>0.8*np.pi:
                array[n+1]+=round(abs(array[n+1]-array[n])/np.pi)*np.pi*-1*np.sign((array[n+1]-array[n]))
        return array
        
    def adjust_by_pi(self,integer,edge,joint=None):
        #*add or minus some amount of pi to an edges joint angles
        if joint==None:
            joint=5
        edge.angles[:,joint]=edge.angles[:,joint]+np.pi*integer
    #* CHECK METHODS ABOVE ^^^^^^
    
    def next_weld_greedy_on_line(self,current_weld_lines,point=None,remove_point=True):
        #*weld lines is in self weld lines remeber
        #*Gets closest weld point relative to the closest weld points
        
        if type(point)==type(None):
            wpose = self.move_group.get_current_pose().pose
            point=[wpose.position.x,wpose.position.y,wpose.position.z]
            
        all_points=np.array(current_weld_lines)
        print('I aint got no type')
        print(all_points)
        print(type(all_points))
        if np.size(all_points)==0:
            return  None,None,None,None,None,np.inf,[]
        all_points=all_points.reshape(all_points.shape[0]*all_points.shape[1],all_points.shape[2])

        normals=[self.get_desired_point_normal(tuple(n)) for n in all_points]
        
        adjusted_weld_points=[]
        
        for n in range(len(all_points)):
            adjusted_weld_points.append(self.get_adjusted_point(point_key=tuple(all_points[n]),normal=normals[n]))

        x=scipy.spatial.distance.cdist(np.array([point]),adjusted_weld_points,'euclidean')
            
        closest_point_index=np.argmin(x)    
        closest_weld_line_index=np.floor(closest_point_index/2)
        remainder=closest_point_index%2
        
        current_weld_adjusted=deepcopy([adjusted_weld_points[closest_point_index],adjusted_weld_points[(closest_point_index+1+(-2*remainder))]])
        
        current_normals=[normals[closest_point_index],normals[(closest_point_index+1+(-2*remainder))]]
        
        distance=np.min(x)


        current_weld=deepcopy(current_weld_lines[int(closest_weld_line_index)])
        
        if remove_point==True:
            last_pile=deepcopy(current_weld_lines)
            current_weld_lines.remove(current_weld)
        else:
            return current_weld,remainder,closest_weld_line_index
            # last_pile=current_weld_lines

        normal1=current_normals[0]
        normal2=current_normals[1]
        p1=current_weld_adjusted[0]
        p2=current_weld_adjusted[1]
        #*what dictionary am I gonna use?????

        #* Minus Normal
        wp1=self.adjust_by_normal(point=p1,normal=normal1,sign=-1)
        wp2=self.adjust_by_normal(point=p2,normal=normal2,sign=-1)

        control_points=np.array(euclidean_linspace(wp1,wp2,100))
        signp=np.sign(np.cross(normal1,normal2)[-1])
        anglep=np.arccos(np.dot(normal1,normal2)/(np.linalg.norm(normal1)*np.linalg.norm(normal2)))
        #return 
        return p1,p2,normal1,normal2,control_points,distance,last_pile
    
    def execute_weld_lines(self,get_next_weld=None,weld_lines=None):
        if type(weld_lines)==type(None):
            weld_lines=deepcopy(self.weld_lines)
        else:
            weld_lines=deepcopy(weld_lines)
        if get_next_weld==None:
            get_next_weld=self.next_weld_greedy_on_line

        if type(self.normal_dict)==type(None):
            self.normal_dict={}

        current_weld_lines=weld_lines
        if type(current_weld_lines)!=type(list):
            current_weld_lines=current_weld_lines.tolist()

        while len(current_weld_lines)>0:
            self.current_edge==None
            current_welds=[]
            #!Is this for a recursive function
            p1,p2,normal1,normal2,control_points,distance,last_pile=get_next_weld(current_weld_lines)
            
            
            if type(control_points)==type(None):
                break
            
            # input(p2)
            # input(control_points[-1])
            # input(np.linalg.norm(control_points[-1]-p2))
            # input('okay but like what about this')
            #*Above here compact this^
            weld_edge=weld_edge_double_corner_turn(control_points=control_points,normal1=normal1,
            normal2=normal2,control_distance=self.control_distance)
            self.current_edge=weld_edge
            weld_edge.type='weld'

            weld_edge.make_smooth_line(Smoothness_Coefficient=5,Interpolation_Resolution=self.tour_resolution_welds,Weight_Ratio=None)
            
            #*Decide what this actually needs to be.....
            # weld_edge.Path_order(w1,w2)

            weld_edge.make_orientation_vectors()
                    
            current_JA=list(self.move_group.get_current_joint_values())
        
            weld_edge.compute_joint_distances(IK_object=self,seed_state=current_JA)

            for n in range(len(current_JA)):
                weld_edge.angles[:,n]=self.absolute_angle_array(weld_edge.angles[:,n])

            for n in range(len(current_JA)):
                self.correct_joint_angles_to_current(joint=n,edge=weld_edge)
            
            # input('get above')
            self.get_above_point(normal1,point=p1)
            # input('drop down')
            
            self.drop_down_to_point_custom(point=p1,normal=normal1)
            
            current_JA=list(self.move_group.get_current_joint_values())
            
            for n in range(len(current_JA)):
                self.correct_joint_angles_to_current(joint=n,edge=weld_edge)

            for n in range(len(current_JA)):
                weld_edge.angles[:,n]=self.absolute_angle_array(weld_edge.angles[:,n])

            current_welds.append(weld_edge)



            if len(current_weld_lines)>0:
                #*make sure this is handled right
                print(p2)

                p1_1,p2_1,normal1_1,normal2_1,control_points_1,distance,last_pile=get_next_weld(current_weld_lines,point=p2)
                # control_points_1=control_points_1[-1::-1]
                # next_weld_edge=None
                # print('distance')
                # print(math.isclose(distance,0.00,abs_tol=10**-2))
                # print('p1_1')
                # print(p1_1)                
                # print(p2)
                # input(distance)

                while math.isclose(distance,0.00,abs_tol=10**-2) and len(last_pile)>0:
                    next_weld_edge=weld_edge_double_corner_turn(control_points=control_points_1,normal1=normal1_1,normal2=normal2_1,control_distance=self.control_distance)
                    self.current_edge=next_weld_edge
                    next_weld_edge.type='weld'

                    next_weld_edge.make_smooth_line(Smoothness_Coefficient=5,Interpolation_Resolution=self.tour_resolution_welds,Weight_Ratio=None)
                    
                    # next_weld_edge.Path_order(w3,w4)

                    next_weld_edge.make_orientation_vectors()
                    # print(current_welds[-1].angles[-1])
                    
                    print('computing next edges joints')
                    print('utilizing this seed')
                    print(current_welds[-1].angles[-1])
                    # input('good?')

                    next_weld_edge.compute_joint_distances(IK_object=self,seed_state=current_welds[-1].angles[-1])
                    
                    self.current_edge=next_weld_edge

                    current_JA=list(self.move_group.get_current_joint_values())

                    print('after here:')

                    print(current_welds[-1].angles[-1])
                    print(next_weld_edge.angles[0])

                    # input('these compare edge joint angles')

                    for n in range(len(current_JA)):
                            self.correct_joint_angles_to_current(joint=n,edge=next_weld_edge,current=current_welds[-1].angles[-1])

                    # print(next_weld_edge.angles[0])
                    # input('corrected next edge joint angles')

                    for n in range(len(current_JA)):
                        next_weld_edge.angles[:,n]=self.absolute_angle_array(next_weld_edge.angles[:,n])
                    
                    # print(next_weld_edge.angles[0])
                    # input('absoluted next edge joint angles')


                    if self.end_effector_joint_limits[0]<=np.min(next_weld_edge.angles[:,5]) and np.max(next_weld_edge.angles[:,5]) <=self.end_effector_joint_limits[-1] and len(last_pile)>0:
                        
                        current_welds.append(deepcopy(next_weld_edge))
                        
                        if self.solution_type=='tsp':
                            self.repeat+=-1
                            self.tour_count+=1
                        
                        p1_1,p2_1,normal1_1,normal2_1,control_points_1,distance,last_pile=get_next_weld(current_weld_lines,point=p2_1)
                    else:
                        distance=100
                        current_weld_lines=last_pile
                # else:
                #     pass
            fail = False
            
            for edge_send in current_welds:
                current_JA=list(self.move_group.get_current_joint_values())
                # self.plot_edge(edge_send)
                for n in range(len(current_JA)):
                    if not(math.isclose(current_JA[n]*np.pi/180,edge_send.angles[0,n]*np.pi/180,abs_tol=10**-2)):
                        self.move_group.stop()
                        print('WARNING STARTING ANGLES DO NOT MATCH CURRENT ANGLES')
                        print(edge_send.angles[0])
                        print(current_JA)
                        rospy.sleep(5)
                        # self.move_group.stop()
                        #*If the first point of the edge does not match the current point kill the program
                        fail=True
                        # break
                        self.move_group.go(edge_send.angles[0])
                        input('well')
                        # exit()
                # if fail!=True:
                    # self.send_edge_to_moveit_scurve(edge_send,tacking=False,weld_it=False)
                try:
                    self.send_edge(edge_send,tacking=False,Welding=True)
                except:
                    self.move_group.stop()
                    print('Failure retrying exit')
                    
                    fail=True
                    # adjusted_weld_points=last_pile
                    exit()
        
        print('Welds Finished Lifting Up')
        self.get_above_point(normal=edge_send.normal2,point=self.return_read_point(delta=False))

    def print_array(self,time,Angles,Velocities,Accelerations,message=None):
        #*THis prints the edges position time and velocity over time
        pt=0.00001
        if type(self.fig2)==type(None):
            self.fig2, self.axs2 = plt.subplots(6,3)
        
        self.fig2.suptitle('Spline: Joint Angle Solutions for each point')
            # pt=5
            # self.fig2.ion()
        # self.fig2.clf()
        # if type(message)!=type(None):
        #     fig.suptitle(message)
        
        fig=self.fig2
        axs=self.axs2
        
        # fig.clf()
        # fig, axs = plt.subplots(6,3)
        for n in axs:
            for k in n:
                k.clear()
        # print(axs)
        # exit()
        strings=['\u03B8(degrees)','\u03B8(degrees)/s','\u03B8(degrees)/s^2']
        for n in axs:
            for k in range(len(n)):
                n[k].set_ylabel(strings[k])
                n[k].set_xlabel('time(seconds)')
        
        axs[0,0].scatter(time,Angles[:,0]*180/np.pi)
        axs[0,0].set_title('Q0p')
        axs[0,1].scatter(time,Velocities[:,0]*180/np.pi)
        axs[0,1].set_title('Q0v')
        axs[0,2].scatter(time,Accelerations[:,0]*180/np.pi)
        axs[0,2].set_title('Q0a')


        axs[1,0].scatter(time,Angles[:,1]*180/np.pi)
        axs[1,0].set_title('Q1p')
        axs[1,1].scatter(time,Velocities[:,1]*180/np.pi)
        axs[1,1].set_title('Q1v')
        axs[1,2].scatter(time,Accelerations[:,1]*180/np.pi)
        axs[1,2].set_title('Q1a')


        axs[2,0].scatter(time,Angles[:,2]*180/np.pi)
        axs[2,0].set_title('Q2p')
        axs[2,1].scatter(time,Velocities[:,2]*180/np.pi)
        axs[2,1].set_title('Q2v')
        axs[2,2].scatter(time,Accelerations[:,2]*180/np.pi)
        axs[2,2].set_title('Q2a')

        axs[3,0].scatter(time,Angles[:,3]*180/np.pi)
        axs[3,0].set_title('Q3p')
        axs[3,1].scatter(time,Velocities[:,3]*180/np.pi)
        axs[3,1].set_title('Q3v')
        axs[3,2].scatter(time,Accelerations[:,3]*180/np.pi)
        axs[3,2].set_title('Q3a')

        axs[4,0].scatter(time,Angles[:,4]*180/np.pi)
        axs[4,0].set_title('Q4p')
        axs[4,1].scatter(time,Velocities[:,4]*180/np.pi)
        axs[4,1].set_title('Q4v')
        axs[4,2].scatter(time,Accelerations[:,4]*180/np.pi)
        axs[4,2].set_title('Q4a')


        axs[5,0].scatter(time,Angles[:,5]*180/np.pi)
        axs[5,0].set_title('Q5p')
        axs[5,1].scatter(time,Velocities[:,5]*180/np.pi)
        axs[5,1].set_title('Q5v')
        axs[5,2].scatter(time,Accelerations[:,5]*180/np.pi)
        axs[5,2].set_title('Q5a')

        plt.pause(pt)
        # plt.show()
        # plt.show(block=False)

    def plot_edge(self,edge):
        #* Yeah this just plots the total solution angles
        t1=np.linspace(0,1,len(edge.angles))
        
        if type(self.fig3)==type(None):
            self.fig3, self.axs3 = plt.subplots(6)
            self.fig3.suptitle('Weld Edge: Joint Angle Solutions for each point')
            # self.fig3.ion()
        
        fig=self.fig3
        axs=self.axs3
        
        # fig.clear()
        for n in axs:
            n.clear()
        
        # input(dir(axs[0]))
        axs[0].scatter(t1,edge.angles[:,0]*180/np.pi)
        axs[0].set_title('Q0_ik')

        axs[1].scatter(t1,edge.angles[:,1]*180/np.pi)
        axs[1].set_title('Q1_ik')

        axs[2].scatter(t1,edge.angles[:,2]*180/np.pi)
        axs[2].set_title('Q2_ik')

        axs[3].scatter(t1,edge.angles[:,3]*180/np.pi)
        axs[3].set_title('Q3_ik')

        axs[4].scatter(t1,edge.angles[:,4]*180/np.pi)
        axs[4].set_title('Q4_ik')

        axs[5].scatter(t1,edge.angles[:,5]*180/np.pi)
        axs[5].set_title('Q5_ikpp')
        
        # fig.update()
        plt.pause(0.001)

        # plt.show()
    #*THESE ARE YOUR PLAN CHECKERS BELLOW
    def change_limit_check(self,joint_start,joint_end,change_limits,raise_exepction=False):
        for n in range(len(joint_start)):
            #*This range can be altered but it basically throws away a trajectory if there is a joint angle change greater than 90 degrees
            if abs(joint_end[n]-joint_start[n])>change_limits[n]:
                print('CHANGE LIMIT CHECK FAILURE')
                print('broke joint limit:'+str(n)+', with a change limit of: '+str(change_limits[n]))
                print('With a requested: '+str(abs(joint_end[n]-joint_start[n]))+'rads of joint angle change')
                self.move_group.stop()
                if raise_exepction==True:
                    raise Exception('WARNING BAD ALIGNMENT TRAJECTORY')
                return False
        return True

    # def iterative_within_joint_bounds_check(self,bounds=[-np.inf,np.inf,]):
    #     pass
    def velocity_acceleration_jerk_check(self,velocity=None,acceleration=None,jerk=None,velocity_max=100,acceleration_max=1000,jerk_max=10000,raise_exepction=False):
        
        if type(velocity)!=type(None):
            max=np.max(velocity)
            min=np.min(velocity)
            if abs(max)>velocity_max or abs(min)>velocity_max:
                print('VELOCITY LIMIT CHECK FAILURE')
                print('broke velocity limit:'+str(velocity_max))
                self.move_group.stop()
                if raise_exepction==True:
                    raise Exception('WARNING BAD ALIGNMENT TRAJECTORY')
                return False

        if type(acceleration)!=type(None):
            max=np.max(acceleration)
            min=np.min(acceleration)
            if abs(max)>acceleration_max or abs(min)>acceleration_max:
                print('ACCELERATION LIMIT CHECK FAILURE')
                print('broke acceleration limit:'+str(acceleration_max))
                self.move_group.stop()
                if raise_exepction==True:
                    raise Exception('WARNING BAD ALIGNMENT TRAJECTORY')
                return False

        if type(jerk)!=type(jerk):
            max=np.max(jerk)
            min=np.min(jerk)
            if abs(max)>jerk_max or abs(min)>jerk_max:
                print('JERK LIMIT CHECK FAILURE')
                print('broke jerk limit:'+str(jerk_max))
                self.move_group.stop()
                if raise_exepction==True:
                    raise Exception('WARNING BAD ALIGNMENT TRAJECTORY')
                return False

        return True
    
    def iterative_joint_angle_check(self,plan,change_limits,start_joint_angles=None,raise_exepction=False):
        bool=True
        
        if type(start_joint_angles)!=type(None):
            start_limits=[0.01]*6
            bool=self.change_limit_check(joint_start=start_joint_angles,joint_end=list(plan.joint_trajectory.points[0].positions),change_limits=start_limits)

        if bool:
            for n in range(0,len(plan.joint_trajectory.points)-1):
                joint_start=list(plan.joint_trajectory.points[n].positions)
                joint_end=list(plan.joint_trajectory.points[n+1].positions)
                bool=self.change_limit_check(joint_start=joint_start,joint_end=joint_end,change_limits=change_limits)
        
        if raise_exepction==True and not(bool):
            raise Exception('WARNING BAD ALIGNMENT TRAJECTORY')
        return bool
        
    def iterative_fk_bounds_check(self,plan,bounds=None,raise_exepction=False):
        if type(bounds)==type(None):
            bounds=[-np.inf,np.inf,-np.inf,np.inf,self.Transform([0,0,0])[-1]+self.Z_offset,np.inf]

        path_joint_angles=[list(plan.joint_trajectory.points[n].positions) for n in range(len(plan.joint_trajectory.points))]
        fk_array=np.array(self.get_fk_array(path_joint_angles))
        for n in range(len(fk_array)):
            point=fk_array[n]
            bool=self.point_within_bounds_check(point=point,bounds=bounds)
            if raise_exepction==True:
                raise Exception('WARNING BAD ALIGNMENT TRAJECTORY')
            else:
                bool=False
                return bool
        bool=True

    def point_within_bounds_check(self,point,bounds,raise_exepction=False):
        bool=True
        for n in range(6):
            if point[n]<bounds[n*2] or point[n]>bounds[n*2]:
                print('FK BOUND LIMIT CHECK FAILURE')
                print('broke bound limit:'+str(n)+', with point:'+str(point))
                self.move_group.stop()
                if raise_exepction==True:
                    raise Exception('WARNING BAD ALIGNMENT TRAJECTORY')
                else:
                    bool=False
                    return bool
        return bool
    
    def iterative_constant_joint_angle_check(self,plan,joint=[5],curr=None,tol=10**-1,raise_exepction=False):
        if type(curr)==type(None):
            curr=list(self.move_group.get_current_joint_values())
        bool=True
        for n in plan.joint_trajectory.points:
            for k in joint:
                if not(math.isclose(n.positions[k],curr[k],abs_tol=tol)):
                    print('CONSTANT JOINT LIMIT CHECK FAILURE')
                    print('broke constant joint limit:'+str(k)+', with angle:'+str(n.positions[k])+', For desired angle: '+str(curr[k])+', with an absolute tolerance of: '+str(tol))
                    self.move_group.stop()
                    if raise_exepction==True:
                        raise Exception('WARNING BAD ALIGNMENT TRAJECTORY')
                    else:
                        bool=False
                        return bool
        return bool 
    
    #* THESE ARE YOUR PLAN CHECKERS ABOVE

    def Transform(self,point,normal=False):
        #*This just applies the linear transform to any point or line/normal
        if type(self.T)==type(None):
            T=np.eye(4)
        else:
            T=self.T
        if normal==False:
            return np.matmul(T,np.append(np.array(point),[1]))[0:3]
        else:
            return np.matmul(T[0:3,0:3],np.array(point))
         
    def InvTransform(self,point,normal=False):
        #*This just applies the linear transform to any point or line/normal
        InvT=np.matrix(self.T).I
        if normal==False:
            return np.matmul(InvT,np.append(np.array(point),[1]))[0:3]
        else:
            return np.matmul(InvT[0:3,0:3],np.array(point))
          
    def compute_normal(self,P1,Transform=True):
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
        if Transform:
            tac_normals=self.Transform(tac_normals,normal=True)        
        return (tac_normals)   

    def init_move_group(self,weld_on=True,sound_on=False,display_trajectories=False):
        self.display_trajectories=display_trajectories
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_node',
                anonymous=False)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        #*Initialize the move it node and all the move it commander objects^
        self.aligned_tool0_vector=np.array([0,1,0])
        self.assumed_z=([0,0,1])
        #*This is basically noting that the tool is pointed along the ee y axis
        group_name = "manipulator"
        move_group=moveit_commander.MoveGroupCommander(group_name)
        #*^ create the move group refrence
        self.move_group = move_group
        #*Set up the IK solver and defualt it to Speed
        self.IK_SOLVER = IK("base_link",
                "tool0",solve_type="Distance",timeout=0.015,epsilon=0.0001)
        
        self.weld_velocity=0.1
        # self.transport_velocity=0.005*2*2*2*2*2*2*2*2*2

        self.transport_velocity=0.5*3
        # self.weld_velocity=0.006
        # self.weld_velocity=0.01
        
        # self.interior_transport_velocity=self.transport_velocity/2
        self.tacking_time=1000
        self.post_weld_delay=1
        #*These are the parameters that influence the robots speed and welding speed etc.^^^
        self.pose_default=self.set_pose(Transformation=np.eye(4),set_as_default=False)
        #TODO: why did I do this here????^^^ I belive this is properly done up a level in the TCP class
        #*defualt pose of the weld mesh*^
        self.weld_on=weld_on

        # self.move_group.set_planning_time(30)
        self.move_group.allow_looking(True)
        self.move_group.allow_replanning(True)
        self.move_group.set_max_velocity_scaling_factor(1.0)
        self.move_group.set_max_acceleration_scaling_factor(1.0)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
        
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
        # print('header')
        # print(header)
        header.frame_id='base_link'
        fk_link_names=self.robot.get_link_names()
        robot_state=self.robot.get_current_state()
        robot_state.joint_state.position=joint_values
        sol=fk(header,fk_link_names,robot_state)
        sol=sol.pose_stamped[-1].pose.position
        Point=[sol.x,sol.y,sol.z]
        return Point 
    
    def get_fk_array(self,joint_values_array):
        rospy.wait_for_service('compute_fk')
        gpf=moveit_msgs.srv.GetPositionFK()
        fk=rospy.ServiceProxy('compute_fk',gpf)
        header=std_msgs.msg.Header()
        # print('header')
        # print(header)
        header.frame_id='base_link'
        fk_link_names=self.robot.get_link_names()
        robot_state=self.robot.get_current_state()
        points=[]
        for n in joint_values_array:
            robot_state.joint_state.position=n
            sol=fk(header,fk_link_names,robot_state)
            sol=sol.pose_stamped[-1].pose.position
            points.append([sol.x,sol.y,sol.z])
        return points
    
    def return_read_point(self,point=None,fake_cord=None,fake_rotation=None,noise_scale=None,return_rotation=False,delta=True):
        #*This just returns the current read point from the robot but can also accept fake rotations and cords etc. for testing
        if fake_cord==None and fake_rotation==None:
            pose= self.move_group.get_current_pose().pose
            position= [pose.position.x,pose.position.y,pose.position.z]
            orientation= [pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
            R=scipy.spatial.transform.Rotation.from_quat(orientation)
            # print(R.as_matrix())
            measurement_delta=(50.86-39.18)/1000
            if delta==True:
                point=measurement_delta*np.matmul(R.as_matrix(),[0,np.sqrt(2)/2,np.sqrt(2)/2])+np.array(position)
            else:
              point=np.array(position)  
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
    
    def add_weld_mesh(self,filename):
        #*Add the weld mesh with the pose defualt positioning it in the work space
        self.scene.add_mesh(name='weld_mesh', pose=self.pose_default, filename=filename)
        x=self.scene.get_objects()
        self.scene.add_object(x['weld_mesh'])
        moveit_commander.ApplyPlanningSceneRequest(x['weld_mesh'])
        #*Ive used this exit so it exits the program very quickly after adding the mesh so I can verify its position without the robot moving
        # exit()
        
    def get_rotation_from_vectors(self,v1,v2):
        #*uhhh what am I doing here
        #*assumming both of these vectors are i\hatn the x-y plane this finds the rotation between them we use this to position our welding assuming always that the global z is aligned with the -z of the body or ee
        k=np.cross(v1,v2)
        p=scipy.spatial.transform.Rotation.from_rotvec((np.pi*np.array([0,1,0])))
        theta=np.arccos(np.dot(v1,v2)/(np.linalg.norm(v1)*np.linalg.norm(v2)))
        if math.isclose(abs(theta),np.pi,abs_tol=1e-6):
            r=scipy.spatial.transform.Rotation.from_rotvec((theta*self.assumed_z))
        elif tuple(k)==tuple(np.zeros(3)):
            r=np.eye(3)
        else:
            k=k/np.linalg.norm(k)
            r=scipy.spatial.transform.Rotation.from_rotvec((theta*np.array(k)))
        
        # print(r.as_matrix)
        # print(p.as_matrix)
        # print((r*p).as_matrix())
        # print((p*r).as_matrix())
        # # print(((r*p)-(p*r)).as_matrix())
        # input('hmmmmm?')

        # print(theta*180/np.pi)
        # print(r.as_matrix())
        # print(p.as_matrix())

        r=r*p
        # input(r.as_matrix())
        #*We return this rotation
        return r
    
    def get_IK(self,point,orientation_vector,seed_state=None,angle_corrector=None):    
        
        # orientation_vector=np.array(orientation_vector)/np.linalg.norm(orientation_vector)
        # if type(travel_vector)==type(None):
        #     z_vector=np.array([0,0,-1])
        #     travel_vector=np.cross(orientation_vector,z_vector)
        #     travel_vector=travel_vector/np.linalg.norm(travel_vector)
        #     y_vector=orientation_vector
        # else:
        #     perp_vect=np.cross(orientation_vector,travel_vector)
        #     perp_vector=perp_vect/np.linalg.norm(perp_vect)
        #     y_vector=orientation_vector+perp_vect/2
        #     y_vector=y_vector/np.linalg.norm(y_vector)





        #*This is the method to get and return the inverse kinematics based on the orientation vector and point passed in
        if type(seed_state)==type(None):
            seed_state=self.move_group.get_current_joint_values()
        x=point[0]
        y=point[1]
        z=point[2]

        # print('orientation and orientation norm')
        # print(orientation_vector)
        # print(np.linalg.norm(orientation_vector))
        # input('good?')

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
            # if np.round((angles[4]/(np.pi/2)))%1==0:
            #     angles[4]=(np.pi/2)*np.round((angles[4]/(np.pi/2)))*np.sign(angles[4])
            angles[4]=-np.pi/2
            # input('the angle is: '+str(angles[4]*(180/np.pi)))

        else:
            angles=None

        
        #*Return the angles
        return angles

    def send_edge(self,edge,tacking=False,Welding=True):
        
        Joint_Angles=edge.angles
        Path=edge.Path

        #*Compute the edge here
        time,joint_array=self.time_path_parameratization(joint_angles=Joint_Angles,scaling_metric=self.weld_velocity,method='path_distance',b=self.weldb,c=1,cost=edge.Path_Distance)
        #4.1
        time,joint_angles,joint_velocities,joint_accelerations=self.fit_scipy_spline_to_joint_angles(joint_array,time,cost=edge.Path_Distance,cost_type='path_distance',Smoothness_Coefficient=self.smoothness_coefficient_weld)
        
        # print('in send edge')
        # self.print_array(time,Angles=np.array(joint_angles),Velocities=np.array(joint_velocities),Accelerations=np.array(joint_accelerations),message='weld_joint_angles')
        
        plan=self.create_joint_traj_data_type(time,joint_angles,joint_velocities,joint_accelerations)
            
        if self.display_trajectories==True:
            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = self.robot.get_current_state()
            display_trajectory.trajectory.append(plan)
            # Publish
            self.display_trajectory_publisher.publish(display_trajectory)
            # input('good?')




        self.velocity_acceleration_jerk_check(velocity=joint_velocities,acceleration=joint_accelerations,jerk=None,velocity_max=100,acceleration_max=1000,jerk_max=10000,raise_exepction=False)
        # change_limits=[np.pi/8,np.pi/4,np.pi/4,np.pi/2,0.1,abs(self.end_effector_joint_limits[0]-self.end_effector_joint_limits[-1])]
        
        change_limits=[0.2]*6
        self.iterative_joint_angle_check(plan,change_limits=change_limits,start_joint_angles=list(self.move_group.get_current_joint_values()),raise_exepction=True)
        
        if self.sound_on==True:
                rospy.sleep(0.1)
                self.working_sound.play()
                rospy.sleep(1)
            
        if self.weld_on==True:
            self.toggle_weld.publish()

        #*Send the edge right here
        self.send_joint_trajectory_to_move_it(plan)
        
        if self.weld_on==True:
                self.toggle_weld.publish()
                rospy.sleep(self.post_weld_delay)

    def configuration_type(self,angles):
        #*Take the angles and return what configuration type it is
        if type(angles)==type(None):
            # print(angles)
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
        
    def lock_geometric_type(self,ee_limits):
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
        
        joints=self.move_group.get_current_joint_values()
        lower_bounds=[-2*np.pi]*6
        upper_bounds=[2*np.pi]*6
        lower_bounds[1]=-np.pi
        upper_bounds[1]=0
        # lower_bounds[-2]=joints[-2]+(4*(np.pi/180))
        # upper_bounds[-2]=joints[-2]-(4*(np.pi/180))

        lower_bounds[-1]=ee_limits[0]
        upper_bounds[-1]=ee_limits[-1]

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

        elif joints[2]<=0:
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
        wrist_1_average=np.average([lower_bounds[3],upper_bounds[3]])
        wrist_1_constraint.position=np.average([lower_bounds[3],upper_bounds[3]])
        wrist_1_constraint.tolerance_above=np.pi/2
        wrist_1_constraint.tolerance_below=np.pi/2
        wrist_1_constraint.weight=1

        # wrist_2_constraint=moveit_msgs.msg.JointConstraint()
        # wrist_2_constraint.joint_name=JOINT_NAMES[4]
        # wrist_2_average=np.average([lower_bounds[4],upper_bounds[4]])
        # wrist_2_constraint.position=np.average([lower_bounds[4],upper_bounds[4]])
        # wrist_2_constraint.tolerance_above=(4*(np.pi/180))
        # wrist_2_constraint.tolerance_below=(4*(np.pi/180))
        # wrist_2_constraint.weight=1


        wrist_3_constraint=moveit_msgs.msg.JointConstraint()
        wrist_3_constraint.joint_name=JOINT_NAMES[-1]
        wrist_3_average=np.average([lower_bounds[-1],upper_bounds[-1]])
        wrist_3_constraint.position=wrist_3_average
        wrist_3_constraint.tolerance_above=upper_bounds[-1]-wrist_3_average
        wrist_3_constraint.tolerance_below=wrist_3_average-lower_bounds[-1]
        wrist_3_constraint.weight=1

        constraints=self.move_group.get_path_constraints()
        constraints.joint_constraints=[shoulder_lift_joint_constraint,elbow_joint_constraint,wrist_1_constraint,wrist_3_constraint]
        
        self.move_group.set_path_constraints(constraints)
        self.IK_SOLVER.set_joint_limits(lower_bounds, upper_bounds)
        self.upper_bounds=upper_bounds
        self.lower_bounds=lower_bounds
