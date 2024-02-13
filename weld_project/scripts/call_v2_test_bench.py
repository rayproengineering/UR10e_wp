#!/usr/bin/env python


import scipy
import numpy as np
import condensed_weld_solver_v2
from condensed_weld_solver_v2 import *
# import Create_Weld_Mesh
# from Create_Weld_Mesh import *
# from Create_Weld_Mesh import Weld_Mesh

import Weld_Select
from Weld_Select import Create_Welds
from Weld_Select import *
import pyvista as pv

PART_FILE_NAME='exosent_grid_v3.stl'


class solver_test_bench():
    def __init__(self):
        self.plotter=pv.Plotter()
        self.T=None
        self.condensed_solver=None
    
    def init_solver(self,Weld_Mesh=None):
        if self.Weld_Mesh==None:
            self.Weld_Mesh=Weld_Mesh
        self.condensed_solver=condensed_solver(self.Weld_Mesh)
        self.condensed_solver.init_move_group()
        self.condensed_solver.set_plotter(self.plotter)
        


    def get_weld_normals_from_points_n_orientation(points,orientations):
        p1=points[0]
        p2=points[1]

        pdist=np.linalg.norm(np.array(p2)-np.array(p1))
        weld_length=pdist
        number_of_lines=np.floor(pdist/(weld_length))

        directional_vector=(np.array(p2)-np.array(p1))/pdist
        
        R1=scipy.spatial.transform.Rotation.from_quat(orientations[0])
        R2=scipy.spatial.transform.Rotation.from_quat(orientations[-1])
        print(R1)
        print(R2)
        y_orientation=(R1.as_matrix()[1,0:3]+R2.as_matrix()[1,0:3])/2
        general_normal=np.cross(directional_vector,[0,0,1])
        general_normal=general_normal*-1*np.sign(np.dot(general_normal,y_orientation))
        normal_1=general_normal+directional_vector
        normal_1=normal_1/np.linalg.norm(normal_1)
        normal_2=general_normal+-directional_vector
        normal_2=normal_2/np.linalg.norm(normal_2)
        normal_1[-1]=1
        normal_2[-1]=1
        normal_1=normal_1/np.linalg.norm(normal_1)
        normal_2=normal_2/np.linalg.norm(normal_2)
        
        return normal_1,normal_2
        

    def init_parameter_test(self,use_old_data=False):
        self.init_solver()

        # for n in range(0,int(number_of_lines)):
        #     weld_lines.append([p1+n*weld_length*directional_vector,p1+(n+1)*weld_length*directional_vector])

        number_of_welds=input('How many welds would you like to perform:')
        while type(number_of_welds)!=type(int):
            print('Please enter a valid Integer')
            number_of_welds=input('How many welds would you like to perform:')
        
        # subdivide=input('Would')

        weld_lines=[]
        # weld_normals=[]
        weld_normal_point_dict={}

        for n in range(0,number_of_welds):
            print('Weld Line: '+str(n))
            print('Press Enter to Aquire The First Point:')
            point1,orientation1=self.condensed_solver.return_read_point(return_rotation=True)
            print('Read Point:')
            print(point1)    
            confirm=input('was that point correct[y/N]?:')

            while confirm!='y' and confirm!='Y':
                print('Press Enter to Aquire The First Point:')
                point1,orientation1=self.condensed_solver.return_read_point(return_rotation=True)
                #*read point from robot
                print('Read Point:')
                print(point1)
        
            print('Press Enter to Aquire The Second Point:')
            point2,orientation2=self.condensed_solver.return_read_point(return_rotation=True)
            print('Read Point:')
            print(point2)    
            confirm=input('was that point correct[y/N]?:')

            while confirm!='y' and confirm!='Y':
                print('Press Enter to Aquire The First Point:')
                point2,orientation2=self.condensed_solver.return_read_point(return_rotation=True)
                #*read point from robot
                print('Read Point:')
                print(point2)

            normal_1,normal_2=self.get_weld_normals_from_points_n_orientation(points=[point1,point2],orientations=[orientation1,orientation2])

            weld_lines.append([point1,point2])
            weld_normal_point_dict[tuple(point1)]=normal_1
            weld_normal_point_dict[tuple(point2)]=normal_2
            
        wanna_tack=input('Would you like the start and stop points of the weld to be tack points[y/N]:')
        if wanna_tack=='y' or wanna_tack=='Y':
            wanna_tack=True
        else:
            wanna_tack=False

        self.condesed_solver.set_weld_goals(tack_points=list(weld_normal_point_dict.keys()),weld_lines=weld_lines)
        self.condesed_solver.normal_dict=weld_normal_point_dict
        self.condesed_solver.current_edge=None

        execute=input('would you like to execute?[y/N]:')
        if execute=='y' or execute=='Y':
            self.condensed_solver.run_solution(Tack=wanna_tack,Weld=True)        

    def init_part_test(self,Weld_Mesh_file_name,use_old_weld_goals_data=True,use_old_matched_points=True,register_new_points=False):
        self.Weld_Mesh_file_name=Weld_Mesh_file_name
        self.Weld_Mesh=pv.read(Weld_Mesh_file_name)
        self.Weld_Selector=Create_Welds(self.Weld_Mesh,plotter=self.plotter)
        
        if self.condensed_solver==None:
            self.init_solver(Weld_Mesh=self.Weld_Mesh)
        
        if type(self.T)==type(None):
            if use_old_weld_goals_data==True:
                self.desired_points=np.load('desired_points_np.npy')
                self.desired_weld_lines=np.load('desired_weld_lines_np.npy')
            else:
                self.querry_desired_points()

            

            if use_old_matched_points==True:
                self.desired_point_match=np.load('desired_points_match.npy')
            else:
                self.querry_digital_twins_matching_points()

            if register_new_points==True:
                self.register_part_points(number_of_points=len(self.desired_point_match))
            else:
                self.matched_points=np.load('matched_points.npy')

            self.T=self.Weld_Selector.estimate_pose(desired_points=self.desired_point_match,measured_points=self.matched_points)
        
        self.condensed_solver.set_pose(Transformation=self.T,set_as_default=True)
        self.condensed_solver.add_weld_mesh(filename=self.Weld_Mesh_file_name)
        self.condensed_solver.set_transform(self.T)
        self.condensed_solver.set_weld_goals(weld_lines=self.desired_weld_lines,tack_points=self.desired_points)

    def run_solution(self,cycles=1,Weld=True,Tack=True):
        for n in range(0,cycles):
            print('TRIAL:'+str(n))
            self.condensed_solver.run_solution(Weld=Weld,Tack=Tack)

    def querry_desired_points(self):
        msg1=('Please select the desired weld lines, select two points for each line. Press Q to finalize')
        self.Weld_Selector.query_lines(msg1)
        self.desired_points=self.Weld_Selector.picked_points_fortac
        self.desired_weld_lines=self.Weld_Selector.picked_lines
        np.save('desired_points_np',self.desired_points)
        np.save('desired_weld_lines_np',self.desired_weld_lines)

    def querry_digital_twins_matching_points(self):
        msg3=('Please select the points you would like to match to the digital twin')
        self.Weld_Selector.query_points(msg3)
        self.desired_point_match=deepcopy(self.Weld_Selector.picked_tac_points)
        # print(desired_point_match)
        np.save('desired_points_match',self.desired_point_match)

    def register_part_points(self,return_rotation=False,number_of_points=None,fake_points=None,fake_cord=None,fake_rotation=None):
        matched_points=[]

        if number_of_points==None:
            number_of_points=len(fake_points)

        for n in range(0,number_of_points):
            #* #For each point in the desired point array read a matching point from the IK solver object
            get = input("Press Enter to Take a Point:")
            # *       #take input from user
            # print(n)

            if fake_cord==None and fake_rotation==None:
                point=self.condensed_solver.return_read_point()
                print('Read Point:')
                print(point)
            
            else:
                point=self.condensed_solver.return_read_point(point=fake_points[n],fake_cord=fake_cord,fake_rotation=fake_rotation,noise_scale=None)
                #*read point from robot
                print('Read Point:')
                print(point)
            
            confirm=input('was that point correct[y/N]?:')
            while confirm!='y' and confirm!='Y':
                point=self.condensed_solver.return_read_point(point=fake_points[n],fake_cord=fake_cord,fake_rotation=fake_rotation,noise_scale=None)
                #*read point from robot
                print('Read Point:')
                print(point)
            
            matched_points.append(point)
          #*append the read point to the points to match to the desired
            # print(matched_points)

        self.matched_points=np.array(matched_points)
        np.save('matched_points',matched_points)
        
        # ##*Turn a list into a np.array object
        
        ##*saved the matched points for future 
        # # # # np.save('rotation_current',rotation)
        # # # # np.save('position_current',fake_pose_cord)

def main():
    solver_interface=solver_test_bench()
    solver_interface.init_part_test(Weld_Mesh_file_name='exosent_grid_v3.stl')
    # solver_interface.condensed_solver.run_tsp_tack()
    # solver_interface.condensed_solver.run_tsp_weld()
    # solver_interface.condensed_solver.generate_graph_simple()
    # try:
    solver_interface.run_solution(cycles=10,Weld=True,Tack=False)
    return solver_interface
    # except:
        # return solver_interface

if __name__ == "__main__":
    si=main()


