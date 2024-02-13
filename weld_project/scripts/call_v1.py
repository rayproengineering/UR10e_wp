#!/usr/bin/env python


import scipy
import numpy as np
import deprecated_weld_tsp_solver_v1
from deprecated_weld_tsp_solver_v1 import TSP_solution
from deprecated_weld_tsp_solver_v1 import *
# import Create_Weld_Mesh
# from Create_Weld_Mesh import *
# from Create_Weld_Mesh import Weld_Mesh

import Weld_Select
from Weld_Select import Create_Welds
from Weld_Select import *
import pyvista as pv

if __name__ == "__main__":
    #*step 1) open the part and select the required points
    Weld_Mesh_file_name='Weld_Mesh_sangam_roman_edit_v3.vtk'
    #*This is the refremced vtk, mesh file
    Weld_Mesh=pv.read(Weld_Mesh_file_name)
    #*read the mesh
    plotter=pv.Plotter()
    #*creater the plotter object to distplay
    Weld_Selector=Create_Welds(Weld_Mesh,plotter=plotter)
    #*^Here I pass in the plotter and the Mesh and then call this objects methods

    # msg1=('Please select the desired weld lines, select two points for each line. Press Q to finalize')
    # Weld_Selector.query_lines(msg1)
    # desired_points=Weld_Selector.picked_points_fortac
    # desired_weld_lines=Weld_Selector.picked_lines
    # np.save('desired_points_rs',desired_points)
    # np.save('desired_weld_lines_rs',desired_weld_lines)
    # exit()
    desired_points=np.load('desired_points_rs.npy')
    desired_weld_lines=np.load('desired_weld_lines_rs.npy')
    # msg3=('Please select the points you would like to match to the digital twin')
    # Weld_Selector.query_points(msg3)
    # desired_point_match=deepcopy(Weld_Selector.picked_tac_points)
    # print(desired_point_match)
    # np.save('desired_points_match',desired_point_match)
    # exit()
    desired_point_match=np.load('desired_points_match.npy')
    #*okay right here you get the cordinates of the desired points but you need to match it.....
    #*so create some kind of function to get the points showing them on pyvista and plotting them as it takes in the data.....

    
    # random=np.random.random()
    random=np.pi/2*np.random.random()
    random=0
    # # random=np.pi/8
    
    
    # # rotation=np.eye(3)
    randome2=np.random.random()
    randome3=np.random.random()
    # # randome2=np.random.random()
    # # randome3=np.random.random()
    
    fake_pose_cord=[0.1+(randome2*0.1),0.1+(randome3*0.1),0]
    
    # # fake_pose_cord=[0.2589,-0.8,0]
    # # fake_pose_cord=[0.2,0.2,0]
    
    # # rotation=[[np.cos(rotation),-np.sin(rotation),0],[np.sin(rotation),np.cos(rotation),0],[0,0,1]]
    # # print(rotation)
    # rotation=np.array([[np.cos(random),-np.sin(random),0],[np.sin(random),np.cos(random),0],[0,0,1]])
    # # scaling=1
    # # rotation=list(np.load('rotation_current.npy'))
    # # fake_pose_cord=list(np.load('position_current.npy'))
    IK_SOLVER=Inverse_Kinematics_Solver(weld_on=True)
    #*Initialize the IK_Solver Object mainly used to interact with move it
    matched_points=[]
    #*Initialize a empty set of points to match
    # for n in desired_point_match:
    #     #* #For each point in the desired point array read a matching point from the IK solver object
    #     get = input("Press Enter to Take a Point:")
    #     # *       #take input from user
    #     # print(n)
    #     # point=IK_SOLVER.return_read_point(point=n,fake_cord=fake_pose_cord,fake_rotation=rotation,noise_scale=None)
    #     point=IK_SOLVER.return_read_point()
    #       #*read point from robot
    #     print('Read Point:')
    #     print(point)
    #     matched_points.append(deepcopy(point))
    #   #*append the read point to the points to match to the desired
    #     print(matched_points)

    # matched_points=np.array(matched_points)
    ##*Turn a list into a np.array object
    # np.save('matched_points',matched_points)
    ##*saved the matched points for future use
    # # # # np.save('rotation_current',rotation)
    # # # # np.save('position_current',fake_pose_cord)
    #* This was for testing^
    # exit()
    ##*exit the program if you want after gettings the matched points
    matched_points=np.load('matched_points.npy')
    ##*load the previously read points from the robot^
    # print(matched_points)
    
    # matched_points[:,0]+=-0.3*np.sign(matched_points[0,2])
    matched_points[:,0]+=-0.5
    # print(matched_points)
    # input('Prepare Robot for Welding, Press Enter to Continue:')
    # matched_points=list(matched_points)[1:][:]+list(matched_points)[0][:]
    # print(matched_points)
    T=Weld_Selector.estimate_pose(desired_points=desired_point_match,measured_points=matched_points)
    #*Call the pose matcher in the Weld Selector
    IK_SOLVER.set_pose(Transformation=T,set_as_default=True)
    #*Set the pose of the weld mesh in rviz
    
    
    
        


    ##Test code for checking point alignments\/\/\/\/
    ##
    ##
    # Solution=TSP_solution(IK_SOLVER=IK_SOLVER)
    # #Generate the object for the higher level planning of a nodal TSP solution based on joint cost
    # Solution.configuration=Solution.configuration_type(Solution.IK_SOLVER.move_group.get_current_joint_values())
    # ##This preempts the geometric locking of the robot if enabled, to the current geometric type
    # Solution.lock_geometric_type()
    # input('Next Point?')

    # for n in range(len(matched_points)):
    #     pose=IK_SOLVER.move_group.get_current_pose()
    #     pose.pose.position.x=matched_points[n,0]
    #     pose.pose.position.y=matched_points[n,1]
    #     pose.pose.position.z=matched_points[n,2]
    #     IK_SOLVER.move_group.go(pose)
    #     input('Next Point?')
    # #Test code for checking point alignments^

    # exit()

    IK_SOLVER.add_weld_mesh()
    ## add the weld mesh with the objects method^
    # exit()
    ## exit here if you want to just check the mesh aligment



    ##Stupid \|/

    # print(T)
    # Weld_Mesh2=deepcopy(Weld_Mesh)
    # first_d=deepcopy(desired_points)
    # Weld_Mesh.transform(T,transform_all_input_vectors=True,inplace=True)
    # Weld_Mesh.compute_normals(auto_orient_normals=True,consistent_normals=True,split_vertices=False,inplace=True)
    
    # Stupid ^

    ## Stupid \|/

    # for n in range(len(desired_points)):
    #     desired_points[n]=np.matmul(T,np.append(np.array(desired_points[n]),[1]))[0:3]



    # for n in desired_weld_lines:
    #     n[0]=np.matmul(T,np.append(np.array(n[0]),[1]))[0:3]
    #     n[1]=np.matmul(T,np.append(np.array(n[1]),[1]))[0:3]

    ## Stupid ^
    

    Solution=TSP_solution(IK_SOLVER=IK_SOLVER)
    ##Generate the object for the higher level planning of a nodal TSP solution based on joint cost
    Solution.run_solution(Weld_Mesh=Weld_Mesh,plotter=plotter,tac_points=desired_points,weld_lines=desired_weld_lines,Transformation=T)
    ##Run the solution
