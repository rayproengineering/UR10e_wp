import math
import numpy as np
import copy
from copy import deepcopy
import scipy

import helper_functions_n_objects
from helper_functions_n_objects import *




class edge_line:
    #*This is the main edge line class that stores and calculates all the geometric (not acceleration or velocity) data having to do with the robot moving between two nodes
    def __init__(self,control_points,normal1,normal2):
        #*This accepts an array of control points, and two vectors normal1 and 2 that represent the orientation in the x,y plane at the start and finish of the movement 
        normal1[2]=0
        #*That is why we zero out the vertical components
        normal1=normal1/np.linalg.norm(normal1)
        normal2[2]=0
        normal2=normal2/np.linalg.norm(normal2)
        self.normal1=normal1
        self.normal2=normal2
        self.controlpoints=deepcopy(control_points)
        
        self.orientation_vectors=None

        self.Path_Direction=None
        self.pose_default=None
        self.Rotatable=False
        self.repeated=False
        self.type='tack'
        #*by defualt all edges are of the type tack but this may be overwritten
        self.control_rotation=False
        #*by defualt youy can not control the way the robot turns from normal1 to normal2
        self.clockwise=None
        #*This is set to none but it is what you would use to specify the direction of controlled rotation
        self.interior=False
        #*This defualts to false and is origanally used to determine if the edge crosses a ridge or rib of the part
        
    def total_angle_change(self):
        #*This calculates and returns the total angle change of the edge, based on the last joint- if the joint angles are calculated
        total_angle_change=self.angles[-1][5]-self.angles[0][5]
        self.angle_change=total_angle_change
        return total_angle_change

    def Path_order(self,desired_start,desired_end):
        #*This is used to degine the defualt order of the edge i.e. from node i to node j. This is later useful in flipping the edges in order to make sure the end of one edge leads into the start of the other
        self.PathDirection=[desired_start,desired_end]

    def flip_path(self):
        #* Related to path order this reverses the order of the path from node i -> j to j -> i
        self.PathDirection.reverse()
        # print('FLIPPED')
        self.controlpoints=np.flip(self.controlpoints,0)
        vect_store=deepcopy(self.normal1)
        self.orientation_vectors.reverse()
        self.normal1=deepcopy(self.normal2)
        self.normal2=vect_store
        self.Path.reverse()
        self.angles=np.flip(self.angles,0)

    def flip_rotation_ee(self):
        #*This does not flip the order of the path but the direction the end effector turns to reach the desired ending orientation
        if self.angle_change>0:
            self.make_orientation_vectors(control_rotation=True,clockwise=False)
        elif self.angle_change<0:
            self.make_orientation_vectors(control_rotation=True,clockwise=True)

    def make_smooth_line(self,Smoothness_Coefficient=None,Interpolation_Resolution=None,Weight_Ratio=None):
        #*This utilizes the Scipy spline object to generate a function following the provided control points
        #* The Smoothness Coefficient Represents how much the spline is willing the deviat from control points to acheive a smooth line
        #* too high will result in breaking through the mesh surface
        #* The interpolation Resolution is the number of points you would like in the path array
        #* The weight Ratio is how much the Node points are weighted- i.e. how much it tries to minimize those poits errors- compared to the regular points
        if Smoothness_Coefficient==None:
            Smoothness_Coefficient=50
        if Interpolation_Resolution==None:
            Interpolation_Resolution=50
        # if Weight_Ratio==None:
        #     Weight_Ratio=0.01
        self.Interpolation_Resolution=Interpolation_Resolution
        weights=[]
        #*so this works by utilizing parametric splines so lets make arrays of each
        x=self.controlpoints[:,0]
        y=self.controlpoints[:,1]
        z=self.controlpoints[:,2]
        #*We use t to map each of these equations as x(t),y(t),z(t)
        t=np.linspace(0,len(z)-1,len(z))

        #*We set the weights very high at the first and last points for precision and as 1 every where else
        weights=[1]*len(self.controlpoints)
        weights[0]=1000
        weights[-1]=1000
        
        # print(x)
        # print(t)
        #*Here we create each of the splines
        splx=scipy.interpolate.make_smoothing_spline(t, x, w=weights, lam=Smoothness_Coefficient)
        sply=scipy.interpolate.make_smoothing_spline(t, y, w=weights, lam=Smoothness_Coefficient)
        splz=scipy.interpolate.make_smoothing_spline(t, z, w=weights, lam=Smoothness_Coefficient)
        
        
        #Save the parametric equations so that the derivates can be utilized in the controller algorithm
        #Probably something like this https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.UnivariateSpline.derivatives.html

        #*Here we create an interpolation array with the resolution passed in
        t_interpolate=np.linspace(0,len(z)-1,self.Interpolation_Resolution)
        splx_return=splx(t_interpolate)
        sply_return=sply(t_interpolate)
        splz_return=splz(t_interpolate)
        #*This calls each parametric spline with the interpolation array and returns its output x,y,z
        Path_return=[(splx_return[i], sply_return[i], splz_return[i]) for i in range(0, len(t_interpolate))]
        #*Store these in a path return array
        Path_Distance=0
        #*Calculate the total path distance in cartesian
        for n in range(len(Path_return)-1):
            Path_Distance+=scipy.spatial.distance.euclidean(Path_return[n],Path_return[n+1])


        #*These parametric equations are then themselves stored so they can  be called later        
        self.Parametric_Equations=[splx,sply,splz,t]
        self.Path_Distance=Path_Distance
        self.Path_Distance_Direct=np.linalg.norm(np.array(Path_return[-1])-np.array(Path_return[0]))
        #*This is the direct path distance from the start to the end useful later in the program
        self.Path=Path_return
    
    def add_edge_end_point(self,edge):
        #*This you can add a end point to this edge, it is used to create a more precise tour later by adding each next edges start to each last eages end- although this really shouldn't be neccessary at all
        self.normal2=edge.normal1
        self.controlpoints[-1,:]=edge.controlpoints[0,:]
        self.Path.append(edge.Path[0])
        self.orientation_vectors.append(edge.orientation_vectors[-1])
        self.orientation_thetas.append(edge.orientation_thetas[-1])
        self.Interpolation_Resolution+=1
        self.angles=list(self.angles)
        self.angles.append(edge.angles[0,:])
        self.angles=np.array(self.angles)
        self.Path_Distance+=scipy.spatial.distance.euclidean(self.Path[-2],self.Path[-1])
        delta_angles=np.array(self.angles[-1])-np.array(self.angles[-2])
        delta_angles=np.dot(delta_angles,np.array([1,1,1,1,1,1]))
        self.Cost_Function+=self.Cost_Function+np.sqrt(np.dot(delta_angles,delta_angles))



    def compute_joint_distances(self,IK_object,seed_state=None):
        #*This function walks accross the calculated path and calulates each IK solution for each point and orientation and stores them in a angles array
        
        if self.orientation_vectors==None:
            #*We need the orientation of the tool at each path point
            self.make_orientation_vectors()

        angles=[]
        New_Angle_Cost_Function=0
        bool=True

        if type(seed_state)==type(None):
            #*Check if a seed state is provided or not
            bool=False
            #*If the seed state is not provided defualt it to this
            seed_state=IK_object.move_group.get_current_joint_values()
        
        for n in range(len(self.Path)):
            angle=IK_object.get_IK(self.Path[n],self.orientation_vectors[n],seed_state)
            #*query the ik solver for each point and orientation
            trial_limit=10
            c=0
            while angle==None:
                #*If the ik solver fails to get the angle solution the first time we will give it 10 more tries
                angle=IK_object.get_IK(self.Path[n],self.orientation_vectors[n],seed_state)
                c+=1
                if c==trial_limit:
                    print("Warning no angles found for a path point")
                    exit()
            
            else:
                if math.isclose(abs(angle[4]),abs(np.pi/2),abs_tol=10**-1):
                    #*This entire program this joint is expected to be a multiple of 90 degrees yet sometimes its like 89.99999, annoying- so this checks for that and fixes it
                    angle[4]=abs(np.pi/2)*np.sign(angle[4])

                if n>0 or bool:
                    #*If we have already calculates one of the angles or we are given a seed state we check if the angles are flipped into another configuration than the last angles or seed state
                    #TODO: This is neccesarry to get the correct joint distance as 360 flips in the end effector or any other joint will create errors, but we also do similar things in proccess solution in the TSP class, is there ever a better way to do this or improve the efficency of the code
                    for k in range(len(angle)):
                        angle_flipped=angle[k]-np.pi*2*np.sign(angle[k])
                        #*Compares the flipped angles
                        if (abs(seed_state[k]-angle[k])>abs(seed_state[k]-angle_flipped)) and abs(angle_flipped)<=2*np.pi:
                            angle[k]=angle_flipped
                    delta_angles=np.array(angle)-np.array(seed_state)
                    delta_angles=np.dot(delta_angles,np.array([1,1,1,1,1,1]))
                    #*Calculates the delta angles dotted with itself and then the square root of this as the cost between any two ik solutions for points along the line
                    New_Angle_Cost_Function=New_Angle_Cost_Function+np.sqrt(np.dot(delta_angles,delta_angles))
            angles.append(angle)
            seed_state=angle

        self.angles=np.array(angles)
        #*Store the angle solutions and the cost
        # delta_angles=np.array(angles[0])-np.array(angles[-1])
        self.Cost_Function=New_Angle_Cost_Function

    def make_orientation_vectors(self,control_rotation=False,clockwise=None):
        #*This creates the orientation vectors 
        if not(clockwise==None):
            self.clockwise=clockwise
            #*Save the direction of turn if the defualt it overwritten
        self.control_rotation=control_rotation
        #*Store wether or not defualt rotation is overwritten
        orientation_vectors,thetas,total_orientation_change=\
            angle_linespace(tuple(self.normal1*-1),tuple(self.normal2*-1),self.Interpolation_Resolution,control_rotation,clockwise)
        #*Get the spaced out orientations from the angle linespace function
        self.orientation_thetas=thetas
        self.total_orientation_change=total_orientation_change
        self.orientation_vectors=orientation_vectors

class dwell_edge(edge_line):
    #*This is a child class of the edge_class it inherits everything the same except for what is overwritten. It is a dwell utilized in making the total trajectory in the joint angle bounds of +/- 360 degrees
    def __init__(self,control_points,normal1,normal2):
        super().__init__(control_points,normal1,normal2)
        self.type='dwell'
        #*Set the edge type to dwell
        self.Rotatable=True
        #*It is rotatable

    def make_smooth_line(self,Smoothness_Coefficient=None,Interpolation_Resolution=None,Weight_Ratio=None):
        #*Very similar to above but we divide up the path angles into lift up, rotate, and drop down
        if not(Interpolation_Resolution==None):
            Resolution=2*round((Interpolation_Resolution/3)-1/3)
            self.reset_resolution=round((Interpolation_Resolution/3)+1/3)
        super().make_smooth_line(Smoothness_Coefficient,Interpolation_Resolution=Resolution,Weight_Ratio=None)
    
    def flip_rotation_ee(self):
        angles=deepcopy(list(self.angles))
        # print(angles)
        angles.reverse()
        # print(angles)
        self.angles=np.array(angles)
        self.sign=self.sign*-1


    def compute_joint_distances(self,IK_object,sign=None,seed_state=None):
        #*Very similar to the previous compute_joint_distances but it interjects the correct rotation angles into the middle of the path
        if self.orientation_vectors==None:
            self.make_orientation_vectors()

        if sign==None:
            sign=self.sign
        else:
            self.sign=sign
        
        angles=[]
        New_Angle_Cost_Function=0
        bool=True

        if type(seed_state)==type(None):
            bool=False
            seed_state=[0]*IK_object.IK_SOLVER.number_of_joints
        
        for n in range(0,int(round(len(self.Path)/2))):
            #*see this loops only through half the path
            angle=IK_object.get_IK(self.Path[n],self.orientation_vectors[n],seed_state)
            trial_limit=10
            c=0
            while angle==None:
                
                angle=IK_object.get_IK(self.Path[n],self.orientation_vectors[n],seed_state)
                c+=1
                if c==trial_limit:
                    print("Warning no angles found for a path point")
                    exit()

            else:
                if math.isclose(abs(angle[4]),abs(np.pi/2),abs_tol=10**-1):
                    angle[4]=abs(np.pi/2)*np.sign(angle[4])
                if n>0 or bool:
                    for k in range(len(angle)):
                        angle_flipped=angle[k]-np.pi*2*np.sign(angle[k])
                        if (abs(seed_state[k]-angle[k])>abs(seed_state[k]-angle_flipped)) and abs(angle_flipped)<=2*np.pi:
                            angle[k]=angle_flipped
                    delta_angles=np.array(angle)-np.array(seed_state)
                    delta_angles=np.dot(delta_angles,np.array([1,1,1,1,1,1]))
                    New_Angle_Cost_Function=New_Angle_Cost_Function+np.sqrt(np.dot(delta_angles,delta_angles))
            angles.append(angle)
            if angle==[np.inf]*IK_object.IK_SOLVER.number_of_joints:
                seed_state=seed_state
            else:
                seed_state=angle

        ee_flip_array=np.linspace(angle[5],angle[5]+(np.pi*2*sign),self.reset_resolution)
        #*add the flip rotation array
        for n in range(len(ee_flip_array)):
            angle=[seed_state[0],seed_state[1],seed_state[2],seed_state[3],seed_state[4],ee_flip_array[n]]
            angles.append(angle)
        New_Angle_Cost_Function=New_Angle_Cost_Function+np.sqrt((np.pi*2)**2)

        for n in range(int(round(len(self.Path)/2)),len(self.Path)):
            #*add the drop down path
            angle=IK_object.get_IK(self.Path[n],self.orientation_vectors[n],seed_state)
            trial_limit=10
            c=0
            while angle==None:
                
                angle=IK_object.get_IK(self.Path[n],self.orientation_vectors[n],seed_state)
                c+=1
                if c==trial_limit:
                    print("Warning no angles found for a path point")
                    exit()

            else:
                if math.isclose(abs(angle[4]),abs(np.pi/2),abs_tol=10**-1):
                    angle[4]=abs(np.pi/2)*np.sign(angle[4])
                if n>0 or bool:
                    for k in range(len(angle)):
                        angle_flipped=angle[k]-np.pi*2*np.sign(angle[k])
                        if (abs(seed_state[k]-angle[k])>abs(seed_state[k]-angle_flipped)) and abs(angle_flipped)<=2*np.pi:
                            angle[k]=angle_flipped
                    delta_angles=np.array(angle)-np.array(seed_state)
                    delta_angles=np.dot(delta_angles,np.array([1,1,1,1,1,1]))
                    New_Angle_Cost_Function=New_Angle_Cost_Function+np.sqrt(np.dot(delta_angles,delta_angles))
            angles.append(angle)
            if angle==[np.inf]*IK_object.IK_SOLVER.number_of_joints:
                seed_state=seed_state
            else:
                seed_state=angle

        self.angles=np.array(angles)
        # delta_angles=np.array(angles[0])-np.array(angles[-1])
        self.Cost_Function=New_Angle_Cost_Function

class weld_edge_double_corner_turn(edge_line):
    #*This is very similar to the parent class of edge_class but it alters the way the robot tries to track the weld line
    #*This is the main class we are currently using to do the weld lines
    def __init__(self,control_points,normal1,normal2,control_distance):
        #*This class operates diffrenetly in that you feed the weld line in as the control points not the path of the ee, it computes the oath of the ee assuming this path and a required control distance
        #control points are used a little differently in this line
        normal1=normal1/np.linalg.norm(normal1)
        normal2=normal2/np.linalg.norm(normal2)
        self.normal1=normal1
        self.normal2=normal2
        # print('normals')
        # print(normal1)
        # print(normal2)
        # input('Go on?')
        self.controlpoints=deepcopy(control_points)
        self.orientation_vectors=None
        self.Path_Direction=None
        self.pose_default=None
        self.Rotatable=False
        self.repeated=False
        self.control_rotation=False
        self.clockwise=None
        self.interior=False
        self.type='weld'
        #*Clearly set the type to weld
        self.safety_distance=0.05
        self.Rotatable=False
        self.control_distance=control_distance

    def make_smooth_line(self,Smoothness_Coefficient=None,Interpolation_Resolution=None,Weight_Ratio=None):
        if Smoothness_Coefficient==None:
            Smoothness_Coefficient=100
        if Interpolation_Resolution==None: 
            Interpolation_Resolution=50

        # if Weight_Ratio==None:
        #     Weight_Ratio=0.01

        self.Interpolation_Resolution=Interpolation_Resolution
        weights=[]
                
        # print(self.controlpoints)
        

        normal_perp=(self.controlpoints[-1]-self.controlpoints[0])
        #*This is the normal perp to the orientation of the welder along the line^^^
        total_distance=np.linalg.norm(normal_perp)

        #*The number of rotating points in the corners \|/ based on how large the fraction of control distance is over total distance
        num_of_rp=int(np.round(Interpolation_Resolution*(self.safety_distance/total_distance)))
        # if Interpolation_Resolution<2*num_of_rp:
        #     num_of_rp=int(np.floor(Interpolation_Resolution/3))
        #* Wether to turn 'left' or 'right' along the path = sign
        sign=np.sign(np.cross(self.normal1,self.normal2)[-1])
        # print('sign')
        # print(sign)

        #*space out the angles of the ee based on the safety distance
        weld_angles=list(np.linspace(0,np.pi/4*sign,num_of_rp))+[np.pi/4*sign]*(Interpolation_Resolution-2*num_of_rp)+list(np.linspace(np.pi/4*sign,np.pi/2*sign,num_of_rp))
        # weld_angles=[np.pi/4*sign]*(Interpolation_Resolution)
        self.weld_angles=weld_angles
        # input(weld_angles)
        Path_points=[]

        #*Space out the subsections for first corner turn, straight line section and ending corner turn
        offset=normal_perp*num_of_rp/Interpolation_Resolution
        start1=self.controlpoints[0,:]
        end1=self.controlpoints[0,:]+offset
        start3=self.controlpoints[-1,:]-offset
        end3=self.controlpoints[-1,:]
        
        #*use these to create evenly spaced points along the total line
        control_path=euclidean_linspace(start1,end1,num_of_rp)+\
            euclidean_linspace(end1,start3,Interpolation_Resolution-2*num_of_rp)+\
                euclidean_linspace(start3,end3,num_of_rp)
        self.controlpath=control_path
        # print(self.normal1)
        # print(self.normal2)
        # input('Continue?:')
        #*This is the normal perp to the line
        normal_avg=(self.normal1+self.normal2)/2
        normal_perpu=deepcopy(normal_perp/total_distance)

        self.normal_avg=normal_avg
        self.normal_perp=normal_perpu
        self.orientation_vectors=[]

        for n in range(len(control_path)):
            #*Here we are looping through each point in the path
            a=weld_angles[n]
            #*we refrence the weld angles at that point
            A=np.matrix([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0, 0, 1]])
            #*create the associated rotation matrix
            nv=np.matmul(A,self.normal1)
            ov=deepcopy((-1*nv).tolist()[0])
            #*and use it to calculate the orientation vector at that point^
            ov[-1]=0
            ov=ov/np.linalg.norm(ov)
            #*Zero out its vertical component
            self.orientation_vectors.append(ov)
            point=np.array(control_path[n]+nv*self.control_distance)
            #*we use the none zeroed normal vector to move to the point away from the weld line
            Path_points.append(list(point[0]))
            # print(Path_points)
            # input('Continue?')

        #*These path points are then used as before to generate the parametric splines
        # TODO: Maybe some of these methods should be seperated out for better oo programming with the parent class and all the children classes?
        Path_points=np.array(Path_points)
        # print('PATH POINTS')
        # print(Path_points)

        x=Path_points[:,0]
        y=Path_points[:,1]
        z=Path_points[:,2]
        
        t=np.linspace(0,len(z)-1,len(z))

        weights=[1]*len(z)
        weights[0]=1000
        weights[-1]=1000
        
        # print(len(x))
        # print(len(t))
        # print(len(weights))

        splx=scipy.interpolate.make_smoothing_spline(t, x, w=weights, lam=Smoothness_Coefficient)
        sply=scipy.interpolate.make_smoothing_spline(t, y, w=weights, lam=Smoothness_Coefficient)
        splz=scipy.interpolate.make_smoothing_spline(t, z, w=weights, lam=Smoothness_Coefficient)
        
        #Save the parametric equations so that the derivates can be utilized in the controller algorithm
        #Probably something like this https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.UnivariateSpline.derivatives.html
        
        t_interpolate=np.linspace(0,len(z)-1,self.Interpolation_Resolution)
        splx_return=splx(t_interpolate)
        sply_return=sply(t_interpolate)
        splz_return=splz(t_interpolate)
        Path_return=[(splx_return[i], sply_return[i], splz_return[i]) for i in range(0, len(t_interpolate))]
        
        Path_Distance=0

        for n in range(len(Path_return)-1):
            Path_Distance+=scipy.spatial.distance.euclidean(Path_return[n],Path_return[n+1])

        self.Parametric_Equations=[splx,sply,splz,t]
        self.Path_Distance=Path_Distance
        self.Path_Distance_Direct=np.linalg.norm(np.array(Path_return[-1])-np.array(Path_return[0]))
        self.Path=Path_return

    def compute_joint_distances(self,IK_object,seed_state=None):
        #modify this now
        #TODO: I dont think this is any diffrenent than the parents method of the same name so why am I modifying this-- verify later
        if self.orientation_vectors==None:
            self.make_orientation_vectors()

        angles=[]
        New_Angle_Cost_Function=0
        bool=True

        if type(seed_state)==type(None):
            bool=False
            seed_state=[0]*IK_object.IK_SOLVER.number_of_joints
        
        for n in range(len(self.Path)):
            
            angle=IK_object.get_IK(self.Path[n],self.orientation_vectors[n],seed_state)
            
            trial_limit=10
            c=0
            while angle==None:
                
                angle=IK_object.get_IK(self.Path[n],self.orientation_vectors[n],seed_state)
                c+=1
                if c==trial_limit:
                    print("Warning no angles found for a path point")
                    exit()
            
            else:
                # if np.round((angle[4]/np.pi/2)%1)==0:
                #     angle[4]=(np.pi/2)*np.round((angle[4]/(np.pi/2)))*np.sign(angle[4])

                if n>0 or bool:
                    for k in range(len(angle)):
                        angle_flipped=angle[k]-np.pi*2*np.sign(angle[k])
                        if (abs(seed_state[k]-angle[k])>abs(seed_state[k]-angle_flipped)) and abs(angle_flipped)<=2*np.pi:
                            angle[k]=angle_flipped
                    delta_angles=np.array(angle)-np.array(seed_state)
                    delta_angles=np.dot(delta_angles,np.array([1,1,1,1,1,1]))
                    New_Angle_Cost_Function=New_Angle_Cost_Function+np.sqrt(np.dot(delta_angles,delta_angles))
            angles.append(angle)
            if angle==[np.inf]*IK_object.IK_SOLVER.number_of_joints:
                seed_state=seed_state
            else:
                seed_state=angle

        self.angles=np.array(angles)
        # delta_angles=np.array(angles[0])-np.array(angles[-1])
        self.Cost_Function=New_Angle_Cost_Function

    def make_orientation_vectors(self,control_rotation=False,clockwise=None):
        #* Since in this child class the orientation vectors are basically calculated in make_smooth_line this had to be overwritten
        if not(clockwise==None):
            self.clockwise=clockwise
        self.control_rotation=control_rotation        
        self.orientation_thetas=self.weld_angles
        self.orientation_vectors=[]
        for n in range(len(self.controlpath)):
            #*it used the same methodolgy as above
            a=self.weld_angles[n]
            A=np.matrix([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0, 0, 1]])
            ov=((np.matmul(A,self.normal1))*-1).tolist()[0]
            # print(ov)
            # input('continue?')
            # ov=deepcopy((-1*nv).tolist()[0])
            ov[-1]=0
            # print(ov)
            # input('continue?')
            ov=ov/np.linalg.norm(ov)
            self.orientation_vectors.append(ov)

class weld_edge_double_corner_turn_push(edge_line):
    #*This is very similar to the parent class of edge_class but it alters the way the robot tries to track the weld line
    #*This is the main class we are currently using to do the weld lines
    def __init__(self,control_points,normal1,normal2,control_distance):
        #*This class operates diffrenetly in that you feed the weld line in as the control points not the path of the ee, it computes the oath of the ee assuming this path and a required control distance
        #control points are used a little differently in this line
        normal1=normal1/np.linalg.norm(normal1)
        normal2=normal2/np.linalg.norm(normal2)
        self.normal1=normal1
        self.normal2=normal2
        self.controlpoints=deepcopy(control_points)
        self.orientation_vectors=None
        self.Path_Direction=None
        self.pose_default=None
        self.Rotatable=False
        self.repeated=False
        self.control_rotation=False
        self.clockwise=None
        self.interior=False
        self.type='weld'
        #*Clearly set the type to weld
        self.safety_distance=0.03*2.4555
        self.Rotatable=False
        self.control_distance=control_distance

    def make_smooth_line(self,Smoothness_Coefficient=None,Interpolation_Resolution=None,Weight_Ratio=None):
        if Smoothness_Coefficient==None:
            Smoothness_Coefficient=50
        if Interpolation_Resolution==None: 
            Interpolation_Resolution=50

        # if Weight_Ratio==None:
        #     Weight_Ratio=0.01

        self.Interpolation_Resolution=Interpolation_Resolution
        weights=[]
                
        # print(self.controlpoints)
        

        normal_perp=(self.controlpoints[-1]-self.controlpoints[0])
        #*This is the normal perp to the orientation of the welder along the line^^^
        total_distance=np.linalg.norm(normal_perp)

        #*The number of rotating points in the corners \|/ based on how large the fraction of control distance is over total distance
        num_of_rp=int(np.ceil(Interpolation_Resolution*(self.safety_distance/total_distance)))

        #* Wether to turn 'left' or 'right' along the path = sign
        sign=np.sign(np.cross(self.normal1,self.normal2)[-1])
        # print('sign')
        # print(sign)

        #*space out the angles of the ee based on the safety distance
        weld_angles=list(np.linspace(0,np.pi/2*sign,num_of_rp))+[np.pi/2*sign]*(Interpolation_Resolution-num_of_rp)
        # weld_angles=[np.pi/4*sign]*(Interpolation_Resolution)
        self.weld_angles=weld_angles
        Path_points=[]

        #*Space out the subsections for first corner turn, straight line section and ending corner turn
        offset=normal_perp*num_of_rp/Interpolation_Resolution
        start1=self.controlpoints[0,:]
        end1=self.controlpoints[0,:]+offset
        start3=self.controlpoints[-1,:]-offset
        end3=self.controlpoints[-1,:]
        
        #*use these to create evenly spaced points along the total line
        control_path=euclidean_linspace(start1,end1,num_of_rp)+\
            euclidean_linspace(end1,start3,Interpolation_Resolution-2*num_of_rp)+\
                euclidean_linspace(start3,end3,num_of_rp)
        self.controlpath=control_path
        # print(self.normal1)
        # print(self.normal2)
        # input('Continue?:')
        #*This is the normal perp to the line
        normal_avg=(self.normal1+self.normal2)/2
        normal_perpu=deepcopy(normal_perp/total_distance)

        self.normal_avg=normal_avg
        self.normal_perp=normal_perpu
        self.orientation_vectors=[]

        for n in range(len(control_path)):
            #*Here we are looping through each point in the path
            a=weld_angles[n]
            #*we refrence the weld angles at that point
            A=np.matrix([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0, 0, 1]])
            #*create the associated rotation matrix
            nv=np.matmul(A,self.normal1)
            ov=deepcopy((-1*nv).tolist()[0])
            #*and use it to calculate the orientation vector at that point^
            ov[-1]=0
            ov=ov/np.linalg.norm(ov)
            #*Zero out its vertical component
            self.orientation_vectors.append(ov)
            point=np.array(control_path[n]+nv*self.control_distance)
            #*we use the none zeroed normal vector to move to the point away from the weld line
            Path_points.append(list(point[0]))
            # print(Path_points)
            # input('Continue?')

        #*These path points are then used as before to generate the parametric splines
        # TODO: Maybe some of these methods should be seperated out for better oo programming with the parent class and all the children classes?
        Path_points=np.array(Path_points)
        # print('PATH POINTS')
        # print(Path_points)

        x=Path_points[:,0]
        y=Path_points[:,1]
        z=Path_points[:,2]
        
        t=np.linspace(0,len(z)-1,len(z))

        weights=[1]*len(z)
        weights[0]=1000
        weights[-1]=1000
        
        # print(len(x))
        # print(len(t))
        # print(len(weights))

        splx=scipy.interpolate.make_smoothing_spline(t, x, w=weights, lam=Smoothness_Coefficient)
        sply=scipy.interpolate.make_smoothing_spline(t, y, w=weights, lam=Smoothness_Coefficient)
        splz=scipy.interpolate.make_smoothing_spline(t, z, w=weights, lam=Smoothness_Coefficient)
        
        #Save the parametric equations so that the derivates can be utilized in the controller algorithm
        #Probably something like this https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.UnivariateSpline.derivatives.html
        
        t_interpolate=np.linspace(0,len(z)-1,self.Interpolation_Resolution)
        splx_return=splx(t_interpolate)
        sply_return=sply(t_interpolate)
        splz_return=splz(t_interpolate)
        Path_return=[(splx_return[i], sply_return[i], splz_return[i]) for i in range(0, len(t_interpolate))]
        
        Path_Distance=0

        for n in range(len(Path_return)-1):
            Path_Distance+=scipy.spatial.distance.euclidean(Path_return[n],Path_return[n+1])

        self.Parametric_Equations=[splx,sply,splz,t]
        self.Path_Distance=Path_Distance
        self.Path_Distance_Direct=np.linalg.norm(np.array(Path_return[-1])-np.array(Path_return[0]))
        self.Path=Path_return

    def compute_joint_distances(self,IK_object,seed_state=None):
        #modify this now
        #TODO: I dont think this is any diffrenent than the parents method of the same name so why am I modifying this-- verify later
        if self.orientation_vectors==None:
            self.make_orientation_vectors()

        angles=[]
        New_Angle_Cost_Function=0
        bool=True

        if type(seed_state)==type(None):
            bool=False
            seed_state=[0]*IK_object.IK_SOLVER.number_of_joints
        
        for n in range(len(self.Path)):
            
            angle=IK_object.get_IK(self.Path[n],self.orientation_vectors[n],seed_state)
            
            trial_limit=10
            c=0
            while angle==None:
                
                angle=IK_object.get_IK(self.Path[n],self.orientation_vectors[n],seed_state)
                c+=1
                if c==trial_limit:
                    print("Warning no angles found for a path point")
                    exit()
            
            else:
                if np.round((angle[4]/np.pi/2)%1)==0:
                    angle[4]=(np.pi/2)*np.round((angle[4]/(np.pi/2)))*np.sign(angle[4])

                if n>0 or bool:
                    for k in range(len(angle)):
                        angle_flipped=angle[k]-np.pi*2*np.sign(angle[k])
                        if (abs(seed_state[k]-angle[k])>abs(seed_state[k]-angle_flipped)) and abs(angle_flipped)<=2*np.pi:
                            angle[k]=angle_flipped
                    delta_angles=np.array(angle)-np.array(seed_state)
                    delta_angles=np.dot(delta_angles,np.array([1,1,1,1,1,1]))
                    New_Angle_Cost_Function=New_Angle_Cost_Function+np.sqrt(np.dot(delta_angles,delta_angles))
            angles.append(angle)
            if angle==[np.inf]*IK_object.IK_SOLVER.number_of_joints:
                seed_state=seed_state
            else:
                seed_state=angle

        self.angles=np.array(angles)
        # delta_angles=np.array(angles[0])-np.array(angles[-1])
        self.Cost_Function=New_Angle_Cost_Function

    def make_orientation_vectors(self,control_rotation=False,clockwise=None):
        #* Since in this child class the orientation vectors are basically calculated in make_smooth_line this had to be overwritten
        if not(clockwise==None):
            self.clockwise=clockwise
        self.control_rotation=control_rotation        
        self.orientation_thetas=self.weld_angles
        self.orientation_vectors=[]
        for n in range(len(self.controlpath)):
            #*it used the same methodolgy as above
            a=self.weld_angles[n]
            A=np.matrix([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0, 0, 1]])
            ov=((np.matmul(A,self.normal1))*-1).tolist()[0]
            # print(ov)
            # input('continue?')
            # ov=deepcopy((-1*nv).tolist()[0])
            ov[-1]=0
            # print(ov)
            # input('continue?')
            ov=ov/np.linalg.norm(ov)
            self.orientation_vectors.append(ov)
        
class weld_edge_double_corner_turn_oscilations(edge_line):
    #*Adds oscilations
    def __init__(self,control_points,normal1,normal2,control_distance):
        #control points are used a little differently in this line
        self.OPL=(40/6)*50

        normal1=normal1/np.linalg.norm(normal1)
        normal2=normal2/np.linalg.norm(normal2)
        self.normal1=normal1
        self.normal2=normal2
        self.controlpoints=deepcopy(control_points)
        self.orientation_vectors=None
        self.Path_Direction=None
        self.pose_default=None
        self.Rotatable=False
        self.repeated=False
        self.control_rotation=False
        self.clockwise=None
        self.interior=False
        self.type='weld_edge_double_corner_turn'
        self.type='weld'
        self.safety_distance=0.030
        self.oscd=0.03
        self.Rotatable=False
        self.control_distance=control_distance

    def make_smooth_line(self,Smoothness_Coefficient=None,Interpolation_Resolution=None,Weight_Ratio=None):
        if Smoothness_Coefficient==None:
            Smoothness_Coefficient=50
        if Interpolation_Resolution==None: 
            Interpolation_Resolution=50

        # if Weight_Ratio==None:
        #     Weight_Ratio=0.01

        self.Interpolation_Resolution=Interpolation_Resolution
        weights=[]
                
        # print(self.controlpoints)
        

        normal_perp=(self.controlpoints[-1]-self.controlpoints[0])
        # print(normal_perp)
        # input('continue')
        total_distance=np.linalg.norm(normal_perp)
        nO=total_distance*self.OPL
        #*nO is the number of osilations through the entire pass and is based on OPL or oscilations per length
        
        num_of_rp=int(np.ceil(Interpolation_Resolution*(self.safety_distance/total_distance)))
        # if self.orientation_vectors==None:
        #     self.make_orientation_vectors()
        sign=np.sign(np.cross(self.normal1,self.normal2)[-1])
        print('sign')
        print(sign)

        weld_angles=list(np.linspace(0,np.pi/4*sign,num_of_rp))+[np.pi/4*sign]*(Interpolation_Resolution-2*num_of_rp)+list(np.linspace(np.pi/4*sign,np.pi/2*sign,num_of_rp))
        self.weld_angles=weld_angles
        Path_points=[]
        offset=normal_perp*num_of_rp/Interpolation_Resolution
        start1=self.controlpoints[0,:]
        end1=self.controlpoints[0,:]+offset
        start3=self.controlpoints[-1,:]-offset
        end3=self.controlpoints[-1,:]
        
        control_path=euclidean_linspace(start1,end1,num_of_rp)+\
            euclidean_linspace(end1,start3,Interpolation_Resolution-2*num_of_rp)+\
                euclidean_linspace(start3,end3,num_of_rp)
        self.controlpath=control_path
        print(self.normal1)
        print(self.normal2)
        # input('Continue?:')
        normal_avg=(self.normal1+self.normal2)/2
        normal_avg[-1]=np.linalg.norm(normal_avg[0:2])
        normal_avg=normal_avg/np.linalg.norm(normal_avg)
        normal_perpu=deepcopy(normal_perp/total_distance)
        self.normal_avg=normal_avg
        self.normal_perp=normal_perpu
        self.orientation_vectors=[]

        #* THis is how I add the oscilations\|/
        oT=np.linspace(0,nO*2*np.pi,Interpolation_Resolution)
        scurve=s_curve(b=3,c=1,e=4/6)
        #*I use this curve to ramp into and out of the oscilations along the line it is a sigmound function
        tx=list(np.linspace(0,0.5,num_of_rp))+[0.5]*(Interpolation_Resolution-2*num_of_rp)+list(np.linspace(0.5,1,num_of_rp))
        
        S=np.matrix([[1,0],[0,0.075]])
        #*This is the scalling array of the oscilations
        
        normal3=np.cross(normal_avg,normal_perpu)
        #*This is the normal utilized with the perp normal to add the oscilations
        vect_M=np.matrix([normal_perp,normal3])
        ang=(0)*np.sign(normal3[-1])*-1
        R=np.matrix([[np.cos(ang),-np.sin(ang)],[np.sin(ang),np.cos(ang)]])
        #*This is an additional rotation matrix for the vector of the oscilations currently its 0 rotation in the plane perp to teh weld normal
        T=np.array(np.matmul(R,S))
        #*Multiply the rotation and the scaling array

        for n in range(len(control_path)):
            a=weld_angles[n]
            scaling=scurve.ds(tx[n])
            #*This is the scalling term which scales how large the oscialtions are
            if n==0 or n==len(control_path)-1:
                scaling=0
                #*artifically set the first and last oscialtions to zero even though they are really close to this 
            osA=oT[n]
            #*Oscilation angle
            osR=np.array([np.cos(osA),np.sin(osA)])*((1/64)+(2)*np.sin(0.5*osA)**6)
            #*vector calculated to discpace the straight line path, the second scalar at the end scales it to be larger changes in the positive x direction
            osR=np.array(np.matmul(T,osR)).flatten()
            #*pass this rotation vector through the transformation
            osV=np.array(np.matmul(osR,vect_M)).flatten()
            osV=osV*scaling
            A=np.matrix([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0, 0, 1]])
            nv=np.matmul(A,self.normal1)
            # print(nv)
            ov=deepcopy((-1*nv).tolist()[0])
            # print(ov)
            # input('go on?')
            ov[-1]=0
            ov=ov/np.linalg.norm(ov)
            self.orientation_vectors.append(ov)
            point=np.array(control_path[n]+nv*self.control_distance+osV*self.oscd)
            #*calculated the point and add the oscilation offset the rest of this matches or is very similar to above
            Path_points.append(list(point[0]))
            # print(Path_points)
            # input('Continue?')
        Path_points=np.array(Path_points)
        # print('PATH POINTS')
        # print(Path_points)

        x=Path_points[:,0]
        y=Path_points[:,1]
        z=Path_points[:,2]
        
        t=np.linspace(0,len(z)-1,len(z))

        weights=[1]*len(z)
        weights[0]=1000
        weights[-1]=1000
        
        # print(len(x))
        # print(len(t))
        # print(len(weights))

        splx=scipy.interpolate.make_smoothing_spline(t, x, w=weights, lam=Smoothness_Coefficient)
        sply=scipy.interpolate.make_smoothing_spline(t, y, w=weights, lam=Smoothness_Coefficient)
        splz=scipy.interpolate.make_smoothing_spline(t, z, w=weights, lam=Smoothness_Coefficient)
        
        #Save the parametric equations so that the derivates can be utilized in the controller algorithm
        #Probably something like this https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.UnivariateSpline.derivatives.html
        
        t_interpolate=np.linspace(0,len(z)-1,self.Interpolation_Resolution)
        splx_return=splx(t_interpolate)
        sply_return=sply(t_interpolate)
        splz_return=splz(t_interpolate)
        Path_return=[(splx_return[i], sply_return[i], splz_return[i]) for i in range(0, len(t_interpolate))]
        
        Path_Distance=0

        for n in range(len(Path_return)-1):
            Path_Distance+=scipy.spatial.distance.euclidean(Path_return[n],Path_return[n+1])

        self.Parametric_Equations=[splx,sply,splz,t]
        self.Path_Distance=Path_Distance
        self.Path_Distance_Direct=np.linalg.norm(np.array(Path_return[-1])-np.array(Path_return[0]))
        self.Path=Path_return

    def compute_joint_distances(self,IK_object,seed_state=None):
        #modify this now

        if self.orientation_vectors==None:
            self.make_orientation_vectors()

        angles=[]
        New_Angle_Cost_Function=0
        bool=True

        if type(seed_state)==type(None):
            bool=False
            seed_state=[0]*IK_object.IK_SOLVER.number_of_joints
        
        for n in range(len(self.Path)):
            
            angle=IK_object.get_IK(self.Path[n],self.orientation_vectors[n],seed_state)
            
            trial_limit=10
            c=0
            while angle==None:
                
                angle=IK_object.get_IK(self.Path[n],self.orientation_vectors[n],seed_state)
                c+=1
                if c==trial_limit:
                    print("Warning no angles found for a path point")
                    exit()
            
            else:
                if math.isclose(abs(angle[4]),abs(np.pi/2),abs_tol=10**-1):
                    angle[4]=abs(np.pi/2)*np.sign(angle[4])

                if n>0 or bool:
                    for k in range(len(angle)):
                        angle_flipped=angle[k]-np.pi*2*np.sign(angle[k])
                        if (abs(seed_state[k]-angle[k])>abs(seed_state[k]-angle_flipped)) and abs(angle_flipped)<=2*np.pi:
                            angle[k]=angle_flipped
                    delta_angles=np.array(angle)-np.array(seed_state)
                    delta_angles=np.dot(delta_angles,np.array([1,1,1,1,1,1]))
                    New_Angle_Cost_Function=New_Angle_Cost_Function+np.sqrt(np.dot(delta_angles,delta_angles))
            angles.append(angle)
            if angle==[np.inf]*IK_object.IK_SOLVER.number_of_joints:
                seed_state=seed_state
            else:
                seed_state=angle

        self.angles=np.array(angles)
        # delta_angles=np.array(angles[0])-np.array(angles[-1])
        self.Cost_Function=New_Angle_Cost_Function

    def make_orientation_vectors(self,control_rotation=False,clockwise=None):
        if not(clockwise==None):
            self.clockwise=clockwise
        self.control_rotation=control_rotation        
        self.orientation_thetas=self.weld_angles
        self.orientation_vectors=[]
        for n in range(len(self.controlpath)):
            a=self.weld_angles[n]
            A=np.matrix([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0, 0, 1]])
            ov=((np.matmul(A,self.normal1))*-1).tolist()[0]
            # print(ov)
            # input('continue?')
            # ov=deepcopy((-1*nv).tolist()[0])
            ov[-1]=0
            # print(ov)
            # input('continue?')
            ov=ov/np.linalg.norm(ov)
            self.orientation_vectors.append(ov)