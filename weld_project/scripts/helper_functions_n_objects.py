JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

import numpy as np
#*helper functions
def euclidean_linspace(start,stop,steps):
    #*This basically creates points on a line from start to stop
    x=np.linspace(start[0],stop[0],steps)
    y=np.linspace(start[1],stop[1],steps)
    z=np.linspace(start[2],stop[2],steps)
    points=[]
    for n in range(len(x)):
        points.append(tuple([x[n],y[n],z[n]]))
    return points

def euclidean_logspace(start,stop,steps):
    #*Same as above but logarithmically spaces them
    x=np.logspace(start[0],stop[0],steps)
    y=np.logspace(start[1],stop[1],steps)
    z=np.linspace(start[2],stop[2],steps)
    points=[]
    for n in range(len(x)):
        points.append(tuple([x[n],y[n],z[n]]))
    return points

def angle_linespace(start,stop,steps,control_rotation=False,clockwise=None):
    #* This function returns orientation vectors, angles, and the total angle change for the turning from one vector to another in cartesian space

    #* By defualt it turns in the direction of shortest turn distance and only includes angles between 0 and 2pi, control_rotation overrides this anf clockwise as true or false selects the rotation direction

    angle1=np.arctan2(start[1],start[0])
    angle2=np.arctan2(stop[1],stop[0])
    if angle1<0:
        angle1=(2*np.pi)+angle1

    if angle2<0:
        angle2=(2*np.pi)+angle2

    delta_angle=angle2-angle1


    # if angle1%(np.pi/2)==0:
    #     print(angle1)
    # if angle2%(np.pi/2)==0:
    #     print(angle2)
    if control_rotation==False:
        
        if abs(delta_angle)>np.pi:
            delta_angle=(2*np.pi-abs(delta_angle))*-1*np.sign(delta_angle)

    elif control_rotation==True:
        if clockwise==False:
            if delta_angle<0:
                delta_angle=(2*np.pi-abs(delta_angle))
        elif clockwise==True:
            if delta_angle>0:
                delta_angle=(2*np.pi-abs(delta_angle))*-1

    angle_step=delta_angle/steps
    return_angles=[]
    return_vectors=[]

    for n in range(steps):
        next_angle=angle1+n*angle_step
        return_angles.append(next_angle)
        return_vectors.append((np.cos(next_angle),np.sin(next_angle),0))
    
    return return_vectors,return_angles,delta_angle

def return_z_rotation_and_transformation_matrix(angle,cord=None,just_rotation=False):
    if cord==None:
        cord=[0,0,0]
    rotation=[[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]]
    if just_rotation==True:
        return rotation
    else:
        T=np.eye(4)
        T[0:3,0:3]=rotation
        cord=np.array([[cord[0]],[cord[1]],[cord[2]]])
        T[0:3,3:]=cord
    return T

#* Helper objects
class s_curve:
    #*this is the sigmound curve it initializes with the curves parameters, and is spaced between t =(0,1) the derivatives can also be called
    def __init__(self,b,c,e=1):
        self.e=e
        self.b=b
        self.c=c
    def s(self,t):
        s=self.e*1/(1+np.exp(-self.b*(2*t-self.c)))
        return s
    def ds(self,t):
        ds=self.e*(2*self.b*np.exp(-self.b*(2*t-self.c)))/((1+np.exp(-self.b*(2*t-self.c)))**2)
        return ds
    def dds(self,t):
        term1=self.e*(8*(self.b**2)*np.exp(-2*self.b*(2*t-self.c)))/((np.exp(-self.b*(2*t-self.c))+1)**3)
        term2=self.e*(4*(self.b**2)*np.exp(-self.b*(2*t-self.c)))/((1+np.exp(-self.b*(2*t-self.c)))**2)
        dds=term1-term2
        return dds

