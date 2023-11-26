import math as m
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg.linalg import _tensorinv_dispatcher

# Dinh Nghia cho ham sin cos tan 
c = lambda x : np.cos(x)
s = lambda x : np.sin(x)
# c   = lambda x: np.cos(np.deg2rad(x))
# s   = lambda x: np.sin(np.deg2rad(x))
t = lambda x,y : m.atan2(x,y)


class Scara :
    def forward_kinematics_draw(self,theta):
        t1=np.deg2rad(float(theta[0]))
        t2=np.deg2rad(float(theta[1]))
        z=float(theta[2])
        l1= 220             #(Khop1)
        l2= 170            #(Khop2)
        x_d1 = l1*c(t1)
        x_ee = l1*c(t1) +l2*c(t1+t2)
        
        x_Scara = np.array([0,0,x_d1, x_ee, x_ee])
        ################################################
        y_d1 = l1*s(t1)
        y_ee = l1*s(t1) +l2*s(t1+t2)
        y_Scara = np.array([0, 0,y_d1, y_ee, y_ee])
        ###############################################
        z_ee = z
        z_Scara = np.array([0,110,110,110,z])

        return x_Scara,y_Scara,z_Scara

    #####################################
    ##################################### ( DONG HOC THUAN)
    def forwardkinematics(self,theta):
        theta1=np.deg2rad(float(theta[0]))
        theta2=np.deg2rad(float(theta[1]))
        z=float(theta[2])
        # theta1=float(theta[0]) 
        # theta2=float(theta[1])  
        # Z=float(theta[2])
        ##########################################################
        ##########################################################  
        #l1= 270            #(Khop Tinh Tien Z)
        l1= 220            #(Khop1)
        l2= 170            #(Khop2)
                           
        X = l1*c(theta1)+l2*c(theta1+theta2)
        Y = l1*s(theta1)+l2*s(theta1+theta2)
        Z = z
        pos=np.array([X,Y,Z])
        return pos
    #######################################
    ####################################### ( DONG HOC NGHICH)
    def Inverse_Kinematics(self,pos):
    
    ####################### FIND THE THETA 1
        PX=float(pos[0]) 
        PY=float(pos[1])
        PZ=float(pos[2])
        ##########################################################
        #l1= 270            #(Khop Tinh Tien Z tai vi tri home )
        l1= 220             #(Khop1)
        l2= 170                #(Khop2)

        a = 2*PX*l1
        b = 2*PY*l1
        d = PX*PX + PY*PY +l1*l1 - l2*l2
        anp = t(b,a)
        # USING BASIC 01 FOR SOLVE THETA 1 AND THETA2
        # phuong trinh tong quat:  ####       a1*sin(the)+b1*cos(the)=d1 #######
        the1 = anp + t(np.sqrt(1- d*d/(a*a + b*b)) , d/np.sqrt(a*a + b*b))
        # the2 = anp + t(-np.sqrt(1- d*d/(a*a + b*b)),  d/(np.sqrt(a*a + b*b)))
        the2 = t(PY - l1*s(the1) , PX - l1*c(the1)) - the1 
        theta2_deg=np.rad2deg(the2)
        theta1_deg=np.rad2deg(the1)

        the= np.array([theta1_deg, theta2_deg,PZ])
        ################ tranfer degree 
        return the

        

    

        ##########################################################
        ##########################################################  
        
        ##########################################################
        ##########################################################  





        ##########################################################
        ########################################################## 




