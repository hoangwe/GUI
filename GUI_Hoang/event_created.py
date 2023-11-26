import numpy as np
from Main import MainWindow
import serial
from Robot_function import*
from Main import*


class guiEvent(MainWindow):
    def connectInitial(self, port, baudrate):
        self.ui.ser.port = port
        self.ui.ser.baudrate = baudrate
        self.ui.ser.bytesize = serial.EIGHTBITS  # number of bits per bytes
        self.ui.ser.parity = serial.PARITY_NONE  # set parity check: no parity
        self.ui.ser.stopbits = serial.STOPBITS_ONE  # number of stop bits
        self.ui.ser.xonxoff = False  # disable software flow control
        self.ui.ser.rtscts = False  # disable hardware (RTS/CTS) flow control
        self.ui.ser.dsrdtr = False  # disable hardware (DSR/DTR) flow control
        self.ui.ser.writeTimeout = 0  # timeout for write
        self.ui.ser.timeout = 10
        self.ui.ser.open()

    def connectFunction(self):
        if (
            self.ui.port.currentText() != "Arduino Port"
            and self.ui.port.currentText() != ""
        ):
            port = self.ui.port.currentText()
            baudrate = self.ui.baud.currentText()
            guiEvent.connectInitial(self, port, baudrate)

            if self.ui.ser.isOpen():
                self.ui.status.setText("Connected to " + 
                self.ui.port.currentText())
                self.ui.status.adjustSize()
#hien thi nut nhan tren trang thai status ( disconect)
    def disconnectFunction(self):
        if self.ui.ser.isOpen():
            self.ui.ser.close()
            self.ui.status.setText("Disconnected")
            self.ui.status.adjustSize()
            self.ui.btn_disconnect.setEnabled(False)
        else:
            self.ui.status.setText("Disconnected")
            self.ui.status.adjustSize()

  ##############################################################3
    # def SendInformation(self,theta):
    #     if self.ui.ser.isOpen():
    #         theta1=float(theta[0])
    #         theta2=float(theta[1])
    #         Z=float(theta[2])
    #         self.the_gui = np.array([int(theta1),int(theta2),Z,self.grip_robot])
    #         self.ui.ser.write('{},{},{},{}'.format(*self.the_gui).encode())
    #         Data_send =str('{},{},{},{}'.format(*self.the_gui))
    #         self.ui.ser.flushInput()  #flush input buffer, discarding all its contents
    #         self.ui.ser.flushOutput()
    #         print(Data_send)
    def SendInformation_signal(self,theta):
        if self.ui.ser.isOpen():
            theta1=float(theta[0])
            theta2=float(theta[1])
            Z=float(theta[2])
            if theta1>90 or theta1<-90 or theta2>90 or theta2<-90:
                self.ui.status.setStyleSheet("color: red")
                self.ui.status.setText("Position Error !")
            else:
                self.ui.status.setStyleSheet("color: black")
                self.ui.status.setText("Connected to " + 
                self.ui.port.currentText())
                self.the_gui = np.array([int(theta1),int(theta2),Z,self.grip_robot])
                self.ui.ser.write('{},{},{},{}'.format(*self.the_gui).encode())
                Data_send =str('{},{},{},{}'.format(*self.the_gui))
                self.ui.ser.flushInput()  #flush input buffer, discarding all its contents
                self.ui.ser.flushOutput()
                print(Data_send)
    def Send_theta(self):
        if self.ui.ser.isOpen():
            theta1=float(self.ui.theta1_control.text())
            theta2=float(self.ui.theta2_control.text())
            Z=float(self.ui.theta_Z_control.text())
            if theta1>90 or theta1<-90 or theta2>90 or theta2<-90:
                self.ui.status.setStyleSheet("color: red")
                self.ui.status.setText("Position Error !")
            else:
                self.ui.status.setStyleSheet("color: black")
                self.ui.status.setText("Connected to " + 
                self.ui.port.currentText())
                self.the_gui = np.array([int(theta1),int(theta2),Z,self.grip_robot])
                self.the_d = np.array([theta1,theta2,Z])
                self.ui.ser.write('{},{},{},{}'.format(*self.the_gui).encode())
                Data_send =str('{},{},{},{}'.format(*self.the_gui))
                self.ui.ser.flushInput()  #flush input buffer, discarding all its contents
                self.ui.ser.flushOutput()
                print(Data_send)
    def SendInformation_theta(self):
        if self.ui.ser.isOpen():
            theta1=float(self.ui.theta1_control.text())
            theta2=float(self.ui.theta2_control.text())
            Z=float(self.ui.theta_Z_control.text())
            if theta1>90 or theta1<-90 or theta2>90 or theta2<-90:
                self.ui.status.setStyleSheet("color: red")
                self.ui.status.setText("Position Error !")
            else:
                self.ui.status.setStyleSheet("color: black")
                self.ui.status.setText("Connected to " + 
                self.ui.port.currentText())
                self.the_gui = np.array([int(theta1),int(theta2),Z,self.grip_robot])
                self.the_d = np.array([theta1,theta2,Z])
                pos=Scara.forwardkinematics(self,self.the_d)
                self.ui.set_x.setText(str(int(pos[0])))
                self.ui.set_y.setText(str(int(pos[1])))
                self.ui.set_z.setText(str(int(pos[2])))
                self.ui.ser.write('{},{},{},{}'.format(*self.the_gui).encode())
                Data_send =str('{},{},{},{}'.format(*self.the_gui))
                self.ui.ser.flushInput()  #flush input buffer, discarding all its contents
                self.ui.ser.flushOutput()
                print(Data_send)
    
#####################################################################
    def Drop_gripper(self) :
        self.grip_robot=0
        the1=float(self.ui.current_theta1.text())
        the2=float(self.ui.current_theta2.text())
        Z=float(self.ui.current_z1.text())
        theta_send=np.array([the1,the2,Z,0])
        self.ui.ser.write('{},{},{},{}'.format(*theta_send).encode())
        Data_send =str('{},{},{},{}'.format(*theta_send))
        self.ui.ser.flushInput()  #flush input buffer, discarding all its contents
        self.ui.ser.flushOutput()
        print(self.grip_robot)
    def Pick_gripper(self) :
        self.grip_robot=1
        the1=float(self.ui.current_theta1.text())
        the2=float(self.ui.current_theta2.text())
        Z=float(self.ui.current_z1.text())
        theta_send=np.array([the1,the2,Z,1])
        self.ui.ser.write('{},{},{},{}'.format(*theta_send).encode())
        Data_send =str('{},{},{},{}'.format(*theta_send))
        self.ui.ser.flushInput()  #flush input buffer, discarding all its contents
        self.ui.ser.flushOutput()
        print(Data_send)
        
#####################################################################

    def generated_plot3D_robor1(self):
    #     """
    #     Function to plot Data from Actual Angle and Duty Cycle Changing on 2D visualization....
    #     """
        if self.ui.ser.isOpen():
            theta1=self.ui.current_theta1.text()
            theta2=self.ui.current_theta2.text()
            z_d=self.ui.current_z1.text()
           
            if  theta1=='' or theta2=='' or z_d== '':
                theta1=0.0
                theta2=0.0
                z_d=0.0
             
            else:
                theta1=self.ui.current_theta1.text()
                theta2=self.ui.current_theta2.text()
                z_d=self.ui.current_z1.text()
                

            the = [theta1,theta2,z_d]
            #print(the)
            x,y,z = Scara.forward_kinematics_draw(self,the)
            self.plot_3d.axes.clear()
            self.plot_3d.config_display(self.plot_3d)
            # line -[link length] plot
            #print(x)
            #print(y)
            #print(z)
            #self.Form_3d_layout.axes.plot([0.0,x[0]],[0.0,y[0]],[-3,z[0]], linewidth=5)
            self.plot_3d.axes.plot([x[0],x[1]],[y[0],y[1]],[z[0],z[1]], linewidth=5)
            self.plot_3d.axes.plot([x[1],x[2]],[y[1],y[2]],[z[1],z[2]],linewidth=5)
            self.plot_3d.axes.plot([x[2],x[3]],[y[2],y[3]],[z[2],z[3]],linewidth=5)
            self.plot_3d.axes.plot([x[3],x[4]],[y[3],y[4]],[z[3],z[4]],linewidth=5)
            # Joints syntaxis plot
            self.plot_3d.axes.scatter(x[0], y[0], z[0], color='red',linewidth=7)
            self.plot_3d.axes.scatter(x[1], y[1], z[1], color='red',linewidth=7)
            self.plot_3d.axes.scatter(x[2], y[2], z[2], color='red',linewidth=7)
            self.plot_3d.axes.scatter(x[3], y[3], z[3], color='red',linewidth=7)
            self.plot_3d.axes.scatter(x[4], y[4], z[4], marker="o" ,color='red',linewidth=6)
            label = "  ({:.1f},{:.1f},{:.1f})".format(x[4], y[4],float(z_d))
            self.plot_3d.axes.text(x[4], y[4], float(z_d), label, fontsize=16, color="red")  
            self.plot_3d.draw()
            self.plot_3d.draw()
    def plot_realtime(self):
         if self.ui.ser.isOpen():
             guiEvent.generated_plot3D_robor1(self)
             #guiEvent.generated_plot_duty(self)

    def up_signal(self,step):
        X=float(self.ui.current_x.text())
        Y=float(self.ui.current_y.text())
        Z=float(self.ui.current_z.text())+step
        self.ui.set_x.setText(str(X))
        self.ui.set_y.setText(str(Y))
        self.ui.set_z.setText(str(Z))
        pos=np.array([X,Y,Z])
        the=Scara.Inverse_Kinematics(self,pos)
        return the
    def dw_signal(self,step):
        X=float(self.ui.current_x.text())
        Y=float(self.ui.current_y.text())
        Z=float(self.ui.current_z.text())-step
        self.ui.set_x.setText(str(X))
        self.ui.set_y.setText(str(Y))
        self.ui.set_z.setText(str(Z))
        pos=np.array([X,Y,Z])
        the=Scara.Inverse_Kinematics(self,pos)
        return the
       





    







  
           



 
