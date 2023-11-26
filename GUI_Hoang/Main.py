import sys
import platform
import serial.tools.list_ports
import serial
import matplotlib
from matplotlib import cm
import time
import pandas as pd
matplotlib.use("Qt5Agg")
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from matplotlib.animation import FuncAnimation

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import (
    QCoreApplication,
    QPropertyAnimation,
    QDate,
    QDateTime,
    QMetaObject,
    QObject,
    QPoint,
    QRect,
    QSize,
    QTime,
    QUrl,
    Qt,
    QEvent,
)
from PyQt5.QtGui import (
    QBrush,
    QColor,
    QConicalGradient,
    QCursor,
    QFont,
    QFontDatabase,
    QIcon,
    QKeySequence,
    QLinearGradient,
    QPalette,
    QPainter,
    QPixmap,
    QRadialGradient,
)
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import pyqtSlot

import warnings

warnings.filterwarnings("ignore")

from app_modules import *


###################################
################################### ( BIEU DO 2D)
class Display_2D(FigureCanvas):
    def __init__(self, parent=None, width=150, height=100, dpi=75):
        figure = Figure(figsize=(width, height), dpi=dpi)
        figure.tight_layout()
        super(Display_2D, self).__init__(figure)
        self.axes = figure.add_subplot(111)

    def config_display_2D(self, widget):
         widget.axes.grid(True)
         widget.axes.tick_params(axis="x", colors="black")
         widget.axes.tick_params(axis="y", colors="black")
###################################
################################### ( BIEU DO 3D)

class Display(FigureCanvas):
    def __init__(self,parent=None, width = 70, height = 50,dpi=75):
        figure = Figure(figsize=(width,height),dpi=dpi)
        figure.patch.set_facecolor('#343b48')
        figure.suptitle('3D Robotics Simulation',color='white',fontsize=15)
        self.axes = figure.gca()
        figure.tight_layout()
        super(Display, self).__init__(figure)
    def config_display(self,widget):
        widget.axes.set_facecolor('#343b48')
        widget.axes.grid(True)
        widget.axes.set_xlim(0,500)
        widget.axes.set_ylim(-500, 500)
        widget.axes.set_zlim(0, 500)

        widget.axes.set_xlabel('X_axis',color='white',fontsize=10)
        widget.axes.set_ylabel('Y_axis',color='white',fontsize=10)
        widget.axes.set_zlabel('Z_axis',color='white',fontsize=10)
        widget.axes.tick_params(axis='x', colors='white')
        widget.axes.tick_params(axis='y', colors='white')
        widget.axes.tick_params(axis='z', colors='white')        
         
################################
#################################

class MainWindow(QMainWindow):
    def __init__(self):
        ## IMPORT MAIN WINDOWN USER INTERFACE ##
        QMainWindow.__init__(self)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.timer = QtCore.QTimer() # khoi tao bien Timer
        self.threadpool = QtCore.QThreadPool()
        self.ui.ser = serial.Serial()
        self.timer = QtCore.QTimer()
        x=297
        y=0
        z=100
        self.grip_robot=0
        self.pos=np.array([x,y,z])
        # theta1,theta2,theta3=Scara.Inverse_Kinematics(self,self.pos)
        # print(theta1)
        # print(theta2)
        # print(theta3)
        ########################################
        self.theta1=[]
        self.theta2=[]
        self.Z=[]
        self.Read_information()
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.Read_information)
        self.timer.timeout.connect(lambda: guiEvent.plot_realtime(self))
        self.timer.start()

        ########## INITIALIZE CONNECTION #######

        ########################################
        uiConfig.Configuration(self)
        ######### INITIALIZE MATPLOT ###########
        self.plot_1 = Display_2D(self)
        self.plot_1.config_display_2D(self.plot_1)
        self.plot_3d = Display(self,width=50, height=50, dpi=70)  
        # dat cac gia tri hien thi chieu dai , chieu rong cho Bieu Do 3D
        self.plot_3d.config_display(self.plot_3d)
        #######################################
        self.ui.layout_2D.addWidget(self.plot_1)
        self.ui.Form_3d_layout.addWidget(self.plot_3d) # bieu do 3d 
        ########### BUTTON EVENT ###############
        self.ui.btnconnect.clicked.connect(lambda: guiEvent.connectFunction(self))
        self.ui.btn_disconnect.clicked.connect(lambda: guiEvent.disconnectFunction(self))
        #################################################################################
        self.ui.pushButton.clicked.connect(lambda: guiEvent.SendInformation_theta(self))
        self.ui.btn_home.clicked.connect(self.Home)
        self.ui.send_inv.clicked.connect(self.Send_position)
        self.ui.btn_up.clicked.connect(self.up)
        self.ui.btn_dw.clicked.connect(self.dw)
        self.ui.btn_pick.clicked.connect(lambda: guiEvent.Pick_gripper(self))
        self.ui.btn_drop.clicked.connect(lambda: guiEvent.Drop_gripper(self))
        #################################################################################### 

    def Home(self):
        theta1=0
        theta2=0
        z=40
        data=np.array([theta1,theta2,z])
        guiEvent.SendInformation_signal(self,data)

    def Send_position(self) :
        pos_1=float(self.ui.set_x.text())
        pos_2=float(self.ui.set_y.text())
        pos_3=float(self.ui.set_z.text())
        pos=np.array([pos_1,pos_2,pos_3])
        print(pos)
        theta_send=Scara.Inverse_Kinematics(self,pos)
        if pd.isna(theta_send[0]) and pd.isna(theta_send[1]):
            self.ui.status.setText("Position Error !")
            self.ui.status.setStyleSheet("color: red")
        else:
            self.ui.status.setText("Connected to " + 
            self.ui.port.currentText()) 
            self.ui.status.setStyleSheet("color: black")
            self.ui.theta1_control.setText(str(float(int(theta_send[0])*10)/10))
            self.ui.theta2_control.setText(str(float(int(theta_send[1])*10)/10))
            self.ui.theta_Z_control.setText(str(float(int(theta_send[2])*10)/10))
            print(theta_send)
            guiEvent.Send_theta(self)



    def Read_information(self):
        if self.ui.ser.isOpen():
            data =self.ui.ser.readline().decode()
            self.ui.ser.flushInput()
            self.ui.ser.flushOutput()
            print(data)
            time.sleep(0.1)
            self.theta_receive=data.split()
            if len(self.theta_receive)==3 :
                theta_data=np.array([self.theta_receive[0],self.theta_receive[1],self.theta_receive[2]])
                self.ui.current_z1.setText(str(self.theta_receive[2]))
                self.ui.current_theta1.setText(str(self.theta_receive[0]))
                self.ui.current_theta2.setText(str(self.theta_receive[1]))
                X,Y,Z=Scara.forwardkinematics(self,theta_data)
                self.ui.current_z.setText(str(int(Z)))
                self.ui.current_x.setText(str(int(X)))
                self.ui.current_y.setText(str(int(Y)))
                self.theta1.append(self.theta_receive[0])
                self.theta2.append(self.theta_receive[1])
                self.Z.append(self.theta_receive[2])
                self.plot_1.axes.plot(self.theta1,color="green")
                self.plot_1.axes.plot(self.theta2,color="red")
                self.plot_1.axes.plot(self.Z,color="blue")
                self.plot_1.draw()  
    def up(self) :
        theta=guiEvent.up_signal(self,10)
        guiEvent.SendInformation_signal(self,theta)
        self.ui.theta1_control.setText(str(int(theta[0])))
        self.ui.theta2_control.setText(str(int(theta[1])))
        self.ui.theta_Z_control.setText(str(int(theta[2])))
    def dw(self) :
        theta=guiEvent.dw_signal(self,10)
        guiEvent.SendInformation_signal(self,theta)
        self.ui.theta1_control.setText(str(int(theta[0])))
        self.ui.theta2_control.setText(str(int(theta[1])))
        self.ui.theta_Z_control.setText(str(int(theta[2])))


## CODE RUNNING ##
if __name__ == "__main__":
    display = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(display.exec_())

