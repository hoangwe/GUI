from serial.tools.list_ports_windows import comports
from Main import MainWindow
import serial.tools.list_ports


class uiConfig(MainWindow):
    def generated_port(self):
        """
        List all comm port available in devices manager
        """
        ports = serial.tools.list_ports.comports()
        self.comPort = [
            comport.device for comport in serial.tools.list_ports.comports()
        ]
        count = len(self.comPort)
        if count == 0:
            pass
        elif count == 1:
            self.ui.port.addItem(str(self.comPort[0]))
        elif count == 2:
            self.ui.port.addItem(str(self.comPort[0]))
            self.ui.port.addItem(str(self.comPort[1]))
        elif count == 3:
            self.ui.port.addItem(str(self.comPort[0]))
            self.ui.port.addItem(str(self.comPort[1]))
            self.ui.port.addItem(str(self.comPort[2]))
        elif count == 3:
            self.ui.port.addItem(str(self.comPort[0]))
            self.ui.port.addItem(str(self.comPort[1]))
            self.ui.port.addItem(str(self.comPort[2]))
            self.ui.port.addItem(str(self.comPort[3]))
        else:
            self.ui.port.addItem(str(self.comPort[0]))
            self.ui.port.addItem(str(self.comPort[1]))
            self.ui.port.addItem(str(self.comPort[2]))
            self.ui.port.addItem(str(self.comPort[3]))
            self.ui.port.addItem(str(self.comPort[4]))

    def Configuration(self):
        if not self.ui.ser.isOpen():
         self.ui.btn_disconnect.setEnabled(True)

        uiConfig.generated_port(self)
       

