from PyQt5.QtCore import QFileInfo
from PyQt5.QtWidgets import (QApplication, QCheckBox, QDialog,QMainWindow,QMenu,
        QDialogButtonBox, QFrame, QGroupBox, QLabel, QLineEdit, QListWidget,QGridLayout,
        QTabWidget, QVBoxLayout, QWidget,QSlider,QVBoxLayout,QHBoxLayout,QPushButton,QRadioButton,QCheckBox)
from PyQt5.QtCore import Qt
from PyQt5.QtCore import pyqtSlot
import rospy
import joint_control
import cartesian_control
class TabDialog(QDialog):
    def __init__(self, fileName, parent=None):
        super(TabDialog, self).__init__(parent)

        fileInfo = QFileInfo(fileName)

        tabWidget = QTabWidget()
        tabWidget.addTab(JointTab(fileInfo), "Joint and Cartesian control")
        tabWidget.addTab(CartesianTab(fileInfo), "Cartesian Control")
        tabWidget.addTab(JoystickTab(fileInfo), "Joystick Control")
        tabWidget.addTab(TaskTab(fileInfo), "Task Control")
        tabWidget.addTab(VisionTab(fileInfo), "Vision Control")
        tabWidget.addTab(DrawingTab(fileInfo), "Drawing")
        tabWidget.addTab(PrintingTab(fileInfo), "3DPrinting")

        buttonBox = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)

        buttonBox.accepted.connect(self.accept)
        buttonBox.rejected.connect(self.reject)

        mainLayout = QVBoxLayout()
        mainLayout.addWidget(tabWidget)
        #mainLayout.addWidget(buttonBox)
        self.setLayout(mainLayout)

        self.setWindowTitle("Tab Dialog")


class JointTab(QWidget):
    def __init__(self, fileInfo, parent=None):
        super().__init__()
        super(JointTab, self).__init__(parent)
        self.initUI()
        


    def initUI(self):
        J1 = QGroupBox("Joint angle 1")
        hbox1 = QHBoxLayout()
        hbox2 = QHBoxLayout()
        sld1 = QSlider(Qt.Horizontal, self)
        sld1.setRange(-360, 360)
        sld1.setFocusPolicy(Qt.NoFocus)
        sld1.setPageStep(5)

        self.a=sld1.valueChanged.connect(self.updateLabel1)

        self.label1 = QLabel('0', self)
        self.label1.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
        self.label1.setMinimumWidth(80)

        hbox1.addWidget(sld1)
        hbox1.addSpacing(55)
        hbox1.addWidget(self.label1)

        J2 = QGroupBox("Joint angle 2")

        hbox2 = QHBoxLayout()
        sld2 = QSlider(Qt.Horizontal, self)
        sld2.setRange(-91, 97)
        sld2.setFocusPolicy(Qt.NoFocus)
        sld2.setPageStep(5)

        self.b=sld2.valueChanged.connect(self.updateLabel2)

        self.label2 = QLabel('0', self)
        self.label2.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
        self.label2.setMinimumWidth(80)

        hbox2.addWidget(sld2)
        hbox2.addSpacing(15)
        hbox2.addWidget(self.label2)
        
        J3 = QGroupBox("Joint angle 3")

        hbox3 = QHBoxLayout()
        sld3 = QSlider(Qt.Horizontal, self)
        sld3.setRange(-91, 131)
        sld3.setFocusPolicy(Qt.NoFocus)
        sld3.setPageStep(5)

        self.c=sld3.valueChanged.connect(self.updateLabel3)

        self.label3 = QLabel('0', self)
        self.label3.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
        self.label3.setMinimumWidth(80)

        hbox3.addWidget(sld3)
        hbox3.addSpacing(15)
        hbox3.addWidget(self.label3)
        
        J4 = QGroupBox("Joint angle 4")

        hbox4 = QHBoxLayout()
        sld4 = QSlider(Qt.Horizontal, self)
        sld4.setRange(-360, 360)
        sld4.setFocusPolicy(Qt.NoFocus)
        sld4.setPageStep(5)

        self.d=sld4.valueChanged.connect(self.updateLabel4)

        self.label4 = QLabel('0', self)
        self.label4.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
        self.label4.setMinimumWidth(80)

        hbox4.addWidget(sld4)
        hbox4.addSpacing(15)
        hbox4.addWidget(self.label4)
        
        J5 = QGroupBox("Joint angle 5")

        hbox5 = QHBoxLayout()
        sld5 = QSlider(Qt.Horizontal, self)
        sld5.setRange(-114, 103)
        sld5.setFocusPolicy(Qt.NoFocus)
        sld5.setPageStep(5)

        self.e=sld5.valueChanged.connect(self.updateLabel5)

        self.label5 = QLabel('0', self)
        self.label5.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
        self.label5.setMinimumWidth(80)

        hbox5.addWidget(sld5)
        hbox5.addSpacing(15)
        hbox5.addWidget(self.label5)
        
        J6 = QGroupBox("Joint angle 6")

        hbox6 = QHBoxLayout()
        sld6 = QSlider(Qt.Horizontal, self)
        sld6.setRange(-360, 360)
        sld6.setFocusPolicy(Qt.NoFocus)
        sld6.setPageStep(5)

        self.f=sld6.valueChanged.connect(self.updateLabel6)

        self.label6 = QLabel('0', self)
        self.label6.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
        self.label6.setMinimumWidth(80)

        hbox6.addWidget(sld6)
        hbox6.addSpacing(15)
        hbox6.addWidget(self.label6)

        permissionsLayout1 = QHBoxLayout()
        permissionsLayout1.addWidget(sld1)
        permissionsLayout1.addWidget(self.label1)
        J1.setLayout(permissionsLayout1)
        
        permissionsLayout2 = QHBoxLayout()
        permissionsLayout2.addWidget(sld2)
        permissionsLayout2.addWidget(self.label2)
        J2.setLayout(permissionsLayout2)
        
        permissionsLayout3 = QHBoxLayout()
        permissionsLayout3.addWidget(sld3)
        permissionsLayout3.addWidget(self.label3)
        J3.setLayout(permissionsLayout3)
        
        permissionsLayout4 = QHBoxLayout()
        permissionsLayout4.addWidget(sld4)
        permissionsLayout4.addWidget(self.label4)
        J4.setLayout(permissionsLayout4)
        
        permissionsLayout5 = QHBoxLayout()
        permissionsLayout5.addWidget(sld5)
        permissionsLayout5.addWidget(self.label5)
        J5.setLayout(permissionsLayout5)
        
        permissionsLayout6 = QHBoxLayout()
        permissionsLayout6.addWidget(sld6)
        permissionsLayout6.addWidget(self.label6)
        J6.setLayout(permissionsLayout6)
        
        self.button = QPushButton('Plane and Execute', self)
        self.button.setToolTip('This is an example button')
        self.button.move(100,400)
       
        self.button.clicked.connect(self.on_click)
        
        mainLayout = QVBoxLayout()
        mainLayout.addWidget(J1)
        mainLayout.addWidget(J2)
        mainLayout.addWidget(J3)
        mainLayout.addWidget(J4)
        mainLayout.addWidget(J5)
        mainLayout.addWidget(J6)
        mainLayout.addWidget(self.button)
        mainLayout.addStretch(1)
        self.setLayout(mainLayout)
        
    def updateLabel1(self, value):
      self.label1.setText('X')
      self.label1.setText(str(value))
      self.a=value
    def updateLabel2(self, value):
      self.label2.setText(str(value))
      self.b=value
    def updateLabel3(self, value):
      self.label3.setText(str(value))
      self.c=value
    def updateLabel4(self, value):
      self.label4.setText(str(value))
      self.d=value
    def updateLabel5(self, value):
      self.label5.setText(str(value))
      self.e=value
    def updateLabel6(self, value):
      self.label6.setText(str(value))
      self.f=value
      
      
      
    @pyqtSlot()
    def on_click(self):
      #joint_control.go_to_joint_state(a)
      print('PyQt5 button click')
      if self.a is not None:
        a=self.a
        print(a)
      else:
        a=0
        print(a)
      if self.b is not None:
        b=self.b
        print(b)
      else:
        b=0
        print(b)
      if self.c is not None:
        c=self.c
        print(c)
      else:
        c=0
        print(c)
      if self.d is not None:
        d=self.d
        print(d)
      else:
        d=0
        print(d)
      if self.e is not None:
        e=self.e
        print(e)
      else:
        e=0
        print(e)
      if self.b is not None:
        f=self.f
        print(f)
      else:
        f=0
        print(f)
      joint_control.go_to_joint_state(a,b,c,d,e,f)


class CartesianTab(QWidget):
    def __init__(self, fileInfo, parent=None):
        super().__init__()
        super(CartesianTab, self).__init__(parent)
        self.initUI()
        


    def initUI(self):
        
        X = QGroupBox("X Co-ordinate")
        hbox1 = QHBoxLayout()
        hbox2 = QHBoxLayout()
        sld1 = QSlider(Qt.Horizontal, self)
        sld1.setRange(42, 210)
        sld1.setFocusPolicy(Qt.NoFocus)
        sld1.setPageStep(5)

        self.x=sld1.valueChanged.connect(self.updateLabel1)

        self.label1 = QLabel('0', self)
        self.label1.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
        self.label1.setMinimumWidth(80)

        hbox1.addWidget(sld1)
        hbox1.addSpacing(55)
        hbox1.addWidget(self.label1)

        Y = QGroupBox("Y Co-ordinate")

        hbox2 = QHBoxLayout()
        sld2 = QSlider(Qt.Horizontal, self)
        sld2.setRange(42, 210)
        sld2.setFocusPolicy(Qt.NoFocus)
        sld2.setPageStep(5)

        self.y=sld2.valueChanged.connect(self.updateLabel2)

        self.label2 = QLabel('0', self)
        self.label2.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
        self.label2.setMinimumWidth(80)

        hbox2.addWidget(sld2)
        hbox2.addSpacing(15)
        hbox2.addWidget(self.label2)
        
        Z = QGroupBox("Z Co-ordinate")

        hbox3 = QHBoxLayout()
        sld3 = QSlider(Qt.Horizontal, self)
        sld3.setRange(0, 230)
        sld3.setFocusPolicy(Qt.NoFocus)
        sld3.setPageStep(5)

        self.z=sld3.valueChanged.connect(self.updateLabel3)

        self.label3 = QLabel('0', self)
        self.label3.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
        self.label3.setMinimumWidth(80)
        
        hbox3.addWidget(sld3)
        hbox3.addSpacing(15)
        hbox3.addWidget(self.label3)
        
        startpos = QGroupBox("x- start co-ordinate")
        hbox4 = QHBoxLayout()
        self.textbox = QLineEdit(self)
        self.textbox.move(20, 20)
        self.textbox.resize(280,40)
        #hbox4.addWidget(sld3)
        #hbox4.addSpacing(15)
        hbox4.addWidget(self.textbox)
        
        goal = QGroupBox("Goal Position")
        hbox5 = QHBoxLayout()
        self.textbox2 = QLineEdit(self)
        self.textbox2.move(20, 20)
        self.textbox2.resize(280,40)
        #hbox4.addWidget(sld3)
        #hbox4.addSpacing(15)
        hbox5.addWidget(self.textbox)

        permissionsLayout1 = QHBoxLayout()
        permissionsLayout1.addWidget(sld1)
        permissionsLayout1.addWidget(self.label1)
        X.setLayout(permissionsLayout1)
        
        permissionsLayout2 = QHBoxLayout()
        permissionsLayout2.addWidget(sld2)
        permissionsLayout2.addWidget(self.label2)
        Y.setLayout(permissionsLayout2)
        
        permissionsLayout3 = QHBoxLayout()
        permissionsLayout3.addWidget(sld3)
        permissionsLayout3.addWidget(self.label3)
        Z.setLayout(permissionsLayout3)
       
        permissionsLayout4 = QVBoxLayout()
        #permissionsLayout1.addWidget(sld1)
        permissionsLayout4.addWidget(self.textbox)
        startpos.setLayout(permissionsLayout4)
        
        permissionsLayout5 = QVBoxLayout()
        #permissionsLayout1.addWidget(sld1)
        permissionsLayout5.addWidget(self.textbox2)
        goal.setLayout(permissionsLayout5)
        
        self.button = QPushButton('Plane and execute', self)
        self.button.setToolTip('This is an example button')
        self.button.move(100,400)
       
        self.button.clicked.connect(self.on_click)
        
        mainLayout1 = QVBoxLayout()
        mainLayout1.addWidget(X)
        mainLayout1.addWidget(Y)
        mainLayout1.addWidget(Z)
        mainLayout1.addWidget(self.button)

        mainLayout1.addStretch(1)
        self.setLayout(mainLayout1)
        
        mainLayout2 = QVBoxLayout()
        
        mainLayout2.addWidget(startpos)
        mainLayout2.addWidget(goal)
        mainLayout2.addStretch(10)
        self.setLayout(mainLayout2)
        
    def updateLabel1(self, value):
      self.label1.setText(str(value))
      self.x=value
    def updateLabel2(self, value):
      self.label2.setText(str(value))
      self.y=value
    def updateLabel3(self, value):
      self.label3.setText(str(value))
      self.z=value
    
      
      
    @pyqtSlot()
    def on_click(self):
      #joint_control.go_to_joint_state(a)
      print('PyQt5 button click')
      if self.x is not None:
        x=self.x
        print(x)
      else:
        x=0
        print(x)
      if self.y is not None:
        y=self.y
        print(y)
      else:
        y=0
        print(y)
      if self.z is not None:
        z=self.z
        print(z)
      else:
        z=0
        print(z)
      cartesian_control.pose_goal(x,y,z)


class JoystickTab(QWidget):
    def __init__(self, fileInfo, parent=None):
        super().__init__()
        super(JoystickTab,self).__init__()
        self.initUI()
    def initUI(self):
        grid = QGridLayout()
        grid.addWidget(self.createFirstExclusiveGroup(), 0, 0)
        grid.addWidget(self.createSecondExclusiveGroup(), 1, 0)
        grid.addWidget(self.createNonExclusiveGroup(), 0, 1)
        #grid.addWidget(self.createPushButtonGroup(), 1, 1)

        self.setLayout(grid)

        self.setWindowTitle('Box Layout')
        self.setGeometry(300, 300, 480, 320)
        self.show()

    def createFirstExclusiveGroup(self):
        groupbox = QGroupBox('Exclusive Radio Buttons')

        radio1 = QRadioButton('Radio1')
        radio2 = QRadioButton('Radio2')
        radio3 = QRadioButton('Radio3')
        radio1.setChecked(True)

        vbox = QVBoxLayout()
        vbox.addWidget(radio1)
        vbox.addWidget(radio2)
        vbox.addWidget(radio3)
        groupbox.setLayout(vbox)

        return groupbox

    def createSecondExclusiveGroup(self):
        groupbox = QGroupBox('Exclusive Radio Buttons')
        groupbox.setCheckable(True)
        groupbox.setChecked(False)

        radio1 = QRadioButton('Radio1')
        radio2 = QRadioButton('Radio2')
        radio3 = QRadioButton('Radio3')
        radio1.setChecked(True)
        checkbox = QCheckBox('Independent Checkbox')
        checkbox.setChecked(True)

        vbox = QVBoxLayout()
        vbox.addWidget(radio1)
        vbox.addWidget(radio2)
        vbox.addWidget(radio3)
        vbox.addWidget(checkbox)
        vbox.addStretch(1)
        groupbox.setLayout(vbox)

        return groupbox

    def createNonExclusiveGroup(self):
        groupbox = QGroupBox('Non-Exclusive Checkboxes')
        groupbox.setFlat(True)

        checkbox1 = QCheckBox('Checkbox1')
        checkbox2 = QCheckBox('Checkbox2')
        checkbox2.setChecked(True)
        tristatebox = QCheckBox('Tri-state Button')
        tristatebox.setTristate(True)

        vbox = QVBoxLayout()
        vbox.addWidget(checkbox1)
        vbox.addWidget(checkbox2)
        vbox.addWidget(tristatebox)
        vbox.addStretch(1)
        groupbox.setLayout(vbox)

        return groupbox

    def createPushButtonGroup(self):
        groupbox = QGroupBox('Push Buttons')
        groupbox.setCheckable(True)
        groupbox.setChecked(True)

        pushbutton = QPushButton('Normal Button')
        togglebutton = QPushButton('Toggle Button')
        togglebutton.setCheckable(True)
        togglebutton.setChecked(True)
        flatbutton = QPushButton('Flat Button')
        flatbutton.setFlat(True)
        popupbutton = QPushButton('Popup Button')
        menu = QMenu(self)
        menu.addAction('First Item')
        menu.addAction('Second Item')
        menu.addAction('Third Item')
        menu.addAction('Fourth Item')
        popupbutton.setMenu(menu)

        vbox = QVBoxLayout()
        vbox.addWidget(pushbutton)
        vbox.addWidget(togglebutton)
        vbox.addWidget(flatbutton)
        vbox.addWidget(popupbutton)
        vbox.addStretch(1)
        groupbox.setLayout(vbox)

        return groupbox
            

        
class TaskTab(QWidget):
    def __init__(self, fileInfo, parent=None):
        super(TaskTab, self).__init__(parent)

        grid = QGridLayout()
        grid.addWidget(self.PickPose(), 0, 0)
        grid.addWidget(self.GoalPose(), 1, 0)
        #grid.addWidget(self.createExampleGroup(), 0, 1)
        #grid.addWidget(self.createExampleGroup(), 1, 1)
        self.setLayout(grid)

        
        

    def PickPose(self):
        startpose = QGroupBox("pick position")
        startpos = QGroupBox("x- start co-ordinate")
        hbox4 = QHBoxLayout()
        self.textbox = QLineEdit(self)
        self.textbox.move(20, 20)
        self.textbox.resize(280,40)
        hbox4.addSpacing(15)
        hbox4.addWidget(self.textbox)
        #radio1 = QRadioButton("&Radio pizza")
        #radio2 = QRadioButton("R&adio taco")
        #radio3 = QRadioButton("Ra&dio burrito")

        #radio1.setChecked(True)

        vbox = QVBoxLayout()
        vbox.addWidget(self.textbox)
        #vbox.addWidget(radio2)
        #vbox.addWidget(radio3)
        vbox.addStretch(1)
        startpose.setLayout(vbox)

        return startpose
    def GoalPose(self):
        goalpose = QGroupBox("pick position")
        startpos = QGroupBox("x- start co-ordinate")
        hbox4 = QHBoxLayout()
        self.textbox = QLineEdit(self)
        self.textbox.move(20, 20)
        self.textbox.resize(280,40)
        hbox4.addSpacing(15)
        hbox4.addWidget(self.textbox)
        #radio1 = QRadioButton("&Radio pizza")
        #radio2 = QRadioButton("R&adio taco")
        #radio3 = QRadioButton("Ra&dio burrito")

        #radio1.setChecked(True)

        vbox = QVBoxLayout()
        vbox.addWidget(self.textbox)
        #vbox.addWidget(radio2)
        #vbox.addWidget(radio3)
        vbox.addStretch(1)
        goalpose.setLayout(vbox)

        return goalpose

class VisionTab(QWidget):
    def __init__(self, fileInfo, parent=None):
        super(VisionTab, self).__init__(parent)

        topLabel = QLabel("Open with:")

        applicationsListBox = QListWidget()
        applications = []

        for i in range(1, 31):
            applications.append("Application %d" % i)

        applicationsListBox.insertItems(0, applications)

        alwaysCheckBox = QCheckBox()

        if fileInfo.suffix():
            alwaysCheckBox = QCheckBox("Always use this application to open "
                    "files with the extension '%s'" % fileInfo.suffix())
        else:
            alwaysCheckBox = QCheckBox("Always use this application to open "
                    "this type of file")

        layout = QVBoxLayout()
        layout.addWidget(topLabel)
        layout.addWidget(applicationsListBox)
        layout.addWidget(alwaysCheckBox)
        self.setLayout(layout)
        
class DrawingTab(QWidget):
    def __init__(self, fileInfo, parent=None):
        super(DrawingTab, self).__init__(parent)

        fileNameLabel = QLabel("File Name:")
        fileNameEdit = QLineEdit(fileInfo.fileName())

        pathLabel = QLabel("Path:")
        pathValueLabel = QLabel(fileInfo.absoluteFilePath())
        pathValueLabel.setFrameStyle(QFrame.Panel | QFrame.Sunken)

        sizeLabel = QLabel("Size:")
        size = fileInfo.size() // 1024
        sizeValueLabel = QLabel("%d K" % size)
        sizeValueLabel.setFrameStyle(QFrame.Panel | QFrame.Sunken)

        lastReadLabel = QLabel("Last Read:")
        lastReadValueLabel = QLabel(fileInfo.lastRead().toString())
        lastReadValueLabel.setFrameStyle(QFrame.Panel | QFrame.Sunken)

        lastModLabel = QLabel("Last Modified:")
        lastModValueLabel = QLabel(fileInfo.lastModified().toString())
        lastModValueLabel.setFrameStyle(QFrame.Panel | QFrame.Sunken)

        mainLayout = QVBoxLayout()
        mainLayout.addWidget(fileNameLabel)
        mainLayout.addWidget(fileNameEdit)
        mainLayout.addWidget(pathLabel)
        mainLayout.addWidget(pathValueLabel)
        mainLayout.addWidget(sizeLabel)
        mainLayout.addWidget(sizeValueLabel)
        mainLayout.addWidget(lastReadLabel)
        mainLayout.addWidget(lastReadValueLabel)
        mainLayout.addWidget(lastModLabel)
        mainLayout.addWidget(lastModValueLabel)
        mainLayout.addStretch(1)
        self.setLayout(mainLayout)

class PrintingTab(QWidget):
    def __init__(self, fileInfo, parent=None):
        
        super(PrintingTab, self).__init__(parent)
        fileNameLabel = QLabel("File Name:")
        fileNameEdit = QLineEdit(fileInfo.fileName())

        pathLabel = QLabel("Path:")
        pathValueLabel = QLabel(fileInfo.absoluteFilePath())
        pathValueLabel.setFrameStyle(QFrame.Panel | QFrame.Sunken)

        sizeLabel = QLabel("Size:")
        size = fileInfo.size() // 1024
        sizeValueLabel = QLabel("%d K" % size)
        sizeValueLabel.setFrameStyle(QFrame.Panel | QFrame.Sunken)

        lastReadLabel = QLabel("Last Read:")
        lastReadValueLabel = QLabel(fileInfo.lastRead().toString())
        lastReadValueLabel.setFrameStyle(QFrame.Panel | QFrame.Sunken)

        lastModLabel = QLabel("Last Modified:")
        lastModValueLabel = QLabel(fileInfo.lastModified().toString())
        lastModValueLabel.setFrameStyle(QFrame.Panel | QFrame.Sunken)

        mainLayout = QVBoxLayout()
        mainLayout.addWidget(fileNameLabel)
        mainLayout.addWidget(fileNameEdit)
        mainLayout.addWidget(pathLabel)
        mainLayout.addWidget(pathValueLabel)
        mainLayout.addWidget(sizeLabel)
        mainLayout.addWidget(sizeValueLabel)
        mainLayout.addWidget(lastReadLabel)
        mainLayout.addWidget(lastReadValueLabel)
        mainLayout.addWidget(lastModLabel)
        mainLayout.addWidget(lastModValueLabel)
        mainLayout.addStretch(1)
        self.setLayout(mainLayout)
        
        
if __name__ == '__main__':

    import sys

    app = QApplication(sys.argv)

    if len(sys.argv) >= 2:
        fileName = sys.argv[1]
    else:
        fileName = "."

    tabdialog = TabDialog(fileName)
    tabdialog.show()
    sys.exit(app.exec_())
