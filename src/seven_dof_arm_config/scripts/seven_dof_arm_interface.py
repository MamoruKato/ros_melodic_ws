import sys
import os
import rospy
import numpy as np
from simple_aruco_detector.msg import aruco_msg
from sensor_msgs.msg import JointState
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PIL import Image
from PIL.ImageQt import ImageQt


class Form(QDialog):
   def __init__(self, parent=None):
      super(Form, self).__init__(parent)
      #initialize ros node and subscriber to aruco topic
      rospy.init_node('joint_state_interface', anonymous=True)
      rospy.Subscriber("joint_states", JointState, self.callback)

      
      layout = QVBoxLayout()

      # Button for emiting signal clicked for displaing ID image, not really a button at graphical interface
      # TODO maybe found another for doing it, creating signal is an option 


      self.joint_names = QLabel()
      self.joint_names.setText("Joint Names: ")

      self.name_joint1 = QLabel()

      self.str_state = QLabel()
      self.str_state.setText("Joint States: ")

      self.state_joint1 = QLabel()

      #self.name_joint2 = QLabel()
      #self.state_joint2 = QLabel()

      #self.name_joint3 = QLabel()
      #self.state_joint3 = QLabel()

      #self.name_joint4 = QLabel()
      #self.state_joint4 = QLabel()

      #self.name_joint5= QLabel()
      #self.state_joint5 = QLabel()

      #self.name_joint6 = QLabel()
      #self.state_joint6 = QLabel()

      #self.name_joint7 = QLabel()#
      #self.state_joint7 = QLabel()

      #self.name_joint8 = QLabel()
      #self.state_joint8 = QLabel()

      #self.name_joint9 = QLabel()
      #self.state_joint9 = QLabel()


      self.b1 = QPushButton("Button1")
      self.b1.setCheckable(True)
      self.b1.toggle()
      self.b1.clicked.connect(lambda:self.whichbtn(self.b1))
      self.b1.clicked.connect(self.btnstate)

      # Set Widget on layout in order

      layout.addWidget(self.joint_names)
      layout.addWidget(self.name_joint1)
      layout.addWidget(self.str_state)
      layout.addWidget(self.state_joint1)

      #layout.addWidget(self.name_joint2)
      #layout.addWidget(self.state_joint2)

      #layout.addWidget(self.name_joint3)
      #layout.addWidget(self.state_joint3)

      #layout.addWidget(self.name_joint4)
      #layout.addWidget(self.state_joint4)

      #layout.addWidget(self.name_joint5)
      #layout.addWidget(self.state_joint5)

      #layout.addWidget(self.name_joint6)
      #layout.addWidget(self.state_joint6)

      #layout.addWidget(self.name_joint7)
      #layout.addWidget(self.state_joint7)

      #layout.addWidget(self.name_joint8)
      #layout.addWidget(self.state_joint8)

      #layout.addWidget(self.name_joint9)
      #layout.addWidget(self.state_joint9)
      
      self.setLayout(layout)
      
      self.setWindowTitle("Button demo")

   def btnstate(self):
      if self.b1.isChecked():
         self.label1.setText("Button Pressed")
         rospy.spin();
      else:
         self.label1.setText("Button Released")
         
   def whichbtn(self,b):
      print "clicked button is "+b.text()


   #Callback function
   def callback(self,msg):
      self.str_names = "|"
      self.joint_position = "|"

      for i in range(len(msg.name)):
         self.str_names = self.str_names + "| "+  msg.name[i]

      self.str_names = self.str_names + "||"
      self.name_joint1.setText(self.str_names)

      for i in range(len(msg.position)):

         self.joint_position = self.joint_position + "| " + str(msg.position[i])

      self.state_joint1.setText(self.joint_position)

def main():
   app = QApplication(sys.argv)
   ex = Form()
   ex.show()
   sys.exit(app.exec_())
   
if __name__ == '__main__':
   main()