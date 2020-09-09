import sys
import os
import rospy
from simple_aruco_detector.msg import aruco_msg
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PIL import Image
from PIL.ImageQt import ImageQt


class Form(QDialog):
   def __init__(self, parent=None):
      super(Form, self).__init__(parent)
      #initialize ros node and subscriber to aruco topic
      rospy.init_node('aruco_interface', anonymous=True)
      rospy.Subscriber("aruco_pose", aruco_msg, self.callback)


      layout = QVBoxLayout()

      # Initialize all the widgets
      self.scene = QGraphicsScene(self)
      self.view = QGraphicsView(self.scene)

      # Button for emiting signal clicked for displaing ID image, not really a button at graphical interface
      # TODO maybe found another for doing it, creating signal is an option
      self.button = QPushButton()
      self.button.clicked.connect(lambda:self.display_image(self.aruco_img))

      self.IDLabel = QLabel()
      self.IDLabel.setText("Id da marca")

      self.PoseLabel = QLabel()
      self.PoseLabel.setText("Pose: ")

      self.label1 = QLabel()

      self.RotLabel = QLabel()
      self.RotLabel.setText("Orientation: ")

      self.label2 = QLabel()

      self.b1 = QPushButton("Button1")
      self.b1.setCheckable(True)
      self.b1.toggle()
      self.b1.clicked.connect(lambda:self.whichbtn(self.b1))
      self.b1.clicked.connect(self.btnstate)

      # Set Widget on layout in order

      layout.addWidget(self.b1)
      layout.addWidget(self.view)
      layout.addWidget(self.IDLabel)
      layout.addWidget(self.PoseLabel)
      layout.addWidget(self.label1)
      layout.addWidget(self.RotLabel)
      layout.addWidget(self.label2)


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

   def display_image(self, img):
        self.scene.clear()
        w, h = img.size
        self.imgQ = ImageQt(img)                      #we need to hold reference to imgQ, or it will crash
        pixMap = QPixmap.fromImage(self.imgQ)
        self.scene.addPixmap(pixMap)

        self.view.fitInView(QRectF(0, 0, w, h), Qt.KeepAspectRatio)
        self.scene.update()


   #Callback function
   def callback(self,msg):
      string_x = str(msg.pose.x)
      string_y = str(msg.pose.y)
      string_z = str(msg.pose.z)

      string_rot_x = str(msg.orientation.x)
      string_rot_y = str(msg.orientation.y)
      string_rot_z = str(msg.orientation.z)

      if(msg.ID < 8):
         self.IDLabel.setText("Id da marca: " + str(msg.ID))
         self.aruco_image = "ID_" + str(msg.ID) + ".png"
      else:
         self.IDLabel.setText("Id da Marca > 7")
         self.aruco_image = "error.png"


      self.aruco_image = os.getcwd()+ "/IDS/" + self.aruco_image
      self.aruco_img = Image.open(self.aruco_image)


      # Emit signal clicked for setting ID image at interface
      self.button.clicked.emit(True)

      self.label1.setText(string_x + "||" + string_y + "||" + string_z)
      self.label2.setText(string_rot_x + "||" + string_rot_y + "||" + string_rot_z)

def main():
   app = QApplication(sys.argv)
   ex = Form()
   ex.show()
   sys.exit(app.exec_())

if __name__ == '__main__':
   main()
