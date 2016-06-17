#!/usr/bin/python
from Tkinter import *

import roslib

roslib.load_manifest('aapacs_drones')
import sys
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Transform, TransformStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Empty, UInt8
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class App:
  def __init__(self, master):
    # standard velocity messages
    self.fake_pos = TransformStamped()
    self.fake_pos.transform.translation.x = 0.2


    # Create frames for widgets
    mainframe = Frame(master, bg="")
    mainframe.pack()

    # Labels
    lab = Label(mainframe, text="X position")
    lab.grid(row=0, column=0, sticky="E")
    lab = Label(mainframe, text="Y position")
    lab.grid(row=1, column=0, sticky="E")
    lab = Label(mainframe, text="Z position")
    lab.grid(row=2, column=0, sticky="E")
    lab = Label(mainframe, text="Yaw angle")
    lab.grid(row=3, column=0, sticky="E")

    # Entry boxes
    self.ex = Entry(mainframe)
    self.ex.grid(row=0,column=1,columnspan=2)
    self.ex.insert(0,"0")
    self.ey = Entry(mainframe)
    self.ey.grid(row=1,column=1,columnspan=2)
    self.ey.insert(0,"0")
    self.ez = Entry(mainframe)
    self.ez.grid(row=2,column=1,columnspan=2)
    self.ez.insert(0,"0")
    self.er = Entry(mainframe)
    self.er.grid(row=3,column=1,columnspan=2)
    self.er.insert(0,"0")

    # update button
    self.update_button = Button(mainframe, text="Update", command=self.update_position)
    self.update_button.grid(row=4, column=2)
    self.ground_button = Button(mainframe, text="Ground", command=self.ground_position)
    self.ground_button.grid(row=4, column=0)
    self.hover_button = Button(mainframe, text="Hover", command=self.hover_position)
    self.hover_button.grid(row=4, column=1)

    self.pos_pub = rospy.Publisher('drone', TransformStamped)
    self.send_position()

  def ground_position(self):
    self.fake_pos.transform.translation.x = 0
    self.fake_pos.transform.translation.y = 0
    self.fake_pos.transform.translation.z = 0
    self.fake_pos.transform.rotation = self.yaw_to_rotation(0)

  def hover_position(self):
    self.fake_pos.transform.translation.x = 0
    self.fake_pos.transform.translation.y = 0
    self.fake_pos.transform.translation.z = 0.8
    self.fake_pos.transform.rotation = self.yaw_to_rotation(0)

  def update_position(self):
    self.fake_pos.transform.translation.x = float(self.ex.get())
    self.fake_pos.transform.translation.y = float(self.ey.get())
    self.fake_pos.transform.translation.z = float(self.ez.get())
    self.fake_pos.transform.rotation = self.yaw_to_rotation(float(self.er.get()))

  def send_position(self):
    self.pos_pub.publish(self.fake_pos)
    # root.after(2000, self.send_position())
    # rospy.logwarn('Sending position')

  def yaw_to_rotation(self, yaw):
    # Converts a yaw angle in radians into quaternions as TransformStamped.translation.rotation
    quaternion = quaternion_from_euler(0,0,yaw)
    output = TransformStamped()
    output.transform.rotation.x = quaternion[0]
    output.transform.rotation.y = quaternion[1]
    output.transform.rotation.z = quaternion[2]
    output.transform.rotation.w = quaternion[3]
    return output.transform.rotation

rospy.init_node('fake_posn', anonymous=True)

root = Tk()
root.title("Fake quad position")
app = App(root)


def position_update():
  app.send_position()
  root.after(500, position_update)  # Send position at 2hz
  if rospy.is_shutdown():
    root.destroy()


root.after(0, position_update)
root.mainloop()
root.destroy()
