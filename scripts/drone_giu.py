#!/usr/bin/python
from Tkinter import *

import roslib
roslib.load_manifest('aapacs_drones')
import sys
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Transform
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Empty

class App:

  def __init__(self, master):
    frame=Frame(master, bg="yellow")
    frame.pack()

    # standard velocity messages
    self.up_vel = Twist()
    self.up_vel.linear.z = 0.1

    self.dn_vel = Twist()
    self.dn_vel.linear.z = -0.1

    self.fw_vel = Twist()
    self.fw_vel.linear.x = 0.1

    self.rv_vel = Twist()
    self.rv_vel.linear.x = -0.1

    self.lf_vel = Twist()
    self.lf_vel.linear.y = 0.1

    self.rt_vel = Twist()
    self.rt_vel.linear.y = -0.1

    # nudge messages
    self.up_ndg = Transform()
    self.up_ndg.translation.z = 0.01

    self.dn_ndg = Transform()
    self.dn_ndg.translation.z = -0.01

    self.lf_ndg = Transform()
    self.lf_ndg.translation.y = 0.01

    self.rt_ndg = Transform()
    self.rt_ndg.translation.y = -0.01

    self.fw_ndg = Transform()
    self.fw_ndg.translation.x = 0.01

    self.rv_ndg = Transform()
    self.rv_ndg.translation.x = -0.01

    # quit button
    self.quit_button = Button(frame, text="Quit", command=frame.quit)
    self.quit_button.grid(row=0, column=4)

    # velocity buttons
    self.up = Button(frame, text='Up', command=lambda: self.
vel_pub.publish(self.up_vel))
    self.up.grid(row=1, column=3, padx=10, pady=10)

    self.dn = Button(frame, text='Down', command=lambda: self.
vel_pub.publish(self.dn_vel))
    self.dn.grid(row=3, column=3, padx=10, pady=10)

    self.fwd = Button(frame, text='FWD', command=lambda: self.
vel_pub.publish(self.fw_vel))
    self.fwd.grid(row=1, column=1, padx=10, pady=10)

    self.back = Button(frame, text='BACK', command=lambda: self.
vel_pub.publish(self.rv_vel))
    self.back.grid(row=3, column=1, padx=10, pady=10)

    self.left = Button(frame, text='LEFT', command=lambda: self.
vel_pub.publish(self.lf_vel))
    self.left.grid(row=2, column=0, padx=10, pady=10)

    self.right = Button(frame, text='RIGHT', command=lambda: self.
vel_pub.publish(self.rt_vel))
    self.right.grid(row=2, column=2, padx=10, pady=10)

    # nudge buttons
    self.upn = Button(frame, text='Up', fg="white", bg="black", command=lambda: self.
nudge_pub.publish(self.up_ndg))
    self.upn.grid(row=1, column=8, padx=10, pady=10)

    self.dnn = Button(frame, text='Down', fg="white", bg="black", command=lambda: self.
nudge_pub.publish(self.dn_ndg))
    self.dnn.grid(row=3, column=8, padx=10, pady=10)

    self.fwdn = Button(frame, text='FWD', fg="white", bg="black", command=lambda: self.
nudge_pub.publish(self.fw_ndg))
    self.fwdn.grid(row=1, column=6, padx=10, pady=10)

    self.backn = Button(frame, text='BACK', fg="white", bg="black", command=lambda: self.
nudge_pub.publish(self.rv_ndg))
    self.backn.grid(row=3, column=6, padx=10, pady=10)

    self.leftn = Button(frame, text='LEFT', fg="white", bg="black", command=lambda: self.
nudge_pub.publish(self.lf_ndg))
    self.leftn.grid(row=2, column=5, padx=10, pady=10)

    self.rightn = Button(frame, text='RIGHT', fg="white", bg="black", command=lambda: self.
nudge_pub.publish(self.rt_ndg))
    self.rightn.grid(row=2, column=7, padx=10, pady=10)

    # stop buttons
    self.stop1 = Button(frame, text='STOP', bg="red", command=self.stop_btn)
    self.stop1.grid(row=2, column=1, padx=10, pady=10)

    self.stop2 = Button(frame, text='STOP', bg="red", command=self.stop_btn)
    self.stop2.grid(row=2, column=3, padx=10, pady=10)

    # landing and takeoff
    self.land = Button(frame, text='Land', bg="red", command=lambda: self.land_pub.publish(Empty()))
    self.land.grid(row=4, column=2, padx=10, pady=10)

    self.takeoff = Button(frame, text='Take off', bg="green", command=lambda: self.takeoff_pub.publish(Empty()))
    self.takeoff.grid(row=4, column=0, padx=10, pady=10)

    self.rst = Button(frame, text='Reset', bg="blue", fg="white", command=lambda: self.reset_pub.publish(Empty()))
    self.rst.grid(row=4, column=1, padx=10, pady=10)

    self.msg_pub = rospy.Publisher('monitor/status_msg',String,queue_size=1)
    self.vel_pub = rospy.Publisher('ref_vel',Twist,queue_size=1)
    self.nudge_pub = rospy.Publisher('ref_nudge',Transform,queue_size=1)
    self.land_pub = rospy.Publisher('ardrone/land', Empty,queue_size=1)
    self.reset_pub = rospy.Publisher('ardrone/reset', Empty,queue_size=1)
    self.takeoff_pub = rospy.Publisher('ardrone/takeoff', Empty,queue_size=1)

  def stop_btn(self):
    rospy.loginfo("Stop button pressed")
    # send zero velocity
    self.vel_pub.publish(Twist())

  def say_hi(self):
    rospy.loginfo("Hello")
    self.msg_pub.publish("Hello from control panel")

rospy.init_node('control_panel', anonymous=True)

root = Tk()
app = App(root)
root.mainloop()
root.destroy()
