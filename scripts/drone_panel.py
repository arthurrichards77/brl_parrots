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

class DronePanel:

  def __init__(self, master, title="Drone Control Panel"):
    frame=Frame(master, bg="yellow")
    master.title(title)
    frame.pack()
    self.my_frame = frame

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

    # yaw messages
    self.yaw_l = Twist()
    self.yaw_l.angular.z = 0.1

    self.yaw_r = Twist()
    self.yaw_r.angular.z = -0.1

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

    self.rightyaw = Button(frame, text='R TURN', command=lambda: self.
vel_pub.publish(self.yaw_r))
    self.rightyaw.grid(row=1, column=2, padx=10, pady=10)

    self.leftyaw = Button(frame, text='L TURN', command=lambda: self.
vel_pub.publish(self.yaw_l))
    self.leftyaw.grid(row=1, column=0, padx=10, pady=10)

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

    self.msg_pub = rospy.Publisher('monitor/status_msg',String)
    self.vel_pub = rospy.Publisher('cmd_vel',Twist)
    self.land_pub = rospy.Publisher('ardrone/land', Empty)
    self.reset_pub = rospy.Publisher('ardrone/reset', Empty)
    self.takeoff_pub = rospy.Publisher('ardrone/takeoff', Empty)

  def stop_btn(self):
    rospy.loginfo("Stop button pressed")
    # send zero velocity
    self.vel_pub.publish(Twist())

  def say_hi(self):
    rospy.loginfo("Hello")
    self.msg_pub.publish("Hello from control panel")

  def check_ros(self):
    if rospy.is_shutdown():
      self.my_frame.quit()
    else:
      root.after(1000,app.check_ros)

rospy.init_node('control_panel', anonymous=True)

drone_name=rospy.get_param('drone_name','Drone')
panel_title=drone_name + " Control Panel"

root = Tk()
app = DronePanel(root, title=panel_title)
root.after(1000,app.check_ros)
root.mainloop()
root.destroy()
