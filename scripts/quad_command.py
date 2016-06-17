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
from std_msgs.msg import String, Empty, UInt8, Bool
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from aapacs_pelican.msg import LLStatus

class App:
  def __init__(self, master):
    ## --------------------------------------------- Velocity messages --------------------------------------------- ##
    self.up_vel = Twist()
    self.up_vel.linear.z = 1

    self.dn_vel = Twist()
    self.dn_vel.linear.z = -1

    self.fw_vel = Twist()
    self.fw_vel.linear.x = 1

    self.rv_vel = Twist()
    self.rv_vel.linear.x = -1

    self.lf_vel = Twist()
    self.lf_vel.linear.y = 1

    self.rt_vel = Twist()
    self.rt_vel.linear.y = -1

    self.lf_yaw_vel = Twist()
    self.lf_yaw_vel.angular.z = 1

    self.rt_yaw_vel = Twist()
    self.rt_yaw_vel.angular.z = -1

    # - Diagonal buttons
    self.fr_vel = Twist()
    self.fr_vel.linear.x = 0.7
    self.fr_vel.linear.y = -0.7

    self.fl_vel = Twist()
    self.fl_vel.linear.x = 0.7
    self.fl_vel.linear.y = 0.7

    self.rr_vel = Twist()
    self.rr_vel.linear.x = -0.7
    self.rr_vel.linear.y = -0.7

    self.rl_vel = Twist()
    self.rl_vel.linear.x = -0.7
    self.rl_vel.linear.y = 0.7

    ## --------------------------------------------- Nudge Messages --------------------------------------------- ##
    self.up_ndg = TransformStamped()
    self.up_ndg.transform.translation.z = 0.01

    self.dn_ndg = TransformStamped()
    self.dn_ndg.transform.translation.z = -0.01

    self.lf_ndg = TransformStamped()
    self.lf_ndg.transform.translation.y = 0.01

    self.rt_ndg = TransformStamped()
    self.rt_ndg.transform.translation.y = -0.01

    self.lf_yaw_ndg = TransformStamped()
    self.lf_yaw_ndg.transform.rotation = self.yaw_to_rotation(0.01)

    self.rt_yaw_ndg = TransformStamped()
    self.rt_yaw_ndg.transform.rotation = self.yaw_to_rotation(-0.01)

    self.fw_ndg = TransformStamped()
    self.fw_ndg.transform.translation.x = 0.01

    self.rv_ndg = TransformStamped()
    self.rv_ndg.transform.translation.x = -0.01

    ## --------------------------------------------- Widget frames --------------------------------------------- ##
    mainframe = Frame(master, bg="")
    mainframe.pack()

    frame_vel = Frame(mainframe, bd=2, relief="groove")
    frame_vel.grid(row=0, column=2, padx=10, pady=10, columnspan=2)

    frame_pos = Frame(mainframe, bd=2, relief="groove")
    frame_pos.grid(row=1, column=2, padx=10, pady=10)

    frame_cmds = Frame(mainframe, bd=2, relief="groove")
    frame_cmds.grid(row=1, column=0, padx=10, pady=10)

    frame_cmd = Frame(frame_cmds)#, bd=2, relief="groove")
    frame_cmd.grid(row=1, column=0)#, padx=10, pady=10)

    frame_exps = Frame(mainframe, bd=2, relief="groove")
    frame_exps.grid(row=1, column=1, padx=10, pady=10)

    frame_laurie = Frame(frame_exps)
    frame_laurie.grid(row=0, column=0)

    frame_exp = Frame(frame_exps)
    frame_exp.grid(row=1, column=0)

    frame_status = Frame(mainframe, bd=2, relief="groove")
    frame_status.grid(row=0, column=0, padx=10, pady=10, columnspan=2)#, sticky="N")

    frame_ctrl = Frame(frame_status)
    frame_ctrl.grid()

    # quit button
    #self.quit_button = Button(mainframe, text="Quit", command=mainframe.quit)
    #self.quit_button.grid(row=0, column=4, sticky="N", padx=10, pady=10)

    ## --------------------------------------------- Velocity buttons --------------------------------------------- ##
    vel_label = Label(frame_vel, text="Velocity Controls", font=("TkDefaultFont", 16))
    vel_label.grid(row=0, column=0, columnspan=4, padx=10, pady=10)

    self.up = Button(frame_vel, text='Up', command=lambda: self.send_vel(self.up_vel))
    self.up.grid(row=2, column=3, padx=10, pady=10)

    self.dn = Button(frame_vel, text='Down', command=lambda: self.send_vel(self.dn_vel))
    self.dn.grid(row=4, column=3, padx=10, pady=10)

    self.fwd = Button(frame_vel, text='FWD', command=lambda: self.send_vel(self.fw_vel))
    self.fwd.grid(row=2, column=1, padx=10, pady=10)

    self.back = Button(frame_vel, text='BACK', command=lambda: self.send_vel(self.rv_vel))
    self.back.grid(row=4, column=1, padx=10, pady=10)

    self.left = Button(frame_vel, text='LEFT', command=lambda: self.send_vel(self.lf_vel))
    self.left.grid(row=3, column=0, padx=10, pady=10)

    self.right = Button(frame_vel, text='RIGHT', command=lambda: self.send_vel(self.rt_vel))
    self.right.grid(row=3, column=2, padx=10, pady=10)

    self.left_yaw = Button(frame_vel, text='Yaw L', command=lambda: self.send_vel(self.lf_yaw_vel))
    self.left_yaw.grid(row=1, column=0, padx=10, pady=10)

    self.right_yaw = Button(frame_vel, text='Yaw R', command=lambda: self.send_vel(self.rt_yaw_vel))
    self.right_yaw.grid(row=1, column=2, padx=10, pady=10)

    ## --------------------------------------------- Diagonal velocity buttons --------------------------------------------- ##

    self.fr = Button(frame_vel, text='', command=lambda: self.send_vel(self.fr_vel))
    self.fr.grid(row=2, column=2, padx=10, pady=10)

    self.fl = Button(frame_vel, text='', command=lambda: self.send_vel(self.fl_vel))
    self.fl.grid(row=2, column=0, padx=10, pady=10)

    self.rr = Button(frame_vel, text='', command=lambda: self.send_vel(self.rr_vel))
    self.rr.grid(row=4, column=2, padx=10, pady=10)

    self.rl = Button(frame_vel, text='', command=lambda: self.send_vel(self.rl_vel))
    self.rl.grid(row=4, column=0, padx=10, pady=10)

    ## --------------------------------------------- Stop buttons --------------------------------------------- ##
    self.stop1 = Button(frame_vel, text='STOP', bg="red", fg="white", command=lambda: self.send_vel(Twist()))
    self.stop1.grid(row=3, column=1, padx=10, pady=10)

    self.stop2 = Button(frame_vel, text='STOP', bg="red", fg="white", command=lambda: self.send_vel(Twist()))
    self.stop2.grid(row=3, column=3, padx=10, pady=10)

    ## --------------------------------------------- Velocity radio buttons --------------------------------------------- ##
    self.speeds = [('Fast', 1), ('Medium', 0.5), ('Slow', 0.1)]
    self.vel = DoubleVar()
    self.vel.set(0.1) # Initial value

    rownum = 1
    for text, speed in self.speeds:
      rownum = rownum+1
      self.b = Radiobutton(frame_vel, text=text, variable=self.vel, value=speed)#, command=lambda: self.update_vel(self.vel.get()))
      self.b.grid(row=rownum, column=4, padx=10, pady=10, sticky=W)

    ## --------------------------------------------- Nudge buttons --------------------------------------------- ##
    vel_label = Label(frame_pos, text="Nudge Controls", font=("TkDefaultFont", 16))
    vel_label.grid(row=0, column=0, columnspan=4, padx=10, pady=10)

    self.upn = Button(frame_pos, text='Up', fg="white", bg="black",
                      command=lambda: self.send_nudge(self.up_ndg))
    self.upn.grid(row=1, column=3, padx=10, pady=10)

    self.dnn = Button(frame_pos, text='Down', fg="white", bg="black",
                      command=lambda: self.send_nudge(self.dn_ndg))
    self.dnn.grid(row=3, column=3, padx=10, pady=10)

    self.fwdn = Button(frame_pos, text='FWD', fg="white", bg="black",
                       command=lambda: self.send_nudge(self.fw_ndg))
    self.fwdn.grid(row=1, column=1, padx=10, pady=10)

    self.backn = Button(frame_pos, text='BACK', fg="white", bg="black",
                        command=lambda: self.send_nudge(self.rv_ndg))
    self.backn.grid(row=3, column=1, padx=10, pady=10)

    self.leftn = Button(frame_pos, text='LEFT', fg="white", bg="black",
                        command=lambda: self.send_nudge(self.lf_ndg))
    self.leftn.grid(row=2, column=0, padx=10, pady=10)

    self.rightn = Button(frame_pos, text='RIGHT', fg="white", bg="black",
                         command=lambda: self.send_nudge(self.rt_ndg))
    self.rightn.grid(row=2, column=2, padx=10, pady=10)

    self.yleftn = Button(frame_pos, text='Yaw L', fg="white", bg="black",
                        command=lambda: self.send_nudge(self.lf_yaw_ndg))
    self.yleftn.grid(row=1, column=0, padx=10, pady=10)

    self.yrightn = Button(frame_pos, text='Yaw R', fg="white", bg="black",
                         command=lambda: self.send_nudge(self.rt_yaw_ndg))
    self.yrightn.grid(row=1, column=2, padx=10, pady=10)

    ## --------------------------------------------- Nudge radio buttons --------------------------------------------- ##
    self.nudge_dists = [('0.5 Metres', 0.5), ('0.125 Metres', 0.125), ('0.01 Metre', 0.01)]
    self.nudge_dist = DoubleVar()
    self.nudge_dist.set(0.01) # Initial value

    rownum = 0
    for text, dist in self.nudge_dists:
      rownum = rownum+1
      self.bn = Radiobutton(frame_pos, text=text, variable=self.nudge_dist, value=dist)
      self.bn.grid(row=rownum, column=4, padx=10, pady=10, sticky=W)


    # Commands frame
    cmd_label = Label(frame_cmds, text="Commands", font=("TkDefaultFont", 16))
    cmd_label.grid(row=0, column=0, padx=10, pady=10)

    self.land = Button(frame_cmd, text='Land', bg="red", fg="white", font=("TkDefaultFont", 16),
                       command=lambda: self.cmd_pub.publish('land'))
    self.land.grid(row=1, column=0, padx=10, pady=10)

    self.takeoff = Button(frame_cmd, text='Take off', font=("TkDefaultFont", 16),
                          command=lambda: self.cmd_pub.publish('takeoff'))
    self.takeoff.grid(row=0, column=0, padx=10, pady=10)

    self.exptype = StringVar()
    self.exptype.set('nudge')
    self.expselect = OptionMenu(frame_exp, self.exptype, 'local','global','nudge','vel')
    self.expselect.grid(row=3, column=1, padx=10, pady=10)

    self.exp = Button(frame_exp, text='Experiment', font=("TkDefaultFont", 16),
                           command=lambda: self.cmd_pub.publish('exp:ref' + self.exptype.get())) #('exp:refnudge'))
    self.exp.grid(row=3, column=0, padx=10, pady=10)

    #self.expglobal = Button(frame_cmd, text='Experiment (global)',
    #                        command=lambda: self.cmd_pub.publish('exp:refglobal'))
    #self.expglobal.grid(row=2, column=1, padx=10, pady=10)

    self.hover = Button(frame_cmd, text='Hover', font=("TkDefaultFont", 16),
                        command=lambda: self.cmd_pub.publish('hover'))
    self.hover.grid(row=0, column=1, padx=10, pady=10)

    self.hover = Button(frame_cmd, text='Return Home', font=("TkDefaultFont", 16),
                        command=lambda: self.cmd_pub.publish('returnhome'))
    self.hover.grid(row=1, column=1, padx=10, pady=10)

    #self.resetI = Button(frame_cmd, text='Reset Integ.',
    #                       command=lambda: self.cmd_pub.publish('resetintegrators'))
    #self.resetI.grid(row=0, column=1, padx=10, pady=10)

    self.slambutton = Button(frame_laurie, text='SLAM Start',
                            command=lambda: self.slam_pub.publish(1))
    self.slambutton.grid(row=0, column=1, padx=10, pady=10)

    self.slambutton = Button(frame_laurie, text='SLAM Stop',
                            command=lambda: self.slam_pub.publish(0))
    self.slambutton.grid(row=1, column=1, padx=10, pady=10)

    self.pathbutton = Button(frame_laurie, text='Path Start',
                            command=lambda: self.path_pub.publish(1))
    self.pathbutton.grid(row=0, column=0, padx=10, pady=10)

    self.pathbutton = Button(frame_laurie, text='Path Stop',
                            command=lambda: self.path_pub.publish(0))
    self.pathbutton.grid(row=1, column=0, padx=10, pady=10)

    # Status frame
    cmd_label = Label(frame_status, text="", font=("TkDefaultFont", 16), width=35)
    cmd_label.grid(row=02, column=0, padx=10, pady=10, columnspan=3)

    self.status_label = Label(frame_status, text="Current state: ", font=("TkDefaultFont", 14))
    self.status_label.grid(row=0, column=0, padx=2, pady=2, sticky="W", columnspan=3)
    self.status_label_populated = 0

    # insert traffic light here, on column 2
    self.canvas = Canvas(frame_status, width=20, height=20)
    self.canvas.grid(row=0,column=2,sticky=E)
    self.switch_led = self.canvas.create_oval(1,1,18,18,fill='red')

    self.info_label = Label(frame_status, text="Last message:", font=("TkDefaultFont", 10))
    self.info_label.grid(row=1, column=0, padx=2, pady=2, sticky="W", columnspan=3)

    frame_batt = Frame(frame_status)#, bd=2, relief="groove")
    frame_batt.grid(row=3, column=0, columnspan=3)

    self.batt_label = Label(frame_batt, text="Battery: ##.#v", font=("TkDefaultFont", 12))
    self.batt_label.grid(row=0, column=0, padx=10, pady=4) #, sticky=N+E+S+W

    self.ftime_label = Label(frame_batt, text="Airborne: ##m ##s", font=("TkDefaultFont", 12))
    self.ftime_label.grid(row=0, column=1, padx=10, pady=4)

    frame_ref = Frame(frame_status, bd=2, relief="groove")
    frame_ref.grid(row=2, column=0, padx=10, pady=10)

    self.ref_label = Label(frame_ref, text="Ref Position", font=("TkDefaultFont", 10), width=12)
    self.ref_label.grid(row=0, column=0, columnspan=2, padx=2, pady=2)
    self.ref_x_label = Label(frame_ref, text="X:")
    self.ref_x_label.grid(row=1, column=0, sticky="E")
    self.ref_xval_label = Label(frame_ref, text="-#.##")
    self.ref_xval_label.grid(row=1, column=1, sticky="W")
    self.ref_y_label = Label(frame_ref, text="Y:")
    self.ref_y_label.grid(row=2, column=0, sticky="E")
    self.ref_yval_label = Label(frame_ref, text="-#.##")
    self.ref_yval_label.grid(row=2, column=1, sticky="W")
    self.ref_z_label = Label(frame_ref, text="Z:")
    self.ref_z_label.grid(row=3, column=0, sticky="E")
    self.ref_zval_label = Label(frame_ref, text="-#.##")
    self.ref_zval_label.grid(row=3, column=1, sticky="W")
    self.ref_yaw_label = Label(frame_ref, text="Yaw:")
    self.ref_yaw_label.grid(row=4, column=0, sticky="E")
    self.ref_yawval_label = Label(frame_ref, text="-#.##")
    self.ref_yawval_label.grid(row=4, column=1, sticky="W")

    frame_err = Frame(frame_status, bd=2, relief="groove")
    frame_err.grid(row=2, column=1, padx=10, pady=10)

    self.err_label = Label(frame_err, text="Position Error", font=("TkDefaultFont", 10), width=12)
    self.err_label.grid(row=0, column=0, columnspan=2, padx=2, pady=2)
    self.err_x_label = Label(frame_err, text="X:")
    self.err_x_label.grid(row=1, column=0, sticky="E")
    self.err_xval_label = Label(frame_err, text="-#.##")
    self.err_xval_label.grid(row=1, column=1, sticky="W")
    self.err_y_label = Label(frame_err, text="Y:")
    self.err_y_label.grid(row=2, column=0, sticky="E")
    self.err_yval_label = Label(frame_err, text="-#.##")
    self.err_yval_label.grid(row=2, column=1, sticky="W")
    self.err_z_label = Label(frame_err, text="Z:")
    self.err_z_label.grid(row=3, column=0, sticky="E")
    self.err_zval_label = Label(frame_err, text="-#.##")
    self.err_zval_label.grid(row=3, column=1, sticky="W")
    self.err_yaw_label = Label(frame_err, text="Yaw:")
    self.err_yaw_label.grid(row=4, column=0, sticky="E")
    self.err_yawval_label = Label(frame_err, text="-#.##")
    self.err_yawval_label.grid(row=4, column=1, sticky="W")

    frame_ctrl = Frame(frame_status, bd=2, relief="groove")
    frame_ctrl.grid(row=2, column=2, padx=10, pady=10)

    self.ctrl_label = Label(frame_ctrl, text="Control", font=("TkDefaultFont", 10), width=12)
    self.ctrl_label.grid(row=0, column=0, columnspan=2, padx=2, pady=2)
    self.ctrl_x_label = Label(frame_ctrl, text="Roll:")
    self.ctrl_x_label.grid(row=1, column=0, sticky="E")
    self.ctrl_xval_label = Label(frame_ctrl, text="-#.##")
    self.ctrl_xval_label.grid(row=1, column=1, sticky="W")
    self.ctrl_y_label = Label(frame_ctrl, text="Pitch:")
    self.ctrl_y_label.grid(row=2, column=0, sticky="E")
    self.ctrl_yval_label = Label(frame_ctrl, text="-#.##")
    self.ctrl_yval_label.grid(row=2, column=1, sticky="W")
    self.ctrl_z_label = Label(frame_ctrl, text="Thrust:")
    self.ctrl_z_label.grid(row=3, column=0, sticky="E")
    self.ctrl_zval_label = Label(frame_ctrl, text="-#.##")
    self.ctrl_zval_label.grid(row=3, column=1, sticky="W")
    self.ctrl_yaw_label = Label(frame_ctrl, text="Yaw:")
    self.ctrl_yaw_label.grid(row=4, column=0, sticky="E")
    self.ctrl_yawval_label = Label(frame_ctrl, text="-#.##")
    self.ctrl_yawval_label.grid(row=4, column=1, sticky="W")

    # self.msg_pub = rospy.Publisher('monitor/status_msg',String)
    self.vel_pub = rospy.Publisher('ref_vel', Twist)
    self.nudge_pub = rospy.Publisher('ref_nudge', TransformStamped)
    self.slam_pub = rospy.Publisher('SLAMSTART_TRIGGER', Bool)
    self.path_pub = rospy.Publisher('PATHPLANNING_TRIGGER', Bool)
    # self.land_pub = rospy.Publisher('ardrone/land', Empty)
    # self.reset_pub = rospy.Publisher('ardrone/reset', Empty)
    # self.takeoff_pub = rospy.Publisher('ardrone/takeoff', Empty)
    self.cmd_pub = rospy.Publisher('quad_cmd', String)
    self.heartbeat_pub = rospy.Publisher('cmd_heartbeat', UInt8)
    self.status_sub = rospy.Subscriber('status', String, self.status_callback)
    self.info_sub = rospy.Subscriber('info', String, self.info_callback)
    self.ref_sub = rospy.Subscriber('ref', TransformStamped, self.ref_callback)
    self.err_sub = rospy.Subscriber('err', TransformStamped, self.err_callback)
    self.ctrl_sub = rospy.Subscriber('rcctrl', Twist, self.ctrl_callback)
    self.batt_sub = rospy.Subscriber('batt', UInt8, self.batt_callback)
    self.asctec_sub = rospy.Subscriber('asctec/LL_STATUS', LLStatus, self.asctec_callback)

  def update_vel(self, data):
    rospy.logwarn('Velocity = %.2f', data)

  def status_callback(self, data):
    #rospy.logwarn(data)
    self.status_label.config(text='Current state: ' + data.data)
    self.status_label_populated = 1

  def info_callback(self, data):
    #rospy.logwarn(data)
    if data.data == "":
      self.info_label.config(fg="grey")
    else:
      self.info_label.config(text='Last message: ' + data.data, fg="black")

  def batt_callback(self, data):
    pass

  def asctec_callback(self, data):
    self.batt_label.config(text="Battery: " + "{:04.1f}".format(data.battery_voltage_1/1000.0))
    self.ftime_label.config(text="Airborne: " + "{:02.0f}m".format(data.up_time/60) + " {:02.0f}s".format(data.up_time%60))
    if data.flightMode == 97:
      self.canvas.itemconfig(self.switch_led, fill='green')
    else:
      self.canvas.itemconfig(self.switch_led, fill='red')

  def ctrl_callback(self, data):
    self.ctrl_xval_label.config(text="{:5.2f}".format(data.linear.y)) #data.data[0]))
    self.ctrl_yval_label.config(text="{:5.2f}".format(data.linear.x)) #data.data[1]))
    self.ctrl_zval_label.config(text="{:5.2f}".format(data.linear.z)) #data.data[2]))
    self.ctrl_yawval_label.config(text="{:5.2f}".format(data.angular.z)) #data.data[3]))

    if data.angular.z == -1: #.data[3] == 0:
      self.ctrl_yawval_label.config(fg="green")
    elif data.angular.z == 1:#.data[3] == 1:
      self.ctrl_yawval_label.config(fg="red")
    else:
      self.ctrl_yawval_label.config(fg="black")

  def ref_callback(self, data):
    self.ref_xval_label.config(text="{:5.2f}".format(data.transform.translation.x))
    self.ref_yval_label.config(text="{:5.2f}".format(data.transform.translation.y))
    self.ref_zval_label.config(text="{:5.2f}".format(data.transform.translation.z))
    self.ref_yawval_label.config(text="{:5.2f}".format(self.rotation_to_yaw(data)))

  def err_callback(self, data):
    self.err_xval_label.config(text="{:5.2f}".format(data.transform.translation.x))
    self.err_yval_label.config(text="{:5.2f}".format(data.transform.translation.y))
    self.err_zval_label.config(text="{:5.2f}".format(data.transform.translation.z))
    self.err_yawval_label.config(text="{:5.2f}".format(self.rotation_to_yaw(data)))

  def send_nudge(self, data):
    self.cmd_pub.publish('exp:refnudge')
    scaled_nudge = TransformStamped()
    scaled_nudge.transform.translation.x = data.transform.translation.x*self.nudge_dist.get()*100
    scaled_nudge.transform.translation.y = data.transform.translation.y*self.nudge_dist.get()*100
    scaled_nudge.transform.translation.z = data.transform.translation.z*self.nudge_dist.get()*100
    scaled_yaw = self.rotation_to_yaw(data)*self.nudge_dist.get()*100
    scaled_nudge.transform.rotation = self.yaw_to_rotation(scaled_yaw)
    self.nudge_pub.publish(scaled_nudge)

  def send_vel(self, data):
    self.cmd_pub.publish('exp:refvel')
    scaled_vel = Twist()
    scaled_vel.linear.x = data.linear.x*self.vel.get()
    scaled_vel.linear.y = data.linear.y*self.vel.get()
    scaled_vel.linear.z = data.linear.z*self.vel.get()
    scaled_vel.angular.z = data.angular.z*self.vel.get()
    self.vel_pub.publish(scaled_vel)

  def send_heartbeat(self):
    self.heartbeat_pub.publish(UInt8(1))
    if self.status_label_populated == 0:
      self.cmd_pub.publish('getstate')
    # root.after(2000, self.send_heartbeat())
    # rospy.logwarn('Sending heartbeat')

  def yaw_to_rotation(self, yaw):
    # Converts a yaw angle in radians into quaternions as TransformStamped.translation.rotation
    quaternion = quaternion_from_euler(0,0,yaw)
    output = TransformStamped()
    output.transform.rotation.x = quaternion[0]
    output.transform.rotation.y = quaternion[1]
    output.transform.rotation.z = quaternion[2]
    output.transform.rotation.w = quaternion[3]
    return output.transform.rotation

  def rotation_to_yaw(self, data):
    # Converts a TransformStamped.translation.rotation into a yaw angle
    # Function takes the TransformStamped() directly as the input
    quaternion = (
      data.transform.rotation.x,
      data.transform.rotation.y,
      data.transform.rotation.z,
      data.transform.rotation.w)
    euler_angle = euler_from_quaternion(quaternion)
    return euler_angle[2]


rospy.init_node('control_panel', anonymous=True)

root = Tk()
app = App(root)


def heartbeat():
  app.send_heartbeat()
  root.after(500, heartbeat)  # Send heartbeat at 2hz
  if rospy.is_shutdown():
    root.destroy()


root.after(0, heartbeat())
# root.after(100,root.update)
# root.after(200,root.update_idletasks())
root.mainloop()
root.destroy()
