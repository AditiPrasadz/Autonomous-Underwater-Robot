#!/usr/bin/env python
from std_msgs.msg import Float64
from std_msgs.msg import Bool
import rospy
import numpy as np
import math as m
class XController():

    def __init__(self):

        self.xcurrent = 0.0
        self.xtarget = 0.0
        self.i = 0
        self.counter = 0
        #self.error = 0.02
        self.error_integral = 0.0
        self.time_arr = []
        self.error_arr = []
        self.target_sub = rospy.Subscriber("x_setpoint", Float64, self.target_callback)
        self.est_sub = rospy.Subscriber("x_coord", Float64, self.est_callback)
        self.thrust_pub = rospy.Publisher("lateral_thrust", Float64, queue_size=1)
        self.xerr_pub = rospy.Publisher("xerror", Float64, queue_size=1)
        self.initate = rospy.Publisher("start", Bool, queue_size=1)
        self.initate_sub = rospy.Subscriber("Start", Bool, self.initate_callback)
        self.init = True
    
    def est_callback(self, msg):
        self.xcurrent = msg.data

    def target_callback(self, msg):
        self.xtarget = msg.data

    def initate_callback(self, msg):
        self.init = msg.data

    def get_thrust(self):
        
            kp = 3
            ki = 0.1
            kd = 0.1
            kw = 0.06
            vmax = 0.25
            vmin = -0.25
            filt = np.ones(5)/5
            t = rospy.get_time()
            thrust = 0.0

            if self.xcurrent != 0.0:
            #self.error = 5
                self.time_arr.append(t)
                error = -self.xtarget + self.xcurrent
                self.error_arr.append(error)
                self.i = self.i + 1

            if self.i > 1:
                if self.i >= 6:
                    
                    error = -self.xtarget + self.xcurrent
                    er = Float64()
                    er.data = error
                    self.er_publish(er)
                    error_smooth = np.convolve(self.error_arr, filt, mode="valid")
                    error_der = np.gradient(error_smooth, self.time_arr[2:-2])

                else:
                    error_der = np.array([0.0])

                self.error_integral = ((self.error_arr[self.i-1] + self.error_arr[self.i-2]/2))*(self.time_arr[self.i-1]-self.time_arr[self.i-2]) + self.error_integral
                
                
                if thrust < vmin:
                    self.error_arr[self.i-1] = self.error_arr[self.i-1] + kw*(vmin - thrust)
                    self.error_integral = ((self.error_arr[self.i-1] + self.error_arr[self.i-2]/2))*(self.time_arr[self.i-1]-self.time_arr[self.i-2]) + self.error_integral
                elif thrust > vmax:
                    self.error_arr[self.i-1] = self.error_arr[self.i-1] + kw*(vmax - thrust)
                    self.error_integral = ((self.error_arr[self.i-1] + self.error_arr[self.i-2]/2))*(self.time_arr[self.i-1]-self.time_arr[self.i-2]) + self.error_integral  
                #print("error", error)
                thrust = kp*error + ki*self.error_integral + kd*error_der[error_der.size -1]
                if thrust > vmax:
                    thrust = vmax
                elif thrust < vmin:
                    thrust = vmin
                #print("thrust",thrust)
                #print("xcurrent",self.xcurrent)
                #print("xtarget",self.xtarget)
                print("init",self.init)
                if self.init:
                    if (abs(error) > 0.1):
                        self.publish(thrust)
                        self.initate.publish(False)
                    else:
                        j = 0
                        while (j<2):
                            self.publish(0.0)
                            j = j + 1
                        self.initate.publish(True)

    def publish(self, setpoint):
        msg = Float64()
        msg.data = setpoint
        self.thrust_pub.publish(msg)
    
    def er_publish(self, er):
        self.xerr_pub.publish(er)

    def run(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.get_thrust()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("XController")
    node = XController()
    node.run()
