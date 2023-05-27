#!/usr/bin/env python
from std_msgs.msg import Float64
import rospy
from scipy import integrate
import numpy as np
from scipy.signal import savgol_filter


class MyController():

    def __init__(self):

        self.current = 0.0
        self.target = 0.0
        self.i = 0
        self.error_integral = 0.0
        self.time_arr = []
        self.error_arr = []
        self.target_sub = rospy.Subscriber("depth_setpoint", Float64, self.target_callback)
        self.est_sub = rospy.Subscriber("est_depth", Float64, self.est_callback)
        self.vertical_thrust_pub = rospy.Publisher("vertical_thrust", Float64, queue_size=1)
        self.error_pub = rospy.Publisher("smooth_error", Float64, queue_size=1)
        self.err_pub = rospy.Publisher("error", Float64, queue_size=1)
        
    def est_callback(self, msg):
        self.current = msg.data

    def target_callback(self, msg):
        self.target = msg.data
       
    def get_thrust(self):

        kp = 0.8
        ki = 0.03
        kd = 0.1
        kw = 0.04
        vmax = 0.4
        vmin = -0.4
        filt = np.ones(15)/15
        t = rospy.get_time()
        vertical_thrust = 0.0

        if self.current != 0.0:
            self.time_arr.append(t)
            error = self.target - self.current
            self.error_arr.append(error)
            self.i = self.i + 1

        if self.i > 1:
            if self.i >= 16:
                error = self.target - self.current
                er = Float64()
                er.data = error
                self.er_publish(er)
                error_smooth = np.convolve(self.error_arr, filt, mode="valid")
                error_der = np.gradient(error_smooth, self.time_arr[7:-7])
                z = error_smooth[error_smooth.size-1]
                error_sm = Float64()
                error_sm.data = z
                self.error_publ(error_sm)
                
            else:
                error_der = np.array([0.0])

            
            self.error_integral = ((self.error_arr[self.i-1] + self.error_arr[self.i-2]/2))*(self.time_arr[self.i-1]-self.time_arr[self.i-2]) + self.error_integral
            vertical_thrust = kp*error + ki*self.error_integral + kd*error_der[error_der.size -1]
            
            if vertical_thrust < vmin:
                self.error_arr[self.i-1] = self.error_arr[self.i-1] + kw*(vmin - vertical_thrust)
                self.error_integral = ((self.error_arr[self.i-1] + self.error_arr[self.i-2]/2))*(self.time_arr[self.i-1]-self.time_arr[self.i-2]) + self.error_integral
            elif vertical_thrust > vmax:
                self.error_arr[self.i-1] = self.error_arr[self.i-1] + kw*(vmax - vertical_thrust)
                self.error_integral = ((self.error_arr[self.i-1] + self.error_arr[self.i-2]/2))*(self.time_arr[self.i-1]-self.time_arr[self.i-2]) + self.error_integral  
            
            vertical_thrust = kp*error + ki*self.error_integral + kd*error_der[error_der.size -1]

            if vertical_thrust > vmax:
                vertical_thrust = vmax
            elif vertical_thrust < vmin:
                vertical_thrust = vmin
            
           # if self.error_arr[self.i-1] < -0.1:
            #    vertical_thrust = -0.01

            #if (self.current < -0.9):
            #    vertical_thrust = 0.5
            #print("depth_current",self.current)
            #print("depth_target",self.target)

                

        
        

      

        self.publish(vertical_thrust)
    
    def publish(self, setpoint):
        msg = Float64()
        msg.data = setpoint
        self.vertical_thrust_pub.publish(msg)
    def er_publish(self,er):
        self.err_pub.publish(er)
    def error_publ(self, error):
  
        self.error_pub.publish(error)
        
    def run(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.get_thrust()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("MyController")
    node = MyController()
    node.run()


