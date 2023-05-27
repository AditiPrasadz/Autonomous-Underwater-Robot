#!/usr/bin/env python
from std_msgs.msg import Float64
from std_msgs.msg import Bool
import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from depth_controller.msg import Setpoint

class XController():

    def __init__(self):
        self.xcurr = 0.0
        self.ycurr = 0.0
        self.target = 0.0
        self.lat_stable = True
        self.i = 0
        self.counter = 0
        self.error_integral = 0.0
        self.time_arr = []
        self.error_arr = []
        self.x_sub = rospy.Subscriber("/bluerov/visual_localization/pose", PoseWithCovarianceStamped, self.estx_callback)
        self.y_sub = rospy.Subscriber("/bluerov/visual_localization/pose", PoseWithCovarianceStamped, self.esty_callback)
        self.initiate = rospy.Subscriber("/bluerov/lat_setpoint", Setpoint, self.setpoint_callback)
        self.thrust_pub = rospy.Publisher("lateral_thrust", Float64, queue_size=1)
        self.stable = rospy.Publisher("lat_stable", Bool, queue_size=1)
        self.j = 0

    def estx_callback(self, msg):
        self.xcurr = msg.pose.pose.position.x

    def esty_callback(self, msg):
        self.ycurr = msg.pose.pose.position.y

    def setpoint_callback(self, msg):

        self.flag = msg.flag # X or Y
        self.target = msg.setpoint # 
    

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

            if (((self.xcurr) != 0.0 ) and ((self.ycurr)!= 0.0) and ((self.target)!= 0.0)):

                if self.flag == "X":
                    error = -self.target + self.xcurr
                elif self.flag == "Y":
                    error = self.target - self.ycurr

                self.time_arr.append(t)
                self.error_arr.append(error)
                self.i = self.i + 1

            if self.i > 1:
                if self.i >= 6:
                    
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
                #print("lat_error", error)
                thrust = kp*error + ki*self.error_integral + kd*error_der[error_der.size -1]
                
                if thrust > vmax:
                    thrust = vmax
                elif thrust < vmin:
                    thrust = vmin
        

                self.publish(thrust)
                if abs(error) < 0.07:
                    self.j = self.j +1
                    if self.j >30:
                        self.lat_stable = True
                        self.stable.publish(self.lat_stable)
                        self.j = 0
                else:
                    self.j = 0
                    self.lat_stable = False
                    self.stable.publish(self.lat_stable)

    def publish(self, setpoint):
        msg = Float64()
        msg.data = setpoint
        self.thrust_pub.publish(msg)
    

    def run(self):
        rate = rospy.Rate(100.0)
        while not rospy.is_shutdown():
            self.get_thrust()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("XController")
    node = XController()
    node.run()
