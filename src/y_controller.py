#!/usr/bin/env python
from std_msgs.msg import Float64
import rospy
import numpy as np
from std_msgs.msg import Bool

class YController():

    def __init__(self):

        self.ycurrent = 0.0
        self.ytarget = 0.0
        self.i = 0
        self.init = False
        self.xerror = 0.5
        self.error_integral = 0.0
        self.time_arr = []
        self.error_arr = []
        self.target_sub = rospy.Subscriber("y_setpoint", Float64, self.target_callback)
        self.est_sub = rospy.Subscriber("y_coord", Float64, self.est_callback)
        self.xerr = rospy.Subscriber("xerror", Float64, self.er_callback)
        self.thrust_pub = rospy.Publisher("thrust", Float64, queue_size=1)
        self.yerr_pub = rospy.Publisher("yerror", Float64, queue_size=1)
        self.initate_sub = rospy.Subscriber("start", Bool, self.initate_callback)
        self.initate_pub = rospy.Publisher("Start", Bool, queue_size=1)
    def est_callback(self, msg):
        self.ycurrent = msg.data

    def target_callback(self, msg):
        self.ytarget = msg.data

    def initate_callback(self, msg):
        self.init = msg.data
    
    def er_callback(self, msg):
        self.xerror = msg.data
    
    def get_thrust(self):
        if (self.init) and (abs(self.xerror) < 0.2):
            print("error",self.xerror)
            self.initate_pub.publish(False)
            kp = 30
            ki = 0.6
            kd = 0.1
            kw = 0.08
            vmax = 0.3
            vmin = -0.3
            filt = np.ones(5)/5
            t = rospy.get_time()
            thrust = 0.0

            if self.ycurrent != 0.0:
                self.time_arr.append(t)
                error = self.ytarget - self.ycurrent
                self.error_arr.append(error)
                self.i = self.i + 1

            if self.i > 1:
                if self.i >= 6:
                    
                    error = self.ytarget - self.ycurrent
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
                if abs(error) < 0.1:
                    vmax = 0.1
                    vmin = -0.1
                
                if thrust > vmax:
                    thrust = vmax
                elif thrust < vmin:
                    thrust = vmin
                #print("thrust",thrust)
                #print("ycurrent",self.ycurrent)
                #print("ytarget",self.ytarget)

                self.publish(thrust)
                
        else:
            j = 0
            while (j<2):
                self.publish(0.0)
                j = j + 1
           
            self.initate_pub.publish(True)

    def publish(self, setpoint):
        msg = Float64()
        msg.data = setpoint
        self.thrust_pub.publish(msg)
    
    def er_publish(self, er):
        self.yerr_pub.publish(er)

    def run(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.get_thrust()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("YController")
    node = YController()
    node.run()
