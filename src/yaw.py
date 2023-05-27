#!/usr/bin/env python
from std_msgs.msg import Float64
import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

class YawController():

    def __init__(self):

        self.target = 0.0
        self.curr = 0.0
        self.i = 0
        self.error_integral = 0.0
        self.time_arr = []
        self.error_arr = []
        self.target_sub = rospy.Subscriber("yaw_setpoint", Float64, self.target_callback)
        self.y_sub = rospy.Subscriber("/bluerov/visual_localization/pose", PoseWithCovarianceStamped, self.yaw_callback)
        self.thrust_pub = rospy.Publisher("yaw", Float64, queue_size=1)
        self.counter = 0
    def target_callback(self, msg):

        self.target = msg.data

    def yaw_callback(self,msg):

        q = msg.pose.pose.orientation
        q_list = [q.x, q.y, q.z, q.w]
        (roll, pitch, yaw) = euler_from_quaternion(q_list)
        self.curr = yaw*57.2958
        
    

    def get_thrust(self):

            kp = 0.008
            ki = 0.0003
            kd = 0.0001
            kw = 0.00004
            vmax = 0.25
            vmin = -0.25
            filt = np.ones(5)/5
            t = rospy.get_time()
            thrust = 0.0
            
            if self.counter>2:
                
                err = self.target - self.curr

                if abs(err) < 180:
                    error = err

                else:
                    error = abs(err) - 360

                
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

                thrust = kp*error + ki*self.error_integral + kd*error_der[error_der.size -1]
               
            #    if abs(self.error) < 0.05:
            #        vmax = 0.2
            #        vmin = -0.2
                
                if thrust > vmax:
                    thrust = vmax
                elif thrust < vmin:
                    thrust = vmin

                print(error)
                self.publish(thrust)
            self.counter = self.counter +1
    def publish(self, setpoint):

        msg = Float64()
        msg.data = setpoint
        self.thrust_pub.publish(msg)
    
    def run(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.get_thrust()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("YawController")
    node = YawController()
    node.run()
