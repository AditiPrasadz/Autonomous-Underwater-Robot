#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion


class DepthSetpointNode():
    def __init__(self):
        rospy.init_node("depth_setpoint_publisher")

        self.start_time = rospy.get_time()
        self.xcurrent = 0.0
        self.ycurrent = 0.0 
        self.yawcurrent = 0.0
        self.constant = True
        self.duration = 50.0
        self.setpoint_1 = -0.3
        self.setpoint_2 = -0.8
        self.xsetpoint1 = 0.4
        self.xsetpoint2 = 0.9
        self.ysetpoint1 = 1.5
        self.ysetpoint2 = 1.5
        self.yawsetpoint1 = 180
        self.yawsetpoint2 = 90
        self.setpoint_pub = rospy.Publisher("depth_setpoint",Float64,queue_size=1)
        self.x_sub = rospy.Subscriber("/bluerov/visual_localization/pose", PoseWithCovarianceStamped, self.estx_callback)
        self.y_sub = rospy.Subscriber("/bluerov/visual_localization/pose", PoseWithCovarianceStamped, self.esty_callback)
        self.yaw_sub = rospy.Subscriber("/bluerov/visual_localization/pose", PoseWithCovarianceStamped, self.estyaw_callback)
        self.yerr_pub = rospy.Publisher("y_error",Float64,queue_size=1)
        self.xerr_pub = rospy.Publisher("x_error",Float64,queue_size=1)
        self.yaw_pub = rospy.Publisher("yaw_error",Float64,queue_size=1)

    def estx_callback(self, msg):
        self.xcurrent = msg.pose.pose.position.x

    def esty_callback(self, msg):
        self.ycurrent = msg.pose.pose.position.y
    
    def estyaw_callback(self, msg):
        q = msg.pose.pose.orientation
        q_list = [q.x, q.y, q.z, q.w]
        (roll, pitch, yaw) = euler_from_quaternion(q_list)
        self.yawcurrent = yaw*57.2958
        print(self.yawcurrent)
        

        
    def get_setpoint(self):
        if self.constant:
            setpoint = self.setpoint_1
            err = - self.xsetpoint1 + self.xcurrent
            self.xerr_pub.publish(err)
            yerr = self.ysetpoint1 - self.ycurrent
            self.yerr_pub.publish(yerr)
            yawerr = self.yawsetpoint1 - self.yawcurrent
            print(yawerr)
            self.yaw_pub.publish(yawerr)
        else:
            now = rospy.get_time()
            time = self.start_time - now

            i = time % (self.duration * 2)
            if i > (self.duration):
                setpoint = self.setpoint_1
                setpoint = self.setpoint_1
                err = - self.xsetpoint1 + self.xcurrent
                self.xerr_pub.publish(err)
                yerr = self.ysetpoint1 - self.ycurrent
                self.yerr_pub.publish(yerr)
                yawerr = self.yawsetpoint1 - self.yawcurrent
                self.yaw_pub.publish(yawerr)
            else:
                setpoint = self.setpoint_2
                err = - self.xsetpoint2 + self.xcurrent
                self.xerr_pub.publish(err)
                yerr = self.ysetpoint2 - self.ycurrent
                self.yerr_pub.publish(yerr)
                yawerr = self.yawsetpoint2 - self.yawcurrent
                self.yaw_pub.publish(yawerr)

        self.publish_setpoint(setpoint)
        
    def publish_setpoint(self, setpoint):
        msg = Float64()
        msg.data = setpoint
        self.setpoint_pub.publish(msg)

    def run(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.get_setpoint()
            rate.sleep()


def main():
    node = DepthSetpointNode()
    node.run()


if __name__ == "__main__":
    main()
