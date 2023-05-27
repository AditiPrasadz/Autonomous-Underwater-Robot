#!/usr/bin/env python
import rospy
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float64


def pressure_callback(pressure_msg, publisher):
    
    pascal_per_meter = 1.0e4
    atm_pressure = 101325
    d = -(pressure_msg.fluid_pressure-atm_pressure) / pascal_per_meter
    msg = Float64()
    msg.data = d
    publisher.publish(msg)


def main():
    rospy.init_node("depth_calculator")
    d_pub = rospy.Publisher("est_depth", Float64, queue_size=1)
    pressure_sub = rospy.Subscriber("pressure", FluidPressure,
                                    pressure_callback, d_pub)                             
    rospy.spin()


if __name__ == "__main__":
    main()
