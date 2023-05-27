#!/usr/bin/env python
import rospy 
import numpy as np
from fav_msgs.msg import RangeMeasurementArray
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
import math 

class RovRanges():
    def __init__(self):
        self.ap_width = 0.6
        self.ap_height= 0.4
        self.pressure = Float64()
        self.real_postion = np.matrix([[0],[0],[0],[0]])
        self.current_depth= 0.0
        self.range = [0] * 4
        self.H_matrix= np.matrix([[1, 0, 0, 0.5],
                            [0, math.cos(math.pi/2), -math.sin(math.pi/2), 3.35],
                            [0, math.sin(math.pi/2),  math.cos(math.pi/2),-0.50],
                            [0, 0, 0, 1]])
        rospy.init_node("RovRanges")
        self.range_read= RangeMeasurementArray()
        #self.id_read= RangeMeasurementArray()
        self.pressure_sub   = rospy.Subscriber("/bluerov/pressure", FluidPressure, self.pressure_callback)
        self.range_sub      = rospy.Subscriber("/bluerov/ranges", RangeMeasurementArray,self.range_callback)
        self.loclize_pub = rospy.Publisher("MeasuredPostion", Float64MultiArray, queue_size=1)
        self.prex = rospy.Publisher("prex", Float64, queue_size=1)
        self.prey = rospy.Publisher("prey", Float64, queue_size=1)
    def pressure_callback(self, pressure_msg):
        pascal_per_meter = 1.0e4
        self.current_depth = (  1.0e5 - pressure_msg.fluid_pressure ) / pascal_per_meter                                 
    
    def IdSum(self):
        sum = 0
        ID = self.SortId()
        for i in range(3):
            sum = sum + ID[i]
        return sum
    def SortId(self):
        i = 0
        counter = 1
        x = len(self.range_read)
        ID = [0] * x
        while (i < x):
            for j in self.range_read:
                if (j.id == counter):
                    ID[i] = j.id

            i = i + 1
            counter = counter +1
        return ID
    def Sort(self):
        i = 0
        counter = 1
        x = len(self.range_read)
        print(self.range_read)
        range = [0] * x
        while (i < x):
            for j in self.range_read:
                if (j.id == counter):
                    range[i] = j.range

            i = i + 1
            counter = counter +1
        print(range)
        return range

    def loclize(self):
        range = self.Sort()
        d1 = range[0]
        d2 = range[1]
        d3 = range[2]
        
        print(d1,d2,d3)
        if self.IdSum()   == 6:  
            x = (math.pow(d1,2)-math.pow(d2,2)+math.pow(self.ap_width,2))/(2*self.ap_width)
            y = (math.pow(d1,2)-math.pow(d3,2)+math.pow(self.ap_height,2))/(2*self.ap_height)
            z = math.sqrt(math.pow(d1,2)-math.pow(x,2)-math.pow(y,2))
        elif self.IdSum() == 7:
            x = (math.pow(d1,2)-math.pow(d2,2)+math.pow(self.ap_width,2))/(2*self.ap_width)
            y = (math.pow(d2,2)-math.pow(d3,2)+math.pow(self.ap_height,2))/(2*self.ap_height)
            z = math.sqrt(math.pow(d1,2)-math.pow(x,2)-math.pow(y,2))
            
        elif self.IdSum() == 8:
            x = (math.pow(d2,2)-math.pow(d3,2)+math.pow(self.ap_width,2))/(2*self.ap_width)
            y = (math.pow(d1,2)-math.pow(d2,2)+math.pow(self.ap_height,2))/(2*self.ap_height)
            z = math.sqrt(math.pow(d1,2)-math.pow(x,2)-math.pow(y,2))
        elif self.IdSum() == 9:
            x = (math.pow(d2,2)-math.pow(d3,2)+math.pow(self.ap_width,2))/(2*self.ap_width)
            y = (math.pow(d1,2)-math.pow(d3,2)+math.pow(self.ap_height,2))/(2*self.ap_height)
            z = math.sqrt(math.pow(d3,2)-math.pow((self.ap_height-y),2)-math.pow((self.ap_width-x),2))

        P_vector = np.matrix([[x],[y],[z],[1]])
        self.real_postion = np.matmul(self.H_matrix,P_vector)


        return self.real_postion

    def range_callback(self, msg):
        self.range_read = msg.measurements
        #self.range_id = msg.id
        #print(msg.id)
        if len(self.range_read) >= 3:
            real_postion=self.loclize()
        else:
             print("Less than 3 april tags are detected")

    def publish(self):
        msg = Float64MultiArray()
        msg.data = np.array([self.real_postion.item(0),self.real_postion.item(1),self.current_depth])
        self.loclize_pub.publish(msg)
        #self.loclize_pub = rospy.Publisher("MeasuredPostion", Float64MultiArray, queue_size=1)
        self.prex.publish(self.real_postion.item(0))
        self.prey.publish(self.real_postion.item(1))


    def run(self):
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

def main():
    node = RovRanges()
    node.run()

if __name__ == "__main__":
    main()
