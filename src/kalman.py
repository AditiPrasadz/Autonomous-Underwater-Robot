import numpy as np
from numpy.linalg import inv
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
import math as m


class KalmanFilter():

    def __init__(self):
        rospy.init_node("Kalman_Filter")
        self.dt = 0.0
        self.measure_pos = Float64MultiArray()
        self.X_int = np.array([[0.5], [1.0], [0.0], [0.0]])  # [x][y][x.dot][y.dot]
        self.x_mean = 0.5
        self.y_mean = 1.0
        self.counter = 0
        self.flag = 0
        self.x_pub = rospy.Publisher("x_coord", Float64, queue_size=1)
        self.y_pub = rospy.Publisher("y_coord", Float64, queue_size=1)
        self.time_arr = []
        self.x_arr = []
        self.y_arr = []
        
        self.X = np.array([[0.0], [0.0], [0.0], [0.0]])
        
        self.P_mat = np.array([[1.0,0.0,0.0,0.0],
                                [0.0,1.0,0.0,0.0],
                                [0.0,0.0,1.0,0.0],
                                [0.0,0.0,0.0,1.0]])
        self.A_mat = np.array([[1.0,0.0,0.0,0.0],
                                [0.0,1.0,0.0,0.0],
                                [0.0,0.0,1.0,0.0],
                                [0.0,0.0,0.0,1.0]])
         
        self.Q_mat = np.array([[1.0,0.0,0.0,0.0],
                                 [0.0,1.0,0.0,0.0],
                                 [0.0,0.0,1.0,0.0],
                                 [0.0,0.0,0.0,1.0]]) 

        self.R_mat = np.array([[1.0,0.0,0.0,0.0],
                                 [0.0,1.0,0.0,0.0],
                                 [0.0,0.0,1.0,0.0],
                                 [0.0,0.0,0.0,1.0]]) 
        self.measure_pos_sub = rospy.Subscriber("MeasuredPostion",Float64MultiArray,self.measured_pos_callBack)
    def measured_pos_callBack(self , msg):
        
        self.measure_pos = msg.data
        self.time_arr.append(rospy.get_time())
        self.x_arr.append(self.measure_pos[0])
        self.y_arr.append(self.measure_pos[1])
        self.counter = self.counter + 1
        print(self.counter)
    
    def kf_predict(self):
        self.X = self.A_mat @ self.X_int
        self.P_mat = (self.A_mat @ (self.P_mat @ np.transpose(self.A_mat))) + self.Q_mat
   
    def kf_update(self): 

        z_ip = np.array([[self.x_arr[self.counter-1]],[self.y_arr[self.counter-1]],
                         [self.X.item(2)],[self.X.item(3)]])
        k = self.P_mat @ (inv(self.P_mat+ self.R_mat))
        self.X_int = self.X + (k @ (z_ip - self.X))
        self.P_mat = self.P_mat - (k @ self.P_mat)

    def get_variance(self, list, mean):
        sum = 0.0
        
        for i in range(2):
            sum = sum + m.pow((list[i]-mean),2)

        return sum   
    def get_covariance(self):
        sum = 0.0
        
        for i in range(2):
            sum = sum + ((self.x_arr[i]-self.x_mean) * 
                         (self.y_arr[i]-self.y_mean))
        return sum      
            

    def kalman_filter_calc(self):
        
        if(self.counter > 2 ):
            self.dt = self.time_arr[self.counter-1 ]-self.time_arr[self.counter - 2 ]
            self.A_mat[0][2] = self.dt
            self.A_mat[1][3] = self.dt
            self.kf_predict()
            self.kf_update()
            x = self.X_int.item(0)
            y = self.X_int.item(1) -0.2
            print("x =" + str(x))
            print("y =" + str(y))
            self.x_pub.publish(x)
            self.y_pub.publish(x)
        elif(self.counter == 2  ):
            print("stuck")
            x = self.x_arr[1]
            y = self.y_arr[1]
            x_dot = (self.x_arr[1] - self.x_arr[0])/(self.time_arr[1]-self.time_arr[0])
            y_dot = (self.y_arr[1] - self.y_arr[0])/(self.time_arr[1]-self.time_arr[0])
            self.X_int = np.array([[x],[y],[x_dot],[y_dot]])
            self.P_mat[0][0] = self.get_variance(self.x_arr, self.x_mean)* self.get_variance(self.x_arr, self.x_mean)
            self.P_mat[1][1] = self.get_variance(self.y_arr, self.y_mean)* self.get_variance(self.y_arr, self.y_mean)
            self.P_mat[0][1] = self.get_covariance()
            self.P_mat[1][0] = self.get_covariance()

    def run(self):
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            self.kalman_filter_calc()
            rate.sleep()

def main():
    node =  KalmanFilter()
    node.run()

if __name__ == "__main__":
    main()
