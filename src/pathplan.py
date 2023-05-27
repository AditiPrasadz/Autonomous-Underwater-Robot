#!/usr/bin/env python
from pyamaze import maze,agent,textLabel
from queue import PriorityQueue
from depth_controller.msg import Path
from std_msgs.msg import Bool
from std_msgs.msg import String
import rospy

class Path():

    def __init__(self):

        self.node_sub = rospy.Subscriber("/bluerov/node", Nodes, self.node_callback)
        self.initiate = rospy.Subscriber("/bluerov/initate", Bool, self.initate_callback)
        self.orientation = rospy.Subscriber("/bluerov/orientation", String, self.orientation_callback)
        self.path_pub = rospy.Publisher("/bluerov/path", Path, queue_size=1)
        self.initiate = False
        self.m=maze(6,3)
        self.m.CreateMaze()
        self.m.maze_map = {(1, 1): {'E': 1, 'W': 0, 'N': 0, 'S': 1}, (2, 1): {'E': 1, 'W': 0, 'N': 1, 'S': 1}, (3, 1): {'E': 1, 'W': 0, 'N': 1, 'S': 1}, (4, 1): {'E': 1, 'W': 0, 'N': 1, 'S': 1}, (5, 1): {'E': 1, 'W': 0, 'N': 1, 'S': 1}, (6, 1): {'E': 1, 'W': 0, 'N': 1, 'S': 0}, (1, 2): {'E': 1, 'W': 1, 'N': 0, 'S': 1}, (2, 2): {'E': 1, 'W': 1, 'N': 1, 'S': 1}, (3, 2): {'E': 1, 'W': 1, 'N': 1, 'S': 1}, (4, 2): {'E': 1, 'W': 1, 'N': 1, 'S': 1}, (5, 2): {'E': 1, 'W': 1, 'N': 1, 'S': 1}, (6, 2): {'E': 1, 'W': 1, 'N': 1, 'S': 0}, (1, 3): {'E': 0, 'W': 1, 'N': 0, 'S': 1}, (2, 3): {'E': 0, 'W': 1, 'N': 1, 'S': 1}, (3, 3): {'E': 0, 'W': 1, 'N': 1, 'S': 1}, (4, 3): {'E': 0, 'W': 1, 'N': 1, 'S': 1}, (5, 3): {'E': 0, 'W': 1, 'N': 1, 'S': 1}, (6, 3): {'E': 0, 'W': 1, 'N': 1, 'S': 0}}
    
    def node_callback(self, msg):
        self.row = msg.row
        self.col = msg.col
    
    def initate_callback(self, msg):
        self.initiate = msg.data

    def orientation_callback(self, msg):
        self.orientation = msg.data

    def h(self,cell1,cell2):
        x1,y1=cell1
        x2,y2=cell2
        return abs(x1-x2) + abs(y1-y2)
    
    def aStar(self,m,row,col):
        start=(row,col)
        print("start",start)
        g_score={cell:float('inf') for cell in m.grid}
        g_score[start]=0
        f_score={cell:float('inf') for cell in m.grid}
        f_score[start]=self.h(start,(1,1))

        open=PriorityQueue()
        open.put((self.h(start,(1,1)),self.h(start,(1,1)),start))
        aPath={}
        while not open.empty():
            currCell=open.get()[2]
            if currCell==(1,1):
                break
            for d in 'ESNW':
                if m.maze_map[currCell][d]==True:
                    if d=='E':
                        childCell=(currCell[0],currCell[1]+1)
                    if d=='W':
                        childCell=(currCell[0],currCell[1]-1)
                    if d=='N':
                        childCell=(currCell[0]-1,currCell[1])
                    if d=='S':
                        childCell=(currCell[0]+1,currCell[1])

                    temp_g_score=g_score[currCell]+1
                    temp_f_score=temp_g_score+self.h(childCell,(1,1))

                    if temp_f_score < f_score[childCell]:
                        g_score[childCell]= temp_g_score
                        f_score[childCell]= temp_f_score
                        open.put((temp_f_score,self.h(childCell,(1,1)),childCell))
                        aPath[childCell]=currCell
        fwdPath={}
        cell=(1,1)
        while cell!=start:
            fwdPath[aPath[cell]]=cell
            cell=aPath[cell]
        return fwdPath

    def run(self):
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            if self.initiate == True:
                
                self.m.maze_map[(self.row,self.col)][self.orientation] = 0
                path = self.aStar(self.m,self.row,self.col)
                self.path_pub.publish(path)
                print(self.m.maze_map)
                print(path)
                self.initiate = False
                rate.sleep()
    
if __name__ == '__main__':
    rospy.init_node("PathPlanner")
    node = Path()
    node.run()
