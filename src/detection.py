#!/usr/bin/env python
import rospy  # this is the python interface for ROS
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool


class Node():
   def __init__(self):
      rospy.init_node("collision_detection")
      self.detection_sub = rospy.Subscriber('/bluerov/front_camera/tag_detections',
                                             AprilTagDetectionArray,
                                             self.on_detections)
      self.collision = rospy.Publisher("/bluerov/detected", Bool, queue_size=1)
      self.detected = False
      self.detection = False
   def on_detections(self, msg: AprilTagDetectionArray):
      for detection in msg.detections:  # iterate over all detections
            self.detection = True
            tag_id = detection.id[0]
            tag_size = detection.size[0]
            frame = detection.pose.header.frame_id
            pose: Pose = detection.pose.pose.pose
            q = pose.orientation
            p = pose.position
            
            if abs(p.z)<=0.5 and abs(p.z)>=0.2:
               self.detected = True
               self.collision.publish(self.detected)
            
            else:  
               self.detected = False
               self.collision.publish(self.detected)
            
            rospy.loginfo(
               f'Tag {tag_id} with size {tag_size:.4f} relative to {frame}: \n'
               f'pos: {p.x:.2f} | {p.y:.2f} | {p.z:.2f}\n'
               f'q: {q.w:.3f} | {q.x:.3f} | {q.y:.3f} | {q.z:.3f}')
      
      if self.detection == False:
         self.detected = False
         self.collision.publish(self.detected)
      
      self.detection = False
   def run(self):
      rospy.spin()


def main():
   node = Node()
   node.run()


if __name__ == "__main__":
   main()
