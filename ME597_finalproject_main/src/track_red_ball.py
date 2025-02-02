#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as RosImage
from PIL import Image
from process import *

class dist:
    def __init__(self):
        self.image = []
        self.bridge = CvBridge()
      
    def scan_callback(self, data):
        try:
            #Converts data from ros image to cv image
            cvImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #Convert image from an array into a Pill Image
            self.image = Image.fromarray(cvImage)
        except CvBridgeError as e:
            print("Error: ",e)
        
    
    
    def track_node(self):
        # Declare the node, and register it with a unique name
        rospy.init_node('track_node', anonymous=True)
    
        #create subscriber to get rosimage and publisher to publish robot speeds
        rospy.Subscriber("/camera/rgb/image_raw", RosImage, self.scan_callback)
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    
        #Define the execution rate object (10Hz)
        rate = rospy.Rate(10)
        
        # Log/trace information on console
        rospy.loginfo('[my_control_node] Running')
                   
        # Create message object with a specific type
        vel_msg = Twist()
        
        # Variables needed in code
        keypoints = []
        image_array = []
        
        prev_angular = 0
        
        #This is the main node loop
        while not rospy.is_shutdown():
        
           #Convert image into an array
           image_array = np.array(self.image)
             
           if len(image_array) > 0:
               #process image to get keypoint positions
               keypoints = find_keypts(np.array(self.image))
               ball_pix, tot_pix = num_pixels(self.image)
               
               #obtain height of image and calculate x_coordinate center
               h = len(image_array[0])
               o_x = h/2
               
               #Logic to make the car track ball
               if len(keypoints) > 0:
                   dif_x = keypoints[0][0] - o_x
                   dif_pix = 3000/tot_pix - ball_pix/tot_pix
               
                   #use the difference (dif_x) to set direction of movement of the car
                   if dif_x > 0:  
                       vel_msg.angular.z = -0.5
                       vel_msg.linear.x = dif_pix*100000
                       prev_angular = -0.5
                   else:
                       vel_msg.angular.z = 0.5
                       vel_msg.linear.x = dif_pix*100000
                       prev_angular = 0.5
               else: #car loses ball, keeps rotating in same direction until it finds ball
                   vel_msg.angular.z = prev_angular
                   vel_msg.linear.x = 0
                   
               
               
           else: #car does not move if there is no image
               vel_msg.linear.x = 0
               vel_msg.angular.z = 0
               prev_angular = 0



           # PUblish the data
           pub.publish(vel_msg)
           # Sleep the necessary amount of time to keep a 10Hz execution rate
           rate.sleep()


if __name__ == '__main__':
  try:
    d = dist()
    d.track_node()
  except rospy.ROSInterruptException:
          pass
