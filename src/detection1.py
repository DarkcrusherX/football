
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped

lower1 = np.array([0, 120, 70])    #limits for red object detection
upper1 = np.array([10, 255, 255])

lower2 = np.array([170, 120, 70])  
upper2 = np.array([180, 255, 255])

bridge = CvBridge()
rospy.init_node('detection2', anonymous=True)

publish_mid = rospy.Publisher('/player1/midpoint', PoseStamped,queue_size=20)
midpoint = PoseStamped()
def show_image(img):
  midpoint = PoseStamped()
  cv2.namedWindow("Image Window2")
  img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
  frame_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
  mask1 = cv2.inRange(frame_HSV, lower1, upper1)
  mask2 = cv2.inRange(frame_HSV, lower2, upper2)
  mask = mask1 +mask2
  mask_Open = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((10, 10)))
  mask_Close = cv2.morphologyEx(mask_Open, cv2.MORPH_CLOSE, np.ones((20, 20))) 
  mask_Perfect = mask_Close
  conts, h = cv2.findContours(mask_Perfect.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
  for c in conts:  
    x, y, w, h = cv2.boundingRect(c)
    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
    cv2.circle(img, (x + int(w*0.5), y + int(h*0.5)), 4, (0,0,255), -1)
    midpoint.pose.position.x = x + int(w*0.5)    
    midpoint.pose.position.y = y + int(h*0.5)  
  publish_mid.publish(midpoint)
  cv2.imshow("Image Window2", img)
  cv2.waitKey(3)


def image_callback(img_msg):
  #rospy.loginfo(img_msg.header)
  cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
  show_image(cv_image)   

sub_image = rospy.Subscriber("/player1/usb_cam/image_raw", Image, image_callback)

rospy.spin()
