from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

rospy.init_node('cvexample')
BRIDGE = CvBridge()

def cv_callback(msg):
   cv_image = BRIDGE.compressed_imgmsg_to_cv2(msg)
   do_stuff(cv_image)

def do_stuff(img):
   # get hsv image from opencv
   hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV) 

   # range of colors, found by trial and error
   lower_color_blue_bound = np.array([110,125,125]) 
   upper_color_blue_bound = np.array([130,255,255]) 
  
   # find pixels in range bounded by BGR color bounds
   mask = cv.inRange(hsv, lower_color_blue_bound, upper_color_blue_bound)

   # find pixels that are in both mask AND original img
   masked_img = cv.bitwise_and(img, img, mask=mask)
   ros_img = BRIDGE.cv2_to_imgmsg(masked_img)
   rgb_pub.publish(ros_img)
   hsv_pub.publish(hsv)    

# subscriber/publishers
cam_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, cv_callback)
rgb_pub = rospy.Publisher('/camera/rgb/masked', Image, queue_size=1)
hsv_pub = rospy.Publisher('/camera/rgb/hsv_image', Image, queue_size=1)

# control loop
while not rospy.is_shutdown():
    rospy.sleep(10)
