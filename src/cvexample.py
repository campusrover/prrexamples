from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

rospy.init_node('cvexample')
BRIDGE = CvBridge()

def cv_callback(msg):
   cv_image = BRIDGE.compressed_imgmsg_to_cv2(msg)
   do_stuff(cv_image)

cam_sub = rospy.Subscriber('/camera/rgb/image_raw’, Image, cv_callback)

