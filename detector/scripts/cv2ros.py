import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

cap = cv2.VideoCapture(2) 

class ImagePublisher:
    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic", Image, queue_size=10)
        self.bridge = CvBridge()
 
    def publish_image(self):
        ret,image = cap.read()
        image = image[:,:(int)((image.shape[1])/2)]
        msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.image_pub.publish(msg)
 
if __name__ == '__main__':
    rospy.init_node('image_publisher')
    image_publisher = ImagePublisher()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        image_publisher.publish_image()
        rate.sleep()
