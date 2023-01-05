#!/usr/bin/env python3
import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32
from duckietown_msgs import msg

class LateralPositionError:
    def __init__(self):
        self.cvbridge = cv_bridge.CvBridge()

        # Subscribe to image topic
        self.image_sub = rospy.Subscriber('~image/in/compressed', CompressedImage, self.callback, queue_size=1)
        
        # Publishers
        # Lateral error
        self.error_pub = rospy.Publisher('~error/lateral', Float32, queue_size=1)

        # Transformed image
        self.image_pub = rospy.Publisher('~image/out/compressed', CompressedImage, queue_size=1)

        # Messages
        self.error = Float32()

        # Clean up before stop
        rospy.on_shutdown(self.cleanup)

    def callback(self, msg) -> None:
        try:
            # Read image from node
            image = self.cvbridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convert to HSV color space
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Find red line
            # lower boundary RED color range values; Hue (0 - 10)
            lower1 = np.array([0, 100, 20])
            upper1 = np.array([10, 255, 255])

            # upper boundary RED color range values; Hue (160 - 180)
            lower2 = np.array([160,100,20])
            upper2 = np.array([179,255,255])
            
            lower_mask = cv2.inRange(hsv, lower1, upper1)
            upper_mask = cv2.inRange(hsv, lower2, upper2)
            
            full_mask = lower_mask + upper_mask
            #result = cv2.bitwise_and(image, image, mask=full_mask)

            # Cut image, only consider 75% of image area
            h, w, _ = image.shape
            search_top = int(3*h/4)
            search_bot = int(search_top + 40)
            full_mask[0:search_top, 0:w] = 0
            full_mask[search_bot:h, 0:w] = 0

            # Find center of mass detected red line
            M = cv2.moments(full_mask)
            cx = 0.0
            cy = 0.0
            if M['m00'] > 0 :
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

            # Publish error
            self.error = cx - w/2
            self.error_pub.publish(self.error)
            
            # Publish transformed image
            # Add circle in point of center of mass
            cv2.circle(image, (int(cx), int(cy)), 20, (0,255,0), -1)
            # Add error value to image
            cv2.putText(image, "Error= " + str(self.error), 
            org=(10,20), fontFace=cv2.FONT_HERSHEY_SIMPLEX, color=(0,255,0), fontScale=0.5, 
                thickness=1, lineType=cv2.LINE_AA)
            out_image = self.cvbridge.cv2_to_compressed_imgmsg(image, 'jpg')
            out_image.header.stamp = rospy.Time.now()
            self.image_pub.publish(out_image)
           

        except cv_bridge.CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def cleanup(self):
        pass
        
# Create node
rospy.init_node("lateral_position_error_node")
node = LateralPositionError()

while not rospy.is_shutdown():
    rospy.spin()