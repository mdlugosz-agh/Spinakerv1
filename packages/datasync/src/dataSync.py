#!/usr/bin/env python3
import rospy
import message_filters
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import Twist2DStamped

class DataSync:

    def __init__(self):
        

        # Subscribe to input image topic
        self.image_sub  = message_filters.Subscriber('~in/image/compressed', CompressedImage)
        self.car_cmd    = message_filters.Subscriber('~in/car_cmd', Twist2DStamped)

        # Publisher
        self.image_pub = rospy.Publisher('~out/image/compressed', CompressedImage, queue_size=1)
        self.car_cmd_pub = rospy.Publisher('~out/car_cmd', Twist2DStamped, queue_size=1)

        # TimeSynchronizer
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.car_cmd], 
            slop = 1, 
            queue_size=10)

        self.ts.registerCallback(self.callback)

        # Clean up before stop
        rospy.on_shutdown(self.cleanup)

    def callback(self, msg_image, msg_car_cmd) -> None:
        
        # Synchronize message image with car_cmd
        msg_car_cmd.header.stamp = msg_image.header.stamp
        
        # Publish synchronised messages
        self.image_pub.publish(msg_image)
        self.car_cmd_pub.publish(msg_car_cmd)
        
    def cleanup(self):
        pass

# Create node
rospy.init_node("datasync_node")
node = DataSync()

while not rospy.is_shutdown():
    rospy.spin()