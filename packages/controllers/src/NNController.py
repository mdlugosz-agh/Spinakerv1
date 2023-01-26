#!/usr/bin/env python3
import rospy
from duckietown_msgs.msg import Twist2DStamped
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv_bridge
import cv2

import tensorflow as tf

class NNController:
    def __init__(self):
        self.cvbridge = cv_bridge.CvBridge()

        # Subscribe to image topic
        self.image_sub = rospy.Subscriber('~image/in/compressed', CompressedImage, self.callback, queue_size=1)
        
        # Publishers
        self.control_pub = rospy.Publisher('~car_cmd', Twist2DStamped, queue_size=1)
        self.img_pub = rospy.Publisher('~image/out/compressed', CompressedImage, queue_size=1)

        # store the session object from the main thread
        # self.session = tf.compat.v1.keras.backend.get_session()

        # Model NN
        self.model = tf.keras.models.load_model('/code/catkin_ws/src/SpinakerV1/assets/nn_models/model-2022-01-25-01-cv2.h5')

        self.twist = Twist2DStamped()
        
        # Clean up before stop
        rospy.on_shutdown(self.cleanup)

    def callback(self, msg) -> None:
        try:

            # Read image from node
            img_org = self.cvbridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            image = img_org

            # Crop image
            image = image[300:500, :, :]

            # Filter image
            image = cv2.cvtColor(image, cv2.COLOR_RGB2YUV)
            image = cv2.GaussianBlur(image, (3, 3), 0)

            # Resize image to size which is recomended by NVIDIA
            image = cv2.resize(image, (80, 60))
            image = (image - np.min(image)) / (np.max(image) - np.min(image))

            # Compute controll omega
            
            #tf.compat.v1.keras.backend.set_session(self.session)
            self.twist.omega = self.model.predict(np.expand_dims(image, axis=0), verbose=0)

            # Set linear speed
            self.twist.v = 0.4
            # Set timestamp
            self.twist.header.stamp = rospy.Time.now()
            
            # Publish controlll
            self.control_pub.publish(self.twist)

            # Publish transformed image
            # Add error value to image
            cv2.putText(img_org, 
                "Controll= " + str(self.twist.omega), 
                org=(10,20), fontFace=cv2.FONT_HERSHEY_SIMPLEX, color=(0,255,0), fontScale=0.5, 
                thickness=1, lineType=cv2.LINE_AA)
            out_image = self.cvbridge.cv2_to_compressed_imgmsg(img_org, 'jpg')
            out_image.header.stamp = rospy.Time.now()
            self.img_pub.publish(out_image)

        except Exception as e:
            rospy.logerr("Error: {0}".format(e))
           
    def cleanup(self):
        # Send stop command
        self.control_pub.publish(Twist2DStamped(omega=0.0, v=0.0))
        # Wait....
        rospy.sleep(1)
        rospy.loginfo("Stop NNController")
        pass

# Create node
rospy.init_node("nn_controller_node")
node = NNController()

while not rospy.is_shutdown():
    rospy.spin()