#!/usr/bin/env python3
import rospy

import numpy as np
import cv_bridge
import cv2
import tensorflow as tf

# import DTROS-related class
from duckietown.dtros import \
    DTROS, \
    DTParam, \
    NodeType, \
    ParamType

# import messages and services
from duckietown_msgs.msg import Twist2DStamped
from sensor_msgs.msg import CompressedImage


class NNController(DTROS):
    def __init__(self, node_name):
        super(NNController, self).__init__(
            node_name=node_name,
            node_type=NodeType.CONTROL
        )

        # Set maximum linear speed
        self.v_max = DTParam('~v_max', param_type=ParamType.FLOAT)

        # Image gausian blur mask
        self.image_blur_ksize = DTParam('~image_blur_ksize', param_type=ParamType.DICT)

        # Image resize
        self.image_resize =  DTParam('~image_resize', param_type=ParamType.DICT)

        self.cvbridge = cv_bridge.CvBridge()

        # Subscribe to image topic
        self.image_sub = rospy.Subscriber('~image/in/compressed', CompressedImage, self.callback, queue_size=1)
        
        # Publishers
        self.control_pub = rospy.Publisher('~car_cmd', Twist2DStamped, queue_size=1)
        self.img_pub = rospy.Publisher('~image1/out/compressed', CompressedImage, queue_size=1)

        # Model NN
        #self.model = tf.keras.models.load_model('/code/catkin_ws/src/SpinakerV1/assets/nn_models/model-v1.h5')
        self.model = tf.keras.models.load_model('/code/catkin_ws/src/SpinakerV1/assets/nn_models/model-2023-05-19.h5')

        # Message to publish
        self.twist = Twist2DStamped()

    def callback(self, msg) -> None:
        try:

            # Read image from node
            img_org = self.cvbridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Image processing            
            # Crop image
            image = img_org[300:500, :, :]

            # Filter image
            image = cv2.cvtColor(image, cv2.COLOR_BGR2YUV)
            image = cv2.GaussianBlur(image, (self.image_blur_ksize.value['x'], self.image_blur_ksize.value['y']), 0)

            # Resize image
            image = cv2.resize(image, (self.image_resize.value['width'], self.image_resize.value['height']))
            
            # Normalize image for NN  
            image = (image - np.min(image)) / (np.max(image) - np.min(image))

            # Predict controll omega
            self.twist.omega = float(self.model.predict(np.expand_dims(image, axis=0), verbose=0))

            # Set linear speed
            self.twist.v = self.v_max.value
            
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
    
    def on_shutdown(self):
        # Send stop command
        self.control_pub.publish(Twist2DStamped(omega=0.0, v=0.0))
        rospy.sleep(1)
        self.control_pub.publish(Twist2DStamped(omega=0.0, v=0.0))
        rospy.sleep(1)
        self.control_pub.publish(Twist2DStamped(omega=0.0, v=0.0))
        # Wait....
        rospy.sleep(1)
        rospy.loginfo("Stop NN controller")

###################################################################################

if __name__ == '__main__':
    some_name_node = NNController(node_name='nn_controller_node')
    rospy.spin()