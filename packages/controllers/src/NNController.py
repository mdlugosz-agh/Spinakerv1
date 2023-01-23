#!/usr/bin/env python3
import rospy
from duckietown_msgs.msg import Twist2DStamped
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv_bridge
import cv2

from tensorflow.python.keras.models import load_model

class NNController:
    def __init__(self):
        self.cvbridge = cv_bridge.CvBridge()

        # Subscribe to image topic
        self.image_sub = rospy.Subscriber('~image/in/compressed', CompressedImage, self.callback, queue_size=1)
        
        # Publishers
        self.control_pub = rospy.Publisher('~car_cmd', Twist2DStamped, queue_size=1)

        # Model NN
        self.model = load_model('/code/catkin_ws/src/SpinakerV1/assets/nn_models/model-e30-b5.h5')

        self.twist = Twist2DStamped()
        
        self.rate = rospy.Rate(30)

        # Clean up before stop
        rospy.on_shutdown(self.cleanup)

    def callback(self, msg) -> None:
        #global sess
        #global graph

        try:

            # Read image from node
            image = np.asarray(self.cvbridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8'))

            # Crop image
            image = image[220:500, :, :]

            # Filter image
            image = cv2.cvtColor(image, cv2.COLOR_RGB2YUV)
            image = cv2.GaussianBlur(image, (3, 3), 0)

            # Resize image to size which is recomended by NVIDIA
            image = cv2.resize(image, (200, 66))
            # image = (image - np.min(image)) / (np.max(image) - np.min(image))


            # Compute controll omega
            #with graph.as_default():
            #    set_session(sess)
            self.twist.omega = self.model.predict(np.expand_dims(image, axis=0))
                
            # Set linear speed
            self.twist.v = 0.15
            # Set timestamp
            self.twist.header.stamp = rospy.Time.now()
            
            # Publish controlll
            self.control_pub.publish(self.twist)

            # Publish transformed image
            self.image_pub = rospy.Publisher('~image/out/compressed', CompressedImage, queue_size=1)

            out_image = self.cvbridge.cv2_to_compressed_imgmsg(image, 'jpg')
            out_image.header.stamp = rospy.Time.now()

            self.image_pub.publish(out_image)

            rospy.loginfo("twis.omega: {}".format(self.twist.omega))

        except Exception as e:
            rospy.logerr("Error: {0}".format(e))
           
    def cleanup(self):
        # Send stop command
        self.control_pub.publish(Twist2DStamped(omega=0.0, v=0.0))
        # Wait....
        rospy.sleep(1)
        rospy.loginfo("Stop NNController")
        pass

#sess = tf.Session()
#graph = tf.get_default_graph()
#set_session(sess)

# Create node
rospy.init_node("nn_controller_node")
node = NNController()

while not rospy.is_shutdown():
    rospy.spin()
