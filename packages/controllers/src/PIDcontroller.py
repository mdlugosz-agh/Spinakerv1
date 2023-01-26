#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from duckietown_msgs.msg import Twist2DStamped

class PIDController:
    def __init__(self):
        
        # Subscribe to error topic
        self.image_sub = rospy.Subscriber('~error', Float32, self.callback, queue_size=1)
        
        # Publishers
        self.control_pub = rospy.Publisher('~car_cmd', Twist2DStamped, queue_size=1)

        self.twist = Twist2DStamped()
        
        self.rate = rospy.Rate(10)

        # Clean up before stop
        rospy.on_shutdown(self.cleanup)

    def callback(self, msg) -> None:
        try:

            # Controller
            self.twist.v = 0.4
            self.twist.omega = (8.0/3.2) * (-float(msg.data)/100) 
            self.twist.header.stamp = rospy.Time.now()
            
            # Publish controlll
            self.control_pub.publish(self.twist)

        except Exception as e:
            rospy.logerr("Error: {0}".format(e))
           
    def cleanup(self):
        # Send stop command
        self.control_pub.publish(Twist2DStamped(omega=0.0, v=0.0))
        # Wait....
        rospy.sleep(1)
        rospy.loginfo("Stop PIDController")
        pass

# Create node
rospy.init_node("pid_controller_node")
node = PIDController()

while not rospy.is_shutdown():
    rospy.spin()