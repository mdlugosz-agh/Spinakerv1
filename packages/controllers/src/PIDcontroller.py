#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from duckietown_msgs.msg import Twist2DStamped
from numpy import rad2deg as rad2deg

class PIDController:
    def __init__(self):
    
        # controller coefficients
        self.Kp = 1.0
        self.Ki = 0.0
        self.Kd = 0.0

    def run(self, v_0, theta_ref, theta_hat, prev_e, prev_int, delta_t):
        """
        Args:
            v_0 (:double:) linear Duckiebot speed (given).
            theta_ref (:double:) reference heading pose
            theta_hat (:double:) the current estiamted theta.
            prev_e (:double:) tracking error at previous iteration.
            prev_int (:double:) previous integral error term.
            delta_t (:double:) time interval since last call.
        returns:
            v_0 (:double:) linear velocity of the Duckiebot 
            omega (:double:) angular velocity of the Duckiebot
            e (:double:) current tracking error (automatically becomes prev_e_y at next iteration).
            e_int (:double:) current integral error (automatically becomes prev_int_y at next iteration).
        """
        
        # Tracking error
        e = theta_ref - theta_hat

        # integral of the error
        e_int = prev_int + e*delta_t

        # anti-windup - preventing the integral error from growing too much
        e_int = max(min(e_int, 1), -1)

        # derivative of the error
        e_der = (e - prev_e)/delta_t

        # PID controller for omega
        omega = self.Kp*e + self.Ki*e_int + self.Kd*e_der
        
        rospy.loginfo(f"Delta time : {delta_t} \nE : {rad2deg(e)} \nE int : {e_int} \nPrev e : {prev_e} \nomega : {omega} \nTheta hat: {rad2deg(theta_hat)}")
        
        return v_0, omega, e, e_int
    
###################################################################################

class WrapperController:
    def __init__(self):
        
        self.controller = PIDController()

        # Subscribe to error topic
        self.error_sub = rospy.Subscriber('~error', Float32, self.callback, queue_size=1)
        
        # Publishers
        self.control_pub = rospy.Publisher('~car_cmd', Twist2DStamped, queue_size=1)

        self.twist = Twist2DStamped()
        
        self.rate = rospy.Rate(30)

        # Maximum angular and linear speed
        self.omega_max = rospy.get_param('~omega_max', 8.0)
        self.v_max = rospy.get_param('~v_max', 0.4)

        self.prev_e = 0.0
        self.prev_int = 0.0

        # Clean up before stop
        rospy.on_shutdown(self.cleanup)

    def callback(self, msg) -> None:
        try:

            # Compute controll
            self.twist.v, self.twist.omega, self.prev_e, self.prev_int = self.controller.run(
                    v_0         = self.v_max, 
                    theta_ref   = 0.0,
                    theta_hat   = msg.data,  
                    prev_e      = self.prev_e, 
                    prev_int    = self.prev_int, 
                    delta_t     = float(1/30))

            # Scalling output form controller
            self.twist.omega = self. omega_max * self.twist.omega
            
            # Add header timestamp
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

###################################################################################

def main():
        # Create node
    rospy.init_node("pid_controller_node")
    node = WrapperController()

    while not rospy.is_shutdown():
        rospy.spin()

if __name__=="__main__":
    main()
