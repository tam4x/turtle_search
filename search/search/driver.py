import rclpy
import math
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtle_interfaces.msg import Brother

class Driver(Node):
    def __init__(self):
        super().__init__("driver_node")
        self.subcriber_1 = self.create_subscription(Brother, "brother_turtle", self.brother_callback, 10)
        self.subscriber_2 = self.create_subscription(Pose, "turtle1/pose", self.pose_callback, 10)
        self.publisher = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.direction = Twist()
        self.direction.linear.x = 0.0
        self.direction.linear.y = 0.0
        self.direction.linear.z = 0.0
        self.direction.angular.x = 0.0
        self.direction.angular.y = 0.0
        self.direction.angular.z = 0.0

        self.brother = Brother()
        self.origin = Pose()
        self.timer = self.create_timer(0.05, self.control)
       
    def brother_callback(self, msg):
        self.brother.distance = msg.distance
        self.brother.x = msg.x
        self.brother.y = msg.y
        self.brother.theta = msg.theta
        self.brother.name = msg.name
        
    def pose_callback(self, msg):
        self.origin.x = msg.x
        self.origin.y = msg.y
        self.origin.theta = msg.theta
        

    def control(self):
        try:
            w = (self.brother.y - self.origin.y)
            z = (self.brother.x - self.origin.x)
            degree_between = math.atan2(w, z)

            if -135.4 < math.degrees(degree_between) < -134.9:
                pass
            
            else:

                if degree_between-0.3 < self.origin.theta < degree_between+0.3:
                    self.direction.linear.x = 2.0
                    self.direction.angular.z = 0.0

                else:
                    if self.brother.x > self.origin.x:
                        self.direction.linear.x = 1.0
                        self.direction.angular.z = -2.0
                    else:
                        self.direction.linear.x = 1.0
                        self.direction.angular.z = 2.0

                self.publisher.publish(self.direction)
                distance = self.brother.distance
            

        except ZeroDivisionError:
            pass



def main(args=None):
    rclpy.init(args = args)
    node = Driver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()