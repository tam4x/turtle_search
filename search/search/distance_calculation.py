import rclpy
import math
from rclpy.node import Node
from turtle_interfaces.msg import Poses
from turtlesim.msg import Pose
from turtle_interfaces.msg import Brother
import time
class DistanceCalculation(Node):
    def __init__(self):
        super().__init__("distance_calculation")
        self.poses = Poses()
        self.origin_pose = Pose()
        self.subscriber_2 = self.create_subscription(Pose, "turtle1/pose", self.origin_turtle_callback, 10)
        self.subscriber_ = self.create_subscription(Poses, "turtle_poses", self.turtle_poses_callback, 10)
        self.publisher = self.create_publisher(Brother, "brother_turtle", 10)
        self.timer = self.create_timer(0.05, self.find_nearest_turtle)

    def turtle_poses_callback(self, msg):
        self.poses.x = msg.x
        self.poses.y = msg.y
        self.poses.theta = msg.theta
        self.poses.name = msg.name

    def origin_turtle_callback(self, msg):
        self.origin_pose.x = msg.x
        self.origin_pose.y = msg.y
        self.origin_pose.theta = msg.theta
        self.origin_pose.linear_velocity = msg.linear_velocity
        self.origin_pose.angular_velocity = msg.angular_velocity
    
    def find_nearest_turtle(self):
        distances = []
        p1 = (self.origin_pose.x, self.origin_pose.y)
        try:
            for i in range(0, len(self.poses.x)):
                p2 = (self.poses.x[i], self.poses.y[i])
                distances.append(math.dist(p1, p2))

            d = min(distances)
            i = distances.index(d)
            nearest_turtle = Brother()
            
            nearest_turtle.distance = d
            nearest_turtle.x = self.poses.x[i]
            nearest_turtle.y = self.poses.y[i]
            try:
                nearest_turtle.name = self.poses.name[i]
            except IndexError:
                nearest_turtle.name = "turtle2"

            nearest_turtle.theta = self.poses.theta[i]

            self.publisher.publish(nearest_turtle)

        except ValueError or IndexError:
            pass


def main(args=None):
    rclpy.init(args = args)
    node = DistanceCalculation()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()