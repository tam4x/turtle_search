import rclpy
import math
import random
from rclpy.node import Node
from turtlesim.msg import Pose
from functools import partial
from turtle_interfaces.msg import Brother
from turtlesim.srv import Kill
from turtle_interfaces.msg import Poses
from turtle_interfaces.msg import Killer

class KillClient(Node):
    def __init__(self):
        super().__init__("kill_turtle")
        self.origin = Pose()
        self.brother = Brother()
        self.poses = Poses()
        self.client = self.create_client(Kill, "kill")
        self.subscriber1 = self.create_subscription(Pose, "turtle1/pose", self.pose_callback, 10)
        self.subscriber2 = self.create_subscription(Brother, "brother_turtle", self.brother_callback, 10)
        self.subscriber3 = self.create_subscription(Poses, "turtle_poses", self.poses_callback, 10)
        self.publisher = self.create_publisher(Killer, "killed_turtle", 10)
        self.killed = Killer()
        self.wait_service()
        self.request = Kill.Request()
        self.timer = self.create_timer(0.05, self.request_kill)

    def pose_callback(self, msg):
        self.origin.x = msg.x
        self.origin.y = msg.y
        self.origin.theta = msg.theta
        

    def brother_callback(self, msg):
        self.brother.distance = msg.distance
        self.brother.x = msg.x
        self.brother.y = msg.y
        self.brother.theta = msg.theta
        self.brother.name = msg.name

    def poses_callback(self, msg):
        self.poses.x = msg.x
        self.poses.y = msg.y
        self.poses.theta = msg.theta
        self.poses.name = msg.name

    def wait_service(self):
        while not self.client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server")
    
    def do_request(self, name):
        self.request.name = name
        future = self.client.call_async(self.request)
        future.add_done_callback(partial(self.callback_call_spawn, name = name))

    def callback_call_spawn(self, future, name):
        try:
            response = future.result()
        except Exception as e:

            self.get_logger().error("Service call failed %r" % (e,))

    
    def request_kill(self):
        if (self.brother.x-0.2 < self.origin.x < self.brother.x+0.2) and (self.brother.y-0.2 < self.origin.y < self.brother.y+0.2):
            
            self.do_request(self.brother.name)
            self.killed.name = self.brother.name
            self.killed.kill = True
        else:
            self.killed.name = ""
            self.killed.kill = False

        self.publisher.publish(self.killed)

def main(args=None):
    rclpy.init(args = args)
    node = KillClient()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()