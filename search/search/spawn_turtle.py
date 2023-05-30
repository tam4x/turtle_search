import rclpy
from turtlesim.srv import Spawn
from turtle_interfaces.msg import Poses
from rclpy.node import Node
from functools import partial
import random
from turtle_interfaces.msg import Killer

class SpawnClient(Node):
    def __init__(self):
        super().__init__("spawn_client")
        self.killed = Killer()
        self.client_ = self.create_client(Spawn, "spawn")
        self.publisher = self.create_publisher(Poses, "turtle_poses", 10)
        self.subscriber = self.create_subscription(Killer, "killed_turtle", self.killed_callback, 10)
        self.wait_service()
        self.request = Spawn.Request()
        self.spawns = Poses()
        self.request_spawn()
        self.timer2 = self.create_timer(0.05, self.publish_spawns)
        self.timer3 = self.create_timer(0.05, self.delete_killed)
        self.timer = self.create_timer(3.0, self.request_spawn)
        
        

    def killed_callback(self, msg):
        self.killed.name = msg.name
        self.killed.kill = msg.kill

    def delete_killed(self):
        if self.killed.kill:
            try:
                i = self.spawns.name.index(self.killed.name)
                del self.spawns.x[i]
                del self.spawns.y[i]
                del self.spawns.theta[i]
                del self.spawns.name[i]
            except ValueError:
                pass

    def wait_service(self):
        while not self.client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server")
    
    def do_request(self, x , y, theta):
        self.request.x = x
        self.request.y= y
        self.request.theta = theta
        self.spawns.x.append(x)
        self.spawns.y.append(y)
        self.spawns.theta.append(theta)
        future = self.client_.call_async(self.request)
        future.add_done_callback(partial(self.callback_call_spawn, x = x, y = y, theta = theta))

    def callback_call_spawn(self, future, x, y, theta):
        try:
            response = future.result()
            self.spawns.name.append(response.name)

        except Exception as e:

            self.get_logger().error("Service call failed %r" % (e,))

            
    def get_spawn(self):
        x = random.uniform(1, 10)
        y = random.uniform(1, 10)
        theta = 0.0
        return x, y, theta
    
    def request_spawn(self):

        x, y, theta = self.get_spawn()
        self.do_request(x, y, theta)

    def publish_spawns(self):
        self.publisher.publish(self.spawns)





def main(args=None):
    rclpy.init(args = args)
    node = SpawnClient()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()