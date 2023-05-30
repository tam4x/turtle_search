import launch_ros
import launch.actions
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess

def generate_launch_description():
    ld = LaunchDescription()
    turtlesim = Node(
        package = "turtlesim",
        executable = "turtlesim_node"
    )
    spawn_turtle = Node(
        package = "search",
        executable= "spawn_turtle"
    )
    distance_calculation = Node(
        package = "search",
        executable = "distance_calculation"
    )
    drivers = Node(
        package = "search",
        executable = "drivers"
    )
    killers = Node(
        package = "search",
        executable = "killers"
    )
    ld.add_action(turtlesim)
    ld.add_action(spawn_turtle)
    ld.add_action(distance_calculation)
    ld.add_action(drivers)
    ld.add_action(killers)


    return ld
