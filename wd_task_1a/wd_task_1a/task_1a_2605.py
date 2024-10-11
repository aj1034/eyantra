#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill
from turtlesim.srv import SetPen
import math
import time


class Turtle(Node):
    def __init__(self, name):
        super().__init__(name)
        self.name = name
        self.publisher = self.create_publisher(Twist, f'/{self.name}/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, f'/{self.name}/pose', self.pose_callback, 10)
        self.pose = Pose()

    def pose_callback(self, data):
        self.pose = data
        # self.get_logger().info(f"Pose updated: x={self.pose.x}, y={self.pose.y}, theta={self.pose.theta}")

    def go_to_setpoint_straightline(self, x_d, y_d, theta_d):
        """Method to move the turtle in a straight line towards a specified setpoint"""
        goal = Pose()
        goal.x = x_d
        goal.y = y_d
        goal.theta = theta_d

        distance_tolerance = 0.1
        angle_tolerance = 0.01
        new_vel = Twist()
        rate = self.create_rate(100)  # 10 Hz

        while True:
            distance_to_goal = math.sqrt((goal.x - self.pose.x) ** 2 + (goal.y - self.pose.y) ** 2)
            angle_to_goal = math.atan2(goal.y - self.pose.y, goal.x - self.pose.x)

            # Normalize angle to the range (-pi, pi)
            angle_error = (angle_to_goal - self.pose.theta + math.pi) % (2 * math.pi) - math.pi

            kp = 0.7  # Adjust this value for testing

            if distance_to_goal >= distance_tolerance:
                self.get_logger().info("hi if2")
                new_vel.linear.x = kp * distance_to_goal
                new_vel.angular.z = 0.0
                self.get_logger().info("hshsh")
            else:
                new_vel.linear.x = 0.0
                new_vel.angular.z = 0.0
                self.get_logger().info("Goal Reached")
                break  # Exit the loop when the goal is reached

            self.publisher.publish(new_vel)
            rclpy.spin_once(self)
            # rate.sleep()  # Maintain the loop rate

    def move_in_circle(self, v, r):
        """Method to move the turtle in a circle with specified radius"""
        msg = Twist()

        msg.linear.x = float(v)
        msg.angular.z = -float(v) / float(r)
        self.publisher.publish(msg)

        time_for_rotation = 2 * math.pi * r / v
        time.sleep(time_for_rotation)  # Consider using a timer instead
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)

    def spawn_turtle(self, x, y, theta, unique_name = None):
        """Spawn a new turtle at (x, y) with orientation theta."""
        client = self.create_client(Spawn, 'spawn')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{self.name} waiting for /spawn service...')

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = unique_name if unique_name else self.name

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'{self.name} successfully spawned at ({x}, {y}) with theta {theta}.')
        else:
            self.get_logger().error(f'{self.name} failed to spawn turtle.')

    def kill_turtle(self, unique_name = None):
        """Kill the turtle."""
        client = self.create_client(Kill, 'kill')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{self.name} waiting for /kill service...')

        request = Kill.Request()
        request.name = unique_name if unique_name else self.name

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'{self.name} successfully killed.')
        else:
            self.get_logger().error(f'{self.name} failed to kill turtle.')
    
    from turtlesim.srv import SetPen

    def disable_pen(self, unique_name = None):
        """Disables the pen for the turtle, preventing it from drawing."""
        client = self.create_client(SetPen, f'/{self.name}/set_pen')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{self.name} waiting for /set_pen service...')
        
        request = SetPen.Request()
        request.r = 0    # Set to any color, doesn't matter here
        request.g = 0
        request.b = 0
        request.width = 0
        request.off = 1  # Turn off the pen (pen up)
        request.name = unique_name if unique_name else self.name

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'{self.name} pen disabled.')
        else:
            self.get_logger().error(f'{self.name} failed to disable pen.')


def sketch_straight_line(t, x_i, y_i, x_f, y_f):
    spawn_angle = math.atan2((y_f - y_i), (x_f - x_i))
    t.spawn_turtle(x_i, y_i, spawn_angle)
    # Move in a loop until the goal is reached
    t.go_to_setpoint_straightline(x_f, y_f, spawn_angle)
    print("hi")
    t.kill_turtle()


def sketch_propellers(t, x_c, y_c, radius):
    t.spawn_turtle(x_c - radius, y_c, math.pi / 2)
    t.move_in_circle(8, radius)
    t.kill_turtle()


def sketch_drone():
    # Create 12 turtle instances
    t = []
    for i in range(13):
        turtle_name = f"turtle_{i}"

        t.append(Turtle(name=turtle_name))

    # Sketching the center square
    sketch_straight_line(t[0], 3.0, 5.0, 5.0, 7.0)
    sketch_straight_line(t[1], 5.0, 7.0, 7.0, 5.0)
    sketch_straight_line(t[2], 7.0, 5.0, 5.0, 3.0)
    sketch_straight_line(t[3], 5.0, 3.0, 3.0, 5.0)

    # Sketching the 4 propellers
    sketch_propellers(t[4], 2.0, 2.0, 1.0)
    sketch_propellers(t[5], 2.0, 8.0, 1.0)
    sketch_propellers(t[6], 8.0, 8.0, 1.0)
    sketch_propellers(t[7], 8.0, 2.0, 1.0)

    # Sketching the lines to complete the frame
    sketch_straight_line(t[8], 2.0, 2.0, 4.0, 4.0)
    sketch_straight_line(t[9], 2.0, 8.0, 4.0, 6.0)
    sketch_straight_line(t[10], 8.0, 8.0, 6.0, 6.0)
    # sketch_straight_line(t[11], 8.0, 2.0, 6.0, 4.0)
    # t[11].go_to_setpoint_straightline(8.0,2.0)
    # t[11].diable_pen
    # t[11].go_to_setpoint_straightline(6.0,4.0,5.0,5.0)


    # t[12].spawn_turtle(5.0,5.0,0.0)


def main(args=None):
    rclpy.init(args=args)
    # Call the function to sketch the drone
    try:
        sketch_drone()
    except KeyboardInterrupt:
        pass  # Handle manual interruption with Ctrl+C
    finally:
        rclpy.shutdown()  # Properly shut down ROS 2


if __name__ == '__main__':
    main()
