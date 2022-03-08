import argparse
import random
import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from freyja_msgs.msg import ReferenceState

max_vel = 1
max_angle = 1


class CmdvelPublisher2D(Node):
    def __init__(self):
        super().__init__('freyja_reference_state_publisher2D')
        self.publisher_ = self.create_publisher(ReferenceState, '/robomaster_0/reference_state', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.x_vel = random.uniform(-max_vel, max_vel)
        self.z_angle = random.uniform(-max_vel, max_vel)

    def timer_callback(self):
        self.x_vel = random.uniform(-max_vel, max_vel)
        self.z_angle = random.uniform(-max_angle, max_angle)
        # x_vel=0.2
        msg = ReferenceState(vn=self.x_vel, an=self.z_angle)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)
        self.i += 1

def main():

    rclpy.init(args=None)

    cmd_vel_publisher_2d = CmdvelPublisher2D()
    rclpy.spin(cmd_vel_publisher_2d)

    cmd_vel_publisher_2d.destroy_node()
    rclpy.shutdown()


# if __name__ == '__main__':
#     parser = argparse.ArgumentParser(description="process publishing data into cmd_vel")
#     parser.add_argument('--rate',
#                         action='store',
#                         default=1,
#                         help='Rate of cmd_vel publishing')
#     parser.add_argument('--publish_count',
#                         action='store',
#                         default=100,
#                         help='Total number of messages to be passed into the topic at the publishing rate')                        
#     parser.add_argument('--max_vel',
#                         action='store',
#                         default=1.0,
#                         help='maximum value of velocity passed to the robot')
#     parser.add_argument('--max_angle', 
#                         action='store',
#                         default=1,
#                         help='maximum value of z-angle passed to the robot')



# msg = f"{{linear: {{x: {x_vel}, y: 0.0, z: 0.0}}, anuglar: {{x: 0.0, y: 0.0, z: 0.0}}}}"
# print(msg)

# publish_command = f"ros2 topic pub --rate {args.rate} /robomaster_0/cmd_vel geometry_msgs/msg/Twist \"{msg}\""
# process = subprocess.Popen(publish_command.split(), stdout=subprocess.PIPE)
# output, error = process.communicate()
