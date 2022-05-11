import argparse
import random
from git import refresh
import numpy as np
import torch
from bisect import bisect_left
import collections

import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from freyja_msgs.msg import ReferenceState, CurrentState

from utils import train_differential, predict_with_uncertainty, train_val_test_split, load_initial_models

max_vel = 1
max_angle = 1

input_dim = 2
n_visible = 1
lag_offset = 0

class ActivePublisher2D(Node):
    def __init__(self):
        super().__init__('freyja_reference_state_publisher2D')
        self.publisher_ = self.create_publisher(ReferenceState, '/robomaster_0/reference_state', 10)
        # The subscriber subscribes to the Freyja state estimator to obtain information about current positions
        # for collision avoidance with the boundaries
        self.subscriber = self.create_subscription(CurrentState, '/robomaster_0/current_state', self.listener_callback, 10)
        self.timer_period = 0.01  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        # Counter for each iteration of timer_callback. Zeroed after each reference state change
        self.i = 0
        self.j = 0
        self.x_vel, self.y_vel = random.uniform(-max_vel, max_vel), random.uniform(-max_vel, max_vel)
        
        self.z_angle = random.uniform(-max_vel, max_vel)
        self.bound_len = 3

        # Needs to change this when the starting position is far different.
        self.cur_pos = [2,-2, 0]
        # A random interval before the reference state changes. 
        self.rand_interval = 0
        # The period of uncertainty evaluations
        self.eval_period = 50000
        # The period between active sampling
        self.active_sample_period = 50000
        #
        self.cur_samples = []
        #
        self.x_vels, self.y_vels = [], []
        self.samples_per_bin = 3
        
        # Initialise the models
        self.models = load_initial_models(ensemble_size=5)

    # Checking whether the passed reference velocities will cause a clash with the "walls"
    # with separation of self.bound_len * 2

    def will_collide(self, vel, pos, dt):

        return pos[0] + vel * dt > self.bound_len or pos[0] + vel * dt < - self.bound_len

    def _avoid_collision(self, x_vel, y_vel, pos, dt):
        if self.will_collide(x_vel, pos[0], dt):
            x_vel = -x_vel
            if self.will_collide(x_vel, pos[0], dt):
                x_vel *= 0.5
                if self.will_collide(x_vel, pos[0], dt):
                    x_vel *= 0.5
        
        if self.will_collide(y_vel, pos[1], dt):
            y_vel = -y_vel
            if self.will_collide(y_vel, pos[1], dt):
                y_vel *= 0.5
                if self.will_collide(y_vel, pos[1], dt):
                    y_vel *= 0.5
        return x_vel, y_vel
    
    def listener_callback(self, cur_state_msg):

        # self.get_logger().info(f'Receiving: {cur_state_msg.state_vector}')
        self.cur_pos = cur_state_msg.state_vector[:3]
        self.cur_vel = cur_state_msg.state_vector[4:6]
        self.cur_samples.append(self.cur_vel)

    def timer_callback(self):
        if self.j >= self.active_sample_period:
            # reset period check counter
            self.j = 0
            self.x_bins, self.x_prob_dist_inv = self.active_sample(self.cur_samples, self.x_vels, self.models, input_ind=0, plot=False)
            self.y_bins, self.y_prob_dist_inv = self.active_sample(self.cur_samples, self.y_vels, self.models, input_ind=1, plot=False)
            self.bin_size = self.x_bins[1] - self.x_bins[0]

        if self.i >= self.rand_interval:
            # reset counter and threshold
            self.i = 0
            self.rand_interval = self.set_rand_interval()

            x_vel_ind = np.random.choice(range(len(self.x_prob_dist_inv)), p=self.x_prob_dist_inv)
            self.x_vel = self.x_bins[x_vel_ind] + np.random.random() * self.bin_size
            y_vel_ind = np.random.choice(range(len(self.y_prob_dist_inv)), p=self.y_prob_dist_inv)
            self.y_vel = self.y_bins[y_vel_ind] + np.random.random() * self.bin_size

            self.x_vel, self.y_vel = self._avoid_collision(self.x_vel, self.y_vel, self.cur_pos, dt=self.rand_interval*self.timer_period)
            # x_vel=0.2
            self.x_vels.append(self.x_vel)
            self.y_vels.append(self.y_vel)
            
            msg = ReferenceState(vn=self.x_vel, ve=self.y_vel)
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg)

        else:
            self.i += 1

    def reset_vel(self):
        stop_msg = ReferenceState()
        self.publisher_.publish(stop_msg)
    
    def set_rand_interval(self, seed=42, dist="uniform", lower=50, upper=200):
        if dist == "uniform":
            return np.random.randint(low=lower, high=upper)
        elif dist == "gaussian":
            return np.round(np.random.normal(loc=(lower+upper)//2, scale=30))
        else:
            raise ValueError("Distribution type unknown")

    def sample_points_from_bins(self, bins, ref_sample, samples_per_bin=3):
        points = np.zeros((len(bins), samples_per_bin), input_dim)
        for i in range(len(bins)):
            for j in samples_per_bin:
                points[i, j, 0] = bins[i] + np.random.random() * self.bin_size
                points[i, j, 1] = ref_sample
        return points

    def active_sample(self, cur_samples, ref_samples, models, num_bins=100, max_val=1, input_ind=None, plot=False):

        bins = np.linspace(-max_val, max_val, num_bins)
        data_size = len(cur_samples)
        
        X = torch.cat([cur_samples[lag_offset:, input_ind], ref_samples[:data_size-lag_offset, input_ind]], axis=1)[:-1]
        y = torch.diff(cur_samples[lag_offset:, input_ind], dim=0)[n_visible-1:]
        X_train, X_val, X_test, y_train, y_val, y_test = train_val_test_split(X, y)

        for model in models:
            train_differential(model, X, y, X_val, y_val)

        sampled_points = self.sample_points_from_bins(bins, ref_samples[-1])
        sampled_points = sampled_points.reshape(len(bins) * self.samples_per_bin, input_dim)
        y_mean_all, y_std_all = predict_with_uncertainty(models, sampled_points)
        y_uncertainty = y_std_all.reshape(len(bins), self.samples_per_bin)
        new_distribution = y_uncertainty/sum(y_uncertainty)

        return bins, new_distribution

def main():

    rclpy.init(args=None)

    cmd_vel_publisher_2d = ActivePublisher2D()
    rclpy.spin(cmd_vel_publisher_2d)
    cmd_vel_publisher_2d.reset_vel()
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
