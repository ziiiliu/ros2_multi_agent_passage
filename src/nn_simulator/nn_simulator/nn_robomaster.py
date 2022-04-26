import rclpy

import numpy as np
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Twist, TransformStamped
from .simulated_robot import SimulatedRobotBase
from .nn_simulator import NNSimpleSimulator
from robomaster_msgs.msg import WheelSpeed
from std_msgs.msg import String
import time

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

from rclpy.qos import qos_profile_sensor_data

class BaseNN(nn.Module):

    def __init__(self, 
                input_dim=3,
                n_hidden=64,
                n_output=3,
                n_layer=3,):
        super(BaseNN, self).__init__()
        self.include_past = False
        self.input_dim = input_dim
        self.n_hidden = n_hidden

        # Input Layer
        self.input = nn.Linear(input_dim, n_hidden)

        # Constructing the hidden layers with a modulelist
        self.fcs = []
        for i in range(n_layer):
            self.fcs.append(nn.Linear(n_hidden, n_hidden))
        self.fcs = nn.ModuleList(self.fcs)

        # Prediction Layer
        self.predict = nn.Linear(n_hidden, n_output)

class PSNN(BaseNN):
    """
    Params
    ----------
    n_visible:  int, number of past timesteps to pass into the model (including the current timestep).
                n = 1 reduces this model to SimplePredictor
    """
    def __init__(self, n_visible=3, n_output=3, n_layer=3, input_dim=3):
        super(PSNN, self).__init__(n_output=n_output, n_layer=n_layer, input_dim=input_dim)
        self.n_visible = n_visible
        # Here we alter the dimension in the input layer
        self.input = nn.Linear(self.input_dim * (self.n_visible+1), self.n_hidden)

    def forward(self, X):
        res = F.relu(self.input(X))
        for fc in self.fcs:
            res = F.relu(fc(res))
        res = self.predict(res)
        return res


class NNRoboMaster(SimulatedRobotBase):
    MAX_V_LINEAR_X_M_S = 3.5
    MAX_V_LINEAR_Y_M_S = 2.8
    MAX_V_ROT_Z_RAD_S = 10.5

    ROBOT_WHEEL_RADIUS = 0.06
    ROBOT_HALF_BASE_LEN_X = 0.15
    ROBOT_HALF_BASE_LEN_Y = 0.15

    def __init__(
        self, uuid, rigid_body_label, node, initial_position, initial_orientation, n_visible=1, input_dim=3
    ):
        super().__init__(
            uuid, rigid_body_label, node, initial_position, initial_orientation
        )

        self.velocity = np.zeros(input_dim)
        self.input_dim = input_dim

        self.angular_velocity_world = 0.0
        self.n_visible = n_visible
        self.nn_input = torch.zeros(input_dim * (n_visible+1))

        self.velocity_ref = Twist()
        self.velocity_subscription = self.node.create_subscription(
            Twist,
            f"/{self.uuid}/cmd_vel",
            self.velocity_callback,
            qos_profile=qos_profile_sensor_data,
        )
        self.wheelspeed_subscription = self.node.create_subscription(
            WheelSpeed,
            f"/{self.uuid}/cmd_wheels",
            self.wheelspeed_callback,
            qos_profile=qos_profile_sensor_data,
        )

        self.dynamics_model = PSNN(n_visible=20, n_layer=3, n_output=3, input_dim=self.input_dim)
        model_path = ""
        self.dynamics_model.load_state_dict(torch.load(model_path))
        self.dynamics_model.eval()

        len_xy = self.ROBOT_HALF_BASE_LEN_X + self.ROBOT_HALF_BASE_LEN_Y
        self.dyn_inv = (
            np.linalg.pinv(
                [
                    [1, 1, len_xy],
                    [1, -1, -len_xy],
                    [1, 1, -len_xy],
                    [1, -1, len_xy],
                ]
            )
            * self.ROBOT_WHEEL_RADIUS
        )
        self.state_publisher_ = self.node.create_publisher(String, 'robomaster_1_state', 10)


    def wheelspeed_callback(self, wheels):
        self.reset_watchdog()

        ws_rpm = np.array([wheels.fl, wheels.fr, wheels.rr, wheels.rl])
        ws_rpm[np.abs(ws_rpm) < 13] = 0.0  # Robot RPM dead band
        ws = ws_rpm / 60 * 2 * np.pi

        rot_inv = self.orientation.as_matrix().transpose()
        v = rot_inv @ self.dyn_inv @ ws

        self.velocity[0] = v[0]
        self.velocity[1] = -v[1]
        self.angular_velocity_world = -v[2]

    def velocity_callback(self, vel):
        self.reset_watchdog()
        self.velocity_ref = vel
        self.angular_velocity_world = -self.velocity_ref.angular.z

    def stop(self):
        self.velocity = np.zeros(3)
        self.angular_velocity_world = 0.0

    def step(self, dt):
        if self.stopped:
            self.stop()
            return
        if self.visible == 1:
            self.nn_input = torch.tensor([self.velocity.linear.x, self.velocity.linear.y, 0, 
                                    self.velocity_ref.linear.x, self.velocity_ref.linear.y, 0])
        else:
            self.nn_input = torch.cat([self.nn_input[self.input_dim:-self.input_dim], self.velocity, self.velocity_ref])
        results = self.dynamics_model(self.nn_input).squeeze().detach()
        self.velocity += results
        self.position += self.velocity * dt
        # self.orientation = R.from_euler("xyz", np.array([0,0,np.clip(
        #                 results[-1],
        #                 -self.MAX_V_ROT_Z_RAD_S,
        #                 self.MAX_V_ROT_Z_RAD_S)]))

        self.publish_state()

    def publish_tf(self):
        tf = TransformStamped()
        tf.header.frame_id = "map"
        tf.header.stamp = self.node.get_clock().now().to_msg()
        tf.child_frame_id = self.rigid_body_label

        tf.transform.translation.x = self.position[0]
        tf.transform.translation.y = self.position[1]
        tf.transform.translation.z = self.position[2]

        orientation = self.orientation.as_quat()
        tf.transform.rotation.x = orientation[0]
        tf.transform.rotation.y = orientation[1]
        tf.transform.rotation.z = orientation[2]
        tf.transform.rotation.w = orientation[3]

        self.tf_publisher.sendTransform(tf)

    def publish_state(self):
        timer_period = 0.1
        msg = String()
        # msg.data = f"timestamp: {self.node.get_clock().now().nanoseconds}, position: {self.position}, velocity: {self.velocity_ref}"
        msg.data = f"timestamp: {time.time()}, position: {self.position}, velocity: {self.velocity_ref}"
        self.state_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    publisher = NNSimpleSimulator("nn_robomaster", r"/robomaster_\d+/", NNRoboMaster)
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
