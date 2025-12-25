import rclpy
import math
from typing import Tuple, Dict, Any

import numpy as np
import gymnasium as gym
from gymnasium import spaces

from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class TurtlebotRLEnv(Node, gym.Env):
    """
    Gym-style περιβάλλον για TurtleBot3 σε ROS 2 (Jazzy).

    Observation:
        - downsampled LaserScan (num_lasers στοιχεία)
        - distance_to_goal (scalar)
        - heading (scalar)

    Action:
        - continuous [linear_x, angular_z] σε κανονικοποιημένο διάστημα [-1, 1]

    step():
        - στέλνει cmd_vel
        - εκτελεί 1 ROS spin
        - υπολογίζει reward & termination
    """

    metadata = {"render_modes": []}

    def __init__(self,
                 num_lasers: int = 24,
                 max_lin: float = 0.22,
                 max_ang: float = 2.0):
        Node.__init__(self, 'turtlebot_rl_env')
        gym.Env.__init__(self)

        # ROS I/O
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # Sensor buffers
        self.scan_data = None
        self.robot_pose = None  # (x, y)

        # Action/Observation spaces
        self.num_lasers = num_lasers
        self.max_lin = max_lin
        self.max_ang = max_ang

        # Action: [lin_scale, ang_scale] in [-1, 1]
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0], dtype=np.float32),
            high=np.array([1.0, 1.0], dtype=np.float32),
            dtype=np.float32,
        )

        # Observation: [lasers..., dist_to_goal, heading]
        obs_len = self.num_lasers + 2
        self.observation_space = spaces.Box(
            low=0.0,
            high=10.0,
            shape=(obs_len,),
            dtype=np.float32,
        )

        # Fixed goal για αρχή
        self.goal = np.array([2.0, 0.0], dtype=np.float32)

        # Internal
        self._done = False

    # ---------- ROS Callbacks ----------

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        # Αν κάτι είναι inf, το κλασικό είναι να το “κόψουμε” σε max_range
        max_range = 3.5
        ranges = np.where(np.isinf(ranges), max_range, ranges)

        if len(ranges) == 0:
            return

        step = max(1, len(ranges) // self.num_lasers)
        downsampled = ranges[::step][:self.num_lasers]
        self.scan_data = downsampled

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.robot_pose = np.array([x, y], dtype=np.float32)

    # ---------- Gym API ----------

    def reset(self,
              seed: int | None = None,
              options: Dict[str, Any] | None = None
              ) -> Tuple[np.ndarray, Dict[str, Any]]:
        super().reset(seed=seed)
        self._done = False

        # Σταμάτα το ρομπότ
        self._stop_robot()

        # Goal σταθερό (μπορούμε να το κάνουμε random αργότερα)
        self.goal = np.array([2.0, 0.0], dtype=np.float32)

        # Δώσε λίγο χρόνο στο ROS να “γεμίσει” scan/odom
        self.get_logger().info('Resetting RL env...')
        # Σημείωση: το actual spin θα το κάνει ο "έξω" κώδικας (training loop)

        obs = self._get_obs()
        info: Dict[str, Any] = {}
        return obs, info

    def step(self, action: np.ndarray):
        """
        action: np.array([lin_scale, ang_scale]) in [-1, 1]
        """
        if self._done:
            obs = self._get_obs()
            return obs, 0.0, True, False, {}

        lin_scale = float(action[0])
        ang_scale = float(action[1])

        v = lin_scale * self.max_lin
        w = ang_scale * self.max_ang

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.cmd_pub.publish(twist)

        # 🔄 Δώσε χρόνο στα ROS callbacks
        rclpy.spin_once(self, timeout_sec=0.05)

        obs = self._get_obs()
        reward = self._compute_reward(obs)
        terminated = self._check_terminated(obs)
        truncated = False

        self._done = terminated or truncated
        info: Dict[str, Any] = {}
        return obs, reward, terminated, truncated, info



    # ---------- Βοηθητικές συναρτήσεις ----------

    def _get_obs(self) -> np.ndarray:
        # Default τιμές αν δεν έχουμε ακόμα δεδομένα
        if self.scan_data is None:
            lasers = np.ones(self.num_lasers, dtype=np.float32) * 3.5
        else:
            lasers = self.scan_data

        if self.robot_pose is None:
            dist = 3.0
            heading = 0.0
        else:
            vec_to_goal = self.goal - self.robot_pose
            dist = float(np.linalg.norm(vec_to_goal))
            # Για απλότητα, heading = dist (placeholder).
            # Μπορούμε να βάλουμε πραγματική γωνία με quaternion → yaw.
            heading = dist

        obs = np.concatenate([lasers, np.array([dist, heading], dtype=np.float32)])
        return obs.astype(np.float32)

    def _compute_reward(self, obs: np.ndarray) -> float:
        lasers = obs[:-2]
        dist = float(obs[-2])

        min_laser = float(np.min(lasers))

        # Μεγάλη αρνητική ποινή αν πάμε πολύ κοντά σε εμπόδιο
        if min_laser < 0.15:
            return -20.0

        # Reward = -distance (θέλουμε να μειώνεται η απόσταση στο goal)
        reward = -dist

        return reward

    def _check_terminated(self, obs: np.ndarray) -> bool:
        lasers = obs[:-2]
        dist = float(obs[-2])

        if dist < 0.2:
            # Έφτασε αρκετά κοντά στο goal
            return True

        if float(np.min(lasers)) < 0.12:
            # Σύγκρουση
            return True

        return False

    def _stop_robot(self):
        twist = Twist()
        self.cmd_pub.publish(twist)

    def close(self):
        self._stop_robot()
        super().destroy_node()
