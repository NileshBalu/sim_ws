#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from unitree_go.msg import LowCmd, LowState
import torch
import torch.nn as nn
from ament_index_python.packages import get_package_share_directory


class ActorNet(nn.Module):
    def __init__(self, num_obs=45, num_actions=12):
        super().__init__()
        self.register_buffer('obs_mean', torch.zeros(num_obs))
        self.register_buffer('obs_std',  torch.ones(num_obs))
        self.actor = nn.Sequential(
            nn.Linear(num_obs, 512), nn.ELU(),
            nn.Linear(512, 256),    nn.ELU(),
            nn.Linear(256, 128),    nn.ELU(),
            nn.Linear(128, num_actions)
        )

    def forward(self, x):
        x = (x - self.obs_mean) / self.obs_std
        x = torch.clamp(x, -5.0, 5.0)
        return self.actor(x)


def load_model(pt_path, logger):
    checkpoint = torch.load(pt_path, map_location='cpu')

    if isinstance(checkpoint, dict) and 'actor_state_dict' in checkpoint:
        state_dict = checkpoint['actor_state_dict']
    elif isinstance(checkpoint, dict) and 'model_state_dict' in checkpoint:
        state_dict = checkpoint['model_state_dict']
    else:
        state_dict = checkpoint

    logger.info(f"actor_state_dict keys: {list(state_dict.keys())}")

    # Extract MLP weights/biases — skip normalizer and critic keys
    weights = []
    biases  = []
    for k, v in state_dict.items():
        if not isinstance(v, torch.Tensor):
            continue
        if 'critic' in k or 'normalizer' in k:
            continue
        if 'weight' in k and len(v.shape) == 2:
            weights.append(v)
        elif 'bias' in k and len(v.shape) == 1:
            biases.append(v)

    if not weights:
        raise RuntimeError(
            f"No actor MLP weights found. Keys: {list(state_dict.keys())}")

    logger.info(f"Found {len(weights)} weight matrices: {[list(w.shape) for w in weights]}")

    actor_dict = {}
    for i, (w, b) in enumerate(zip(weights, biases)):
        actor_dict[f"actor.{i * 2}.weight"] = w
        actor_dict[f"actor.{i * 2}.bias"]   = b

    num_obs     = weights[0].shape[1]
    num_actions = weights[-1].shape[0]

    # Load normalizer stats from state_dict.
    # The checkpoint contains: obs_normalizer._mean, obs_normalizer._var, obs_normalizer._std
    # Prefer _std (already standard deviation) over _var (needs sqrt).
    obs_mean_direct = state_dict.get('obs_normalizer._mean', None)
    obs_std_direct  = state_dict.get('obs_normalizer._std',  None)
    obs_var_direct  = state_dict.get('obs_normalizer._var',  None)

    if obs_mean_direct is not None:
        obs_mean_direct = obs_mean_direct.float().view(-1)
        logger.info(f"Loaded obs_normalizer._mean, shape={obs_mean_direct.shape}")

    if obs_std_direct is not None:
        obs_std_direct = obs_std_direct.float().view(-1)
        logger.info(f"Loaded obs_normalizer._std directly, shape={obs_std_direct.shape}")
    elif obs_var_direct is not None:
        obs_std_direct = torch.sqrt(obs_var_direct.float().view(-1) + 1e-5)
        logger.info("Converted obs_normalizer._var to std via sqrt(var + 1e-5)")


    if obs_mean_direct is not None and obs_std_direct is not None:
        logger.info("Using normalization stats from checkpoint state_dict.")
        obs_mean = obs_mean_direct
        obs_std  = torch.clamp(obs_std_direct, min=1e-5)
    else:
        logger.warn("Normalization stats not found in state_dict")

    logger.info(f"obs_mean[:6] = {obs_mean[:6].tolist()}")
    logger.info(f"obs_std[:6]  = {obs_std[:6].tolist()}")

    return actor_dict, num_obs, num_actions, obs_mean, obs_std


class LocomotionController(Node):
    def __init__(self):
        super().__init__('locomotion_controller')

        # Load model                                                        #
        pkg_share_dir = get_package_share_directory('locomotion_controller')
        default_pt_path = os.path.join(pkg_share_dir, 'models', 'model_1600.pt')
        self.declare_parameter('model_path', default_pt_path)
        pt_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.get_logger().info(f"Checkpoint: {pt_path}")

        try:
            actor_dict, num_obs, num_actions, obs_mean, obs_std = \
                load_model(pt_path, self.get_logger())
            self.model = ActorNet(num_obs=num_obs, num_actions=num_actions)
            result = self.model.load_state_dict(actor_dict, strict=False)
            self.get_logger().info(
                f"load_state_dict: missing={result.missing_keys}, "
                f"unexpected={result.unexpected_keys}")
            self.model.obs_mean.copy_(obs_mean[:num_obs])
            self.model.obs_std.copy_(obs_std[:num_obs])
            self.model.eval()
            self.get_logger().info(
                f"Model ready. num_obs={num_obs}, num_actions={num_actions}")
        except Exception as e:
            self.get_logger().fatal(f"Failed to load model: {e}")
            raise SystemExit

        # Joint convention                                                  #
        # Map between the XML joint order and the RL trainer joint order
        self.rl_to_unitree = [3,  4,  5,  0,  1,  2,  9,  10,  11,  6,  7,  8]

        # Default joint pos used in the RL trainer
        self.default_joint_pos_rl = [
            -0.1,  0.9, -1.8,   # FL
             0.1,  0.9, -1.8,   # FR
            -0.1,  0.9, -1.8,   # RL
             0.1,  0.9, -1.8,   # RR
        ]

        # Sit/stand targets in Unitree SDK order (FR, FL, RR, RL)
        # Hip values = MuJoCo default * hip_sign_unitree
        self.target_pos_stand = [
             0.0,  0.67, -1.3,   # FR
             0.0,  0.67, -1.3,   # FL
            -0.2,  0.67, -1.3,   # RR
             0.2,  0.67, -1.3,   # RL
        ]
        self.target_pos_sit = [
             0.0,  1.36, -2.65,   # FR
             0.0,  1.36, -2.65,   # FL
            -0.2,  1.36, -2.65,   # RR
             0.2,  1.36, -2.65,   # RL
        ]

        # PD gains matching training actuator config exactly               #
        
        self.kp_hip   = 100.0;  self.kd_hip   = 4.0
        self.kp_thigh = 100.0;  self.kd_thigh = 4.0
        self.kp_calf  = 150.0;  self.kd_calf  = 6.0
        self.joint_type = [0, 1, 2,  0, 1, 2,  0, 1, 2,  0, 1, 2]

        # Timing — from training config                                    #
        # sim timestep=0.005, decimation=4 -> policy at 50 Hz
        # Control loop at 500 Hz, policy evaluated every DECIMATION=10 ticks
        self.DECIMATION   = 10
        self.control_tick = 0

        # State variables                                                   #
        self.last_cmd_vec      = [0.0, 0.0, 0.0]
        self.current_joint_pos = torch.zeros(12)
        self.current_joint_vel = [0.0] * 12
        self.base_ang_vel      = [0.0, 0.0, 0.0]
        self.projected_gravity = [0.0, 0.0, -1.0]
        self.last_action       = [0.0] * 12

        self.current_mode       = 1
        self.previous_mode      = 1
        self.motiontime         = 1
        self.duration           = 1000
        self.first_motion_flag  = 1
        self.loco_to_stand_flag = 0

        # LowCmd initialisation                                             #
        self.low_cmd = LowCmd()
        self.low_cmd.head[0]    = 0xFE
        self.low_cmd.head[1]    = 0xEF
        self.low_cmd.level_flag = 0xFF
        self.low_cmd.gpio       = 0
        for i in range(20):
            self.low_cmd.motor_cmd[i].mode = 0x01
            self.low_cmd.motor_cmd[i].q    = 2.146e+9
            self.low_cmd.motor_cmd[i].kp   = 0.0
            self.low_cmd.motor_cmd[i].dq   = 16000.0
            self.low_cmd.motor_cmd[i].kd   = 0.0
            self.low_cmd.motor_cmd[i].tau  = 0.0

        # ROS2 pub/sub                                                      #
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.mode_sub = self.create_subscription(Int32, '/mode', self.mode_cb, 10)
        self.state_sub = self.create_subscription(LowState, '/lowstate', self.state_cb,10)
        self.low_cmd_pub = self.create_publisher(LowCmd, '/lowcmd', 10)
        self.timer = self.create_timer(0.002, self.control_loop)
        self.get_logger().info("Locomotion controller ready.")

    # Callbacks                                                                #

    def cmd_cb(self, msg):
        self.last_cmd_vec = [msg.linear.x, msg.linear.y, msg.angular.z]

    def mode_cb(self, msg):
        self.previous_mode = self.current_mode
        self.current_mode  = msg.data
        if self.current_mode != self.previous_mode:
            self.motiontime         = 0
            self.first_motion_flag  = 0
            self.loco_to_stand_flag = 1 if self.previous_mode == 3 else 0
            if self.current_mode == 3:
                self.last_action  = [0.0] * 12
                self.control_tick = 0

    def state_cb(self, msg):
        for i in range(12):
            self.current_joint_pos[i] = msg.motor_state[i].q
            self.current_joint_vel[i] = msg.motor_state[i].dq
        self.base_ang_vel = list(msg.imu_state.gyroscope)
        # Projected gravity: rotate [0,0,-1] into robot body frame
        # unitree_mujoco quaternion order: [w, x, y, z]
        w, x, y, z = msg.imu_state.quaternion
        self.projected_gravity = [
            2.0 * (w*y - x*z),
            -2.0 * (w*x + y*z),
            -1.0 + 2.0 * (x*x + y*y),
        ]

    # Helpers                                                                  #
    def _get_gains(self, u_idx, stand_mode=False):
        jtype = self.joint_type[u_idx]
        if jtype == 0:   
            kp, kd = self.kp_hip,   self.kd_hip
        elif jtype == 1:
            kp, kd = self.kp_thigh, self.kd_thigh
        else:
            kp, kd = self.kp_calf,  self.kd_calf
        
        if stand_mode:
            kp *= 5.0
        
        return kp, kd

    def _set_motor(self, u_idx, q, stand_mode=False):
        kp, kd = self._get_gains(u_idx, stand_mode)
        self.low_cmd.motor_cmd[u_idx].q   = float(q)
        self.low_cmd.motor_cmd[u_idx].dq  = 0.0
        self.low_cmd.motor_cmd[u_idx].kp  = kp
        self.low_cmd.motor_cmd[u_idx].kd  = kd
        self.low_cmd.motor_cmd[u_idx].tau = 0.0

    # Control loop — 500 Hz                                                   #
    def control_loop(self):
        self.motiontime  += 1
        self.control_tick += 1
        t = min(self.motiontime / self.duration, 1.0)

        if self.current_mode == 1:
            if self.first_motion_flag:
                for i in range(12):
                    self._set_motor(i, self.target_pos_sit[i], stand_mode=True)
            else:
                for i in range(12):
                    q = (1.0 - t) * self.target_pos_stand[i] + t * self.target_pos_sit[i]
                    self._set_motor(i, q, stand_mode=True)
            self.low_cmd_pub.publish(self.low_cmd)

        elif self.current_mode == 2:
            if self.loco_to_stand_flag:
                for i in range(12):
                    self._set_motor(i, self.target_pos_stand[i], stand_mode=True)
            else:
                for i in range(12):
                    q = (1.0 - t) * self.target_pos_sit[i] + t * self.target_pos_stand[i]
                    self._set_motor(i, q, stand_mode=True)
            self.low_cmd_pub.publish(self.low_cmd)

        elif self.current_mode == 3:
            if self.control_tick % self.DECIMATION == 0:
                self._locomotion_step()
            else:
                # Hold last commanded positions between policy steps
                self.low_cmd_pub.publish(self.low_cmd)

    def _locomotion_step(self):
        # Observation order matches actor_terms exactly:
        # base_ang_vel(3), projected_gravity(3), command(3),
        # joint_pos_rel(12), joint_vel(12), last_action(12) -> 45 total
        obs = []

        obs.extend(self.base_ang_vel)
        obs.extend(self.projected_gravity)
        obs.extend(self.last_cmd_vec)

        for i in range(12):
            u_idx  = self.rl_to_unitree[i]
            pos_mj = self.current_joint_pos[u_idx].item()
            obs.append(pos_mj - self.default_joint_pos_rl[i])

        for i in range(12):
            u_idx  = self.rl_to_unitree[i]
            vel_mj = self.current_joint_vel[u_idx]
            obs.append(vel_mj)

        obs.extend(self.last_action)

        num_obs = self.model.actor[0].in_features
        if len(obs) != num_obs:
            self.get_logger().warn(
                f"obs length {len(obs)} != {num_obs}",
                throttle_duration_sec=5.0)
            obs = (obs + [0.0] * num_obs)[:num_obs]

        obs_tensor = torch.tensor([obs], dtype=torch.float32)
        with torch.no_grad():
            action = self.model(obs_tensor)

        self.last_action = action[0].tolist()

        action_scale = 0.25
        for i in range(12):
            target_mj      = action[0, i].item() * action_scale + self.default_joint_pos_rl[i]
            u_idx          = self.rl_to_unitree[i]
            target_unitree = target_mj
            self._set_motor(u_idx, target_unitree)

        self.low_cmd_pub.publish(self.low_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LocomotionController()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()