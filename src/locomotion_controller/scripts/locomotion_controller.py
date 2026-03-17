#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_go.msg import LowCmd
import torch
import torch.nn as nn
from ament_index_python.packages import get_package_share_directory

class ActorNet(nn.Module):
    def __init__(self, num_obs=15, num_actions=12):
        super().__init__()
        
        # Observation Normalization (from obs_normalization=True in config)
        self.register_buffer('running_mean', torch.zeros(num_obs))
        self.register_buffer('running_var', torch.ones(num_obs))

        # Default rsl_rl architecture. Update if you changed it in config!
        self.actor = nn.Sequential(
            nn.Linear(num_obs, 512),
            nn.ELU(),
            nn.Linear(512, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, num_actions)
        )

    def forward(self, x):
        # Apply observation normalization
        x = (x - self.running_mean) / torch.sqrt(self.running_var + 1e-5)
        return self.actor(x)

class LocomotionController(Node):
    def __init__(self):
        super().__init__('locomotion_controller')

        # 1. Load Parameters
        pkg_share_dir = get_package_share_directory('locomotion_controller')
        default_model_path = os.path.join(pkg_share_dir, 'models', 'model_1600.pt')

        self.declare_parameter('model_path', default_model_path)
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.get_logger().info(f"Model Path: {model_path}")

        # 2. Build the Network and Load the raw rsl_rl Weights
        try:
            self.model = ActorNet(num_obs=15, num_actions=12)
            checkpoint = torch.load(model_path, map_location='cpu')
            
            # Get the correct state dictionary depending on the framework
            if isinstance(checkpoint, dict) and 'actor_state_dict' in checkpoint:
                state_dict = checkpoint['actor_state_dict']
            elif isinstance(checkpoint, dict) and 'model_state_dict' in checkpoint:
                state_dict = checkpoint['model_state_dict']
            else:
                state_dict = checkpoint
            
            # 1. Extract weight matrices and bias vectors sequentially (agnostic to exact key names!)
            weights = [v for k, v in state_dict.items() if isinstance(v, torch.Tensor) and 'critic' not in k and 'weight' in k and len(v.shape) == 2]
            biases = [v for k, v in state_dict.items() if isinstance(v, torch.Tensor) and 'critic' not in k and 'bias' in k and len(v.shape) == 1]
            
            if not weights:
                self.get_logger().fatal(f"Available keys in checkpoint: {list(state_dict.keys())}")
                raise SystemExit("Could not find actor weights in checkpoint!")
                
            actor_dict = {}
            # Reconstruct exactly what ActorNet expects (actor.0.weight, actor.2.weight...)
            for i, (w, b) in enumerate(zip(weights, biases)):
                actor_dict[f"actor.{i*2}.weight"] = w
                actor_dict[f"actor.{i*2}.bias"] = b
                
            # 2. Extract observation normalization parameters if they exist
            for k, v in state_dict.items():
                if not isinstance(v, torch.Tensor) or 'critic' in k: 
                    continue
                if 'mean' in k and len(v.shape) == 1:
                    actor_dict['running_mean'] = v.view(-1)
                elif 'var' in k and len(v.shape) == 1:
                    actor_dict['running_var'] = v.view(-1)

            # 3. Dynamically determine num_obs and num_actions
            num_obs = weights[0].shape[1]
            if 'running_mean' in actor_dict:
                num_obs = actor_dict['running_mean'].shape[0]
            num_actions = weights[-1].shape[0]
                
            self.get_logger().info(f"Dynamically inferred: num_obs={num_obs}, num_actions={num_actions}")
            
            # 4. Instantiate model with dynamic shapes and load
            self.model = ActorNet(num_obs=num_obs, num_actions=num_actions)
            self.model.load_state_dict(actor_dict, strict=False)
            self.model.eval() 
            self.get_logger().info(f"Successfully loaded model: {model_path}")
        except Exception as e:
            self.get_logger().fatal(f"Error loading the model!\nPyTorch says: {e}")
            raise SystemExit

        # 3. Subscribers & Publishers
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_cb, 10)
        
        self.low_cmd_pub = self.create_publisher(LowCmd, '/lowcmd', 10)
        
        # State Initialization
        self.last_cmd_vec = [0.0, 0.0, 0.0]
        self.current_joint_pos = torch.zeros(12)
        self.default_joint_pos = [0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3]
        self.kp = 100.0
        self.kd = 5.0

        # Initialize the LowCmd message structure
        self.low_cmd = LowCmd()
        self.low_cmd.head[0] = 0xFE
        self.low_cmd.head[1] = 0xEF
        self.low_cmd.level_flag = 0xFF
        self.low_cmd.gpio = 0

        for i in range(20):
            self.low_cmd.motor_cmd[i].mode = 0x01
            self.low_cmd.motor_cmd[i].q = 2.146E+9    # PosStopF equivalent
            self.low_cmd.motor_cmd[i].kp = 0.0
            self.low_cmd.motor_cmd[i].dq = 16000.0    # VelStopF equivalent
            self.low_cmd.motor_cmd[i].kd = 0.0
            self.low_cmd.motor_cmd[i].tau = 0.0

        # 4. Control Timer (50Hz)
        self.timer = self.create_timer(0.02, self.control_loop)

    def cmd_cb(self, msg: Twist):
        self.last_cmd_vec = [msg.linear.x, msg.linear.y, msg.angular.z]

    def control_loop(self):
        # --- STEP 1: PREPARE OBSERVATIONS ---
        obs_vec = self.last_cmd_vec.copy()
        obs_vec.extend(self.current_joint_pos.tolist())

        # Pad or truncate obs_vec to match the dynamically loaded network
        num_obs = self.model.actor[0].in_features
        if len(obs_vec) < num_obs:
            obs_vec.extend([0.0] * (num_obs - len(obs_vec)))
        elif len(obs_vec) > num_obs:
            obs_vec = obs_vec[:num_obs]

        # Convert to Tensor [1, num_obs]
        obs_tensor = torch.tensor([obs_vec], dtype=torch.float32)

        # --- STEP 2: INFERENCE ---
        with torch.no_grad():
            action = self.model(obs_tensor)

        # --- STEP 3: POST-PROCESS & PUBLISH ---
        action_scale = 0.25
        num_actions = self.model.actor[-1].out_features
        
        # Bound by available default joints to prevent IndexError
        num_to_publish = min(num_actions, len(self.default_joint_pos))
        for i in range(num_to_publish):
            raw_output = action[0, i].item()
            target_q = (raw_output * action_scale) + self.default_joint_pos[i]

            self.low_cmd.motor_cmd[i].q = float(target_q)
            self.low_cmd.motor_cmd[i].dq = 0.0
            self.low_cmd.motor_cmd[i].kp = self.kp
            self.low_cmd.motor_cmd[i].kd = self.kd
            self.low_cmd.motor_cmd[i].tau = 0.0

        # Note: You will need to implement CRC logic in Python if the real robot requires it.
        # self.get_crc(self.low_cmd)
        
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