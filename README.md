
# Learning-based Navigation for a Mobile Robot in Simulation using ROS 2 and Reinforcement Learning

## 1. Overview

This project implements a learning-based navigation framework for a differential-drive mobile robot using **ROS 2 (Jazzy)**, **Gazebo**, and **Reinforcement Learning (RL)**.  
A **TurtleBot3 Burger** is trained in simulation to navigate using **2D LiDAR** observations as input and **continuous velocity commands** as actions, yielding a complete **MDP-based control pipeline** suitable for MSc-level research and subsequent **sim-to-real transfer**.

The framework is designed to be:

- **Modular** (ROS 2 package structure)
- **Reproducible** (fixed configuration and training scripts)
- **Research-ready** (clear MDP definition and RL integration)
- **Extensible** (easy to extend to new tasks, robots, and algorithms)


## 2. Objectives

The main objectives of this work are to:

1. Develop a **ROS 2-based navigation environment** suitable for continuous-control RL algorithms.
2. Integrate **Gazebo simulation**, **LiDAR sensing**, and **odometry feedback** into a closed-loop control setup.
3. Formulate **state, action, and reward functions** under a clear **Markov Decision Process (MDP)** definition.
4. Train navigation policies using **Proximal Policy Optimization (PPO)** from **Stable-Baselines3**.
5. Provide a **reusable and extensible framework** that can support MSc theses, PhD proposals, and research lab projects.

Possible application areas include:

- Obstacle avoidance benchmarks  
- Goal-directed navigation  
- Sim-to-real transfer for mobile robots  


## 3. Problem Formulation as an MDP

Robot navigation is modeled as a **Markov Decision Process (MDP)** defined by the tuple  
\( (\mathcal{S}, \mathcal{A}, P, R, \gamma) \).

### 3.1 State Space (Observation)

At each time step, the agent observes:

- **Downsampled LiDAR scan**: a vector of range values from the 2D laser scanner.
- **Distance-to-goal**: Euclidean distance between the robot and the target pose.
- **Heading feature**: relative orientation between the robot’s heading and the goal direction.

Formally, the state can be written as:



### 3.2 Action Space (Continuous Control)

The action is a continuous 2D vector of velocity commands for a differential-drive robot:



where:




Both are bounded by the TurtleBot3’s physical limits and by safety constraints imposed in the environment.

### 3.3 Reward Function

The reward function is designed to encourage **goal-reaching** while discouraging **collisions** and inefficient motion:

- A penalty proportional to the **distance to the goal**.
- A **large negative reward** when the robot is close to collision or actually collides.
- A **positive terminal reward** when the robot successfully reaches the goal region.

In simplified form:

- Large positive reward: goal reached  
- Large negative reward: collision  
- Shaping terms: distance reduction, smooth motion, time penalty (optional)

### 3.4 Termination Conditions

An episode terminates when any of the following occurs:

- The robot reaches the goal within a predefined tolerance (success).
- The robot collides with an obstacle or leaves the valid workspace (failure).
- A maximum time horizon is reached (timeout).

This formulation enables **fair comparison** with classical navigation baselines and systematic **RL algorithm evaluation**.


## 4. System Architecture

The system follows a modular ROS 2 architecture:

- **ROS 2 Jazzy**
  - **TurtleBot3 in Gazebo**
    - `/scan` – LiDAR measurements
    - `/odom` – Odometry feedback
    - `/cmd_vel` – Velocity commands
  - **RL Environment (Gym-compatible)**
    - Wraps ROS 2 topics and services
    - Provides `reset()` and `step()` interfaces
  - **RL Agent (PPO, Stable-Baselines3)**
    - Interacts with the environment
    - Trains a navigation policy
    - Provides trained models for evaluation

This separation between **simulation**, **environment**, and **agent** eases debugging, benchmarking, and extensions (e.g., new robots or sensors).


## 5. Implemented Features

The project currently includes:

- ✅ **ROS 2 package**: `rl_nav`
- ✅ **Baseline random navigation controller** for comparison
- ✅ **Gym-compatible RL environment** wrapping ROS 2 and Gazebo
- ✅ **Continuous action space** over linear and angular velocity
- ✅ **State representation** based on LiDAR, distance-to-goal, and heading
- ✅ **Reward shaping and termination conditions**
- ✅ **Gazebo integration** with TurtleBot3 Burger
- ✅ **PPO training pipeline** using Stable-Baselines3
- ✅ **Reproducible project structure** for MSc-level work


## 6. Software Requirements

The framework has been tested with the following setup:

- **Operating System**: Ubuntu 24.04
- **ROS 2**: Jazzy
- **Simulator**: Gazebo
- **Robot model**: TurtleBot3 Burger
- **Python**: 3.12
- **RL Library**: Stable-Baselines3 (PPO)
- Additional ROS 2 packages:
  - `turtlebot3`
  - `turtlebot3_gazebo`
  - standard ROS 2 navigation dependencies (for potential comparison baselines)


## 7. Installation and Setup

1. Source ROS 2:

   ```bash
   source /opt/ros/jazzy/setup.bash
   ```

2. Set the TurtleBot3 model:

   ```bash
   export TURTLEBOT3_MODEL=burger
   ```

3. Build and source the ROS 2 workspace containing `rl_nav`:

   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

4. Ensure your Python virtual environment for RL is created (optional but recommended):

   ```bash
   python3 -m venv ~/rl_venv
   source ~/rl_venv/bin/activate
   pip install stable-baselines3[extra]
   ```


## 8. Running the Simulation

1. Launch TurtleBot3 in Gazebo:

   ```bash
   source /opt/ros/jazzy/setup.bash
   export TURTLEBOT3_MODEL=burger
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

2. In a separate terminal, source the workspace and start the RL environment or baseline controller (examples will depend on your node/launch file names):

   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 run rl_nav random_controller    # example baseline
   ```


## 9. Reinforcement Learning Environment

The RL environment is implemented in:

```text
rl_nav/rl_nav/env.py
```

It exposes the standard Gym-like API:

- `reset()` – resets the simulation and returns the initial observation
- `step(action)` – applies the action, advances the simulation, and returns:
  - next observation
  - reward
  - done flag
  - info dictionary

It also defines:

- `observation_space` – LiDAR-based observation vector plus goal-related features
- `action_space` – continuous 2D control over `[linear_velocity, angular_velocity]`

The environment is fully compatible with **Stable-Baselines3** workflows.


## 10. Training with PPO

1. Activate the RL virtual environment:

   ```bash
   source ~/rl_venv/bin/activate
   ```

2. Run the PPO training script:

   ```bash
   python3 train_ppo.py
   ```

3. After training, the model is stored under:

   ```text
   models_ppo/ppo_turtlebot.zip
   ```

This model can be loaded for evaluation or further fine-tuning.


## 11. Evaluation and Future Work

The framework is designed for systematic **quantitative evaluation** and MSc/PhD-level extensions.  
Potential evaluation metrics include:

- **Success rate** (percentage of episodes where the goal is reached)
- **Collision rate** and minimum distance to obstacles
- **Episode length** and path efficiency
- **Energy consumption** approximations (e.g., based on commanded velocities)

Promising directions for future work:

- **Domain randomization** (textures, lighting, sensor noise, obstacle layouts)
- **Curriculum learning** (progressively more complex environments)
- **Transfer learning** from simulation to a physical TurtleBot3
- **Comparison with classical planners** (e.g., move_base, Nav2)
- **Algorithmic comparison**: PPO vs SAC vs TD3 for continuous navigation
- **Multi-goal navigation** and dynamic obstacle scenarios


## 12. Academic Relevance

This project demonstrates:

- Competence in **robotics simulation** and **ROS 2 development**
- Ability to design and implement **learning-based control systems**
- Solid understanding of **MDP formulation** and **RL training pipelines**
- Awareness of **reproducibility**, **benchmarking**, and **research extensions**

Therefore, the framework is suitable as:

- A **foundation for an MSc dissertation**
- Supporting material for a **PhD proposal**
- A starting point for **research lab projects**
- A strong addition to a **robotics or AI portfolio**


## 13. Author

**Panagiota Grosdouli**


## 14. License

The project is intended for **academic and research use**.  
Please cite or acknowledge the author if you use or extend this work in your own research or publications.
