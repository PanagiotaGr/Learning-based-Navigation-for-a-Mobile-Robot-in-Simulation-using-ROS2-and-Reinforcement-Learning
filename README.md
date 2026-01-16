# Learning-based Navigation for a Mobile Robot in Simulation

### ROS 2 Jazzy • Gazebo • Reinforcement Learning (PPO)

> Learning-based navigation framework for a differential-drive mobile robot using **ROS 2 (Jazzy)**, **Gazebo**, and **Reinforcement Learning**. A **TurtleBot3 Burger** is trained in simulation with **2D LiDAR** observations and **continuous velocity commands**, forming a complete **MDP-based control pipeline** suitable for MSc-level research and future **sim-to-real transfer**.

---

## Table of Contents

* [Overview](#overview)
* [Objectives](#objectives)
* [Problem Formulation as an MDP](#problem-formulation-as-an-mdp)

  * [State Space (Observation)](#state-space-observation)
  * [Action Space (Continuous Control)](#action-space-continuous-control)
  * [Reward Function](#reward-function)
  * [Termination Conditions](#termination-conditions)
* [System Architecture](#system-architecture)
* [Implemented Features](#implemented-features)
* [Software Requirements](#software-requirements)
* [Installation and Setup](#installation-and-setup)
* [Running the Simulation](#running-the-simulation)
* [Reinforcement Learning Environment](#reinforcement-learning-environment)
* [Training with PPO](#training-with-ppo)
* [Evaluation and Future Work](#evaluation-and-future-work)
* [Author](#author)

---

## Overview

This project implements a learning-based navigation framework for a differential-drive mobile robot using **ROS 2 (Jazzy)**, **Gazebo**, and **Reinforcement Learning (RL)**.
A **TurtleBot3 Burger** is trained in simulation to navigate using **2D LiDAR** observations as input and **continuous velocity commands** as actions.

The framework is designed to be:

* **Modular** (ROS 2 package structure)
* **Reproducible** (fixed configuration + training scripts)
* **Research-ready** (explicit MDP definition and RL integration)
* **Extensible** (easy to extend to new tasks, robots, and algorithms)

---

## Objectives

The main objectives are:

1. Build a **ROS 2-based navigation environment** suitable for continuous-control RL.
2. Integrate **Gazebo**, **LiDAR**, and **odometry feedback** into a closed-loop control pipeline.
3. Define **state, action, and reward** under a clear **Markov Decision Process (MDP)** formulation.
4. Train navigation policies using **Proximal Policy Optimization (PPO)** with **Stable-Baselines3**.
5. Provide a reusable framework suitable for **MSc theses**, PhD proposals, and research projects.

Example application areas:

* Obstacle avoidance benchmarks
* Goal-directed navigation
* Sim-to-real transfer for mobile robots

---

## Problem Formulation as an MDP

Robot navigation is modeled as a Markov Decision Process (MDP) defined by:

<p align="center">
  <img alt="MDP tuple" src="https://latex.codecogs.com/svg.image?\Large%20(\mathcal{S},\mathcal{A},P,R,\gamma)" />
</p>

### State Space (Observation)

At each time step, the agent observes:

* **Downsampled LiDAR scan**: a vector of range values from the 2D laser scanner
* **Distance-to-goal**: Euclidean distance between robot and target pose
* **Heading feature**: relative orientation between robot heading and goal direction

A compact state representation can be written as:

<p align="center">
  <img alt="state definition" src="https://latex.codecogs.com/svg.image?\Large%20s_t=[\tilde{\mathbf{r}}_t,\ d_t,\ \Delta\psi_t]" />
</p>

where:

* <img alt="r_t" src="https://latex.codecogs.com/svg.image?\tilde{\mathbf{r}}_t" /> is the downsampled LiDAR vector
* <img alt="d_t" src="https://latex.codecogs.com/svg.image?d_t" /> is the distance-to-goal
* <img alt="Delta psi" src="https://latex.codecogs.com/svg.image?\Delta\psi_t" /> is the heading error (goal direction relative to robot heading)

---

### Action Space (Continuous Control)

The action is a continuous 2D vector of velocity commands for a differential-drive robot:

<p align="center">
  <img alt="action definition" src="https://latex.codecogs.com/svg.image?\Large%20a_t=[v_t,\ \omega_t]" />
</p>

where:

* <img alt="v_t" src="https://latex.codecogs.com/svg.image?v_t" /> is the linear velocity (m/s)
* <img alt="omega_t" src="https://latex.codecogs.com/svg.image?\omega_t" /> is the angular velocity (rad/s)

Both are bounded by TurtleBot3 physical limits and safety constraints imposed by the environment:

<p align="center">
  <img alt="action bounds" src="https://latex.codecogs.com/svg.image?\Large%20v_t\in[v_{\min},v_{\max}],\ \omega_t\in[\omega_{\min},\omega_{\max}]" />
</p>

---

### Reward Function

The reward encourages **goal-reaching** while discouraging **collisions** and inefficient behaviour. A practical shaped form is:

<p align="center">
  <img alt="reward" src="https://latex.codecogs.com/svg.image?\Large%20r_t=\alpha(d_{t-1}-d_t)-\beta\cdot\mathbb{1}[\text{collision}]+\rho\cdot\mathbb{1}[\text{goal}]-\lambda" />
</p>

Interpretation:

* <img alt="distance progress" src="https://latex.codecogs.com/svg.image?\alpha(d_{t-1}-d_t)" />: reward progress toward the goal
* <img alt="collision term" src="https://latex.codecogs.com/svg.image?-\beta\cdot\mathbb{1}[\text{collision}]" />: collision penalty
* <img alt="goal term" src="https://latex.codecogs.com/svg.image?\rho\cdot\mathbb{1}[\text{goal}]" />: terminal goal reward
* <img alt="time penalty" src="https://latex.codecogs.com/svg.image?-\lambda" />: optional per-step time penalty

This formulation enables fair comparison with classical navigation baselines and systematic RL evaluation.

---

### Termination Conditions

An episode terminates when:

* **Success**: robot reaches the goal within tolerance
* **Failure**: collision, or robot leaves the valid workspace
* **Timeout**: maximum horizon reached

---

## System Architecture

High-level components:

* **ROS 2 Jazzy**

  * **TurtleBot3 Burger in Gazebo**

    * `/scan` – LiDAR measurements
    * `/odom` – odometry feedback
    * `/cmd_vel` – velocity commands
  * **RL Environment (Gym-compatible)**

    * wraps ROS 2 topics/services
    * provides `reset()` and `step()`
  * **RL Agent (PPO, Stable-Baselines3)**

    * trains a navigation policy
    * saves trained models for evaluation

This separation between **simulation**, **environment**, and **agent** simplifies debugging, benchmarking, and extensions (e.g., new robots/sensors).

---

## Implemented Features

* ✅ ROS 2 package: `rl_nav`
* ✅ Baseline random navigation controller (for comparison)
* ✅ Gym-compatible RL environment wrapping ROS 2 and Gazebo
* ✅ Continuous action space over linear & angular velocity
* ✅ LiDAR + goal features observation design
* ✅ Reward shaping and termination logic
* ✅ Gazebo integration with TurtleBot3 Burger
* ✅ PPO training pipeline using Stable-Baselines3
* ✅ Reproducible structure suitable for MSc-level work

---

## Software Requirements

Tested setup:

* **Ubuntu**: 24.04
* **ROS 2**: Jazzy
* **Simulator**: Gazebo
* **Robot**: TurtleBot3 Burger
* **Python**: 3.12
* **RL Library**: Stable-Baselines3 (PPO)

Additional ROS 2 packages (typical):

* `turtlebot3`
* `turtlebot3_gazebo`
* standard ROS 2 dependencies (Nav2 optional, for classical baseline comparisons)

---

## Installation and Setup

1. Source ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
```

2. Set TurtleBot3 model:

```bash
export TURTLEBOT3_MODEL=burger
```

3. Build and source the ROS 2 workspace:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

4. Create an RL virtual environment (recommended):

```bash
python3 -m venv ~/rl_venv
source ~/rl_venv/bin/activate
pip install stable-baselines3[extra]
```

---

## Running the Simulation

1. Launch TurtleBot3 in Gazebo:

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

2. In a separate terminal, source the workspace and run a baseline controller (example):

```bash
source ~/ros2_ws/install/setup.bash
ros2 run rl_nav random_controller
```

---

## Reinforcement Learning Environment

The RL environment is implemented at:

```text
rl_nav/rl_nav/env.py
```

It exposes a Gym-like API:

* `reset()` → resets simulation and returns the initial observation
* `step(action)` → applies action and returns `(obs, reward, done, info)`

It defines:

* `observation_space` → LiDAR vector plus goal-related features
* `action_space` → continuous 2D control `[linear_velocity, angular_velocity]`

The environment is compatible with **Stable-Baselines3** workflows.

---

## Training with PPO

1. Activate the RL virtual environment:

```bash
source ~/rl_venv/bin/activate
```

2. Run training:

```bash
python3 train_ppo.py
```

3. Output model:

```text
models_ppo/ppo_turtlebot.zip
```

This model can be loaded for evaluation or further fine-tuning.

---

## Evaluation and Future Work

Potential evaluation metrics:

* **Success rate** (goal reached)
* **Collision rate** and minimum distance to obstacles
* **Episode length** and path efficiency
* **Energy proxies** (e.g., based on commanded velocities)

Future directions:

* Domain randomization (textures, lighting, sensor noise, obstacle layouts)
* Curriculum learning (progressively harder scenarios)
* Sim-to-real transfer to a physical TurtleBot3
* Comparison with classical planners (Nav2 / move_base-style baselines)
* Algorithmic comparison: PPO vs SAC vs TD3
* Multi-goal navigation and dynamic obstacle scenarios

---

## Author

**Panagiota Grosdouli**
