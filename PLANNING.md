# Agentic Racer: Planning & Architecture

## 1. Project Goal
Build an autonomous robotics CI/CD testing loop where an AI Agent iteratively tunes and improves the performance of a simulated robotic racecar using physics-based feedback.

## 2. The Physical Reality (The Unchangeable Constraints)
* **The Robot:** NVIDIA JetBot (Differential Drive). It operates by varying the speed of its left and right wheels. It is lightweight and can tip over if acceleration is too aggressive.
* **The Sensors:** 2D LiDAR. The robot can only "see" distance to the walls. 
* **The Track:** A closed-loop track with walls.

## 3. The Objective Function (The Test Harness)
The Agent is evaluated strictly on the following metrics:
1. **Primary Metric:** Minimize `lap_time`.
2. **Failure Condition:** If the robot's collision mesh touches a wall, `crashes = 1`. The run is an automatic failure.

## 4. The Autopilot (The Tunable Brain)
The Agent is only permitted to modify the Autopilot software (Step 3). 
The baseline algorithm is a **Wall Follower** or **Pure Pursuit** controller.
The Agent may tune the following variables:
* `max_forward_speed`: The baseline velocity.
* `steering_gain` (Kp): How aggressively the robot turns to stay centered.
* `lookahead_distance`: How far ahead the LiDAR scans for corners.

## 5. The Agentic Loop
1. The Agent generates a configuration for the Autopilot.
2. The Test Harness boots Gazebo in headless mode, spawning the JetBot in the track.
4. The Agent reads the logs, analyzes the physics failure (e.g., "The steering gain was too low, causing understeer into the outer wall"), and pushes a new configuration.

## 6. The Orchestration Rules (Test-Driven Development)
All Subagents must strictly adhere to the 20-Point Test Matrix located in `TEST_MATRIX.md`. 
An Agent is not permitted to consider a task complete until its generated code passes all 5 of the strict tests defined for its respective domain.
