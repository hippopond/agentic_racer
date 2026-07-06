# Agentic Racer: 20-Point Test Matrix

This matrix defines the strict Test-Driven Development (TDD) criteria for the Multi-Agent Orchestration. No Subagent is permitted to submit a Pull Request or report completion until its component passes all 5 of its assigned tests.

## Step 1: The Track (World Builder Agent)
1. **XML Syntax (T1.1):** The `.sdf` file must parse without Gazebo XML formatting errors.
2. **Collision Meshes (T1.2):** All walls must have `<collision>` tags to prevent the robot from passing through.
3. **Scale Sanity (T1.3):** The track width must be suitable for a 0.2m robot (e.g., track width ~1.5 meters).
4. **Lighting (T1.4):** A `<directional_light>` or `<point_light>` must exist so camera sensors do not see pitch black.
5. **Closed Loop (T1.5):** The outer and inner walls must form a continuous, enclosed circuit with no gaps.

## Step 2: The Racecar (Hardware Agent)
1. **URDF Parsing (T2.1):** The `.urdf` must compile flawlessly via `robot_state_publisher` with no missing parent/child links.
2. **Inertia Sanity (T2.2):** No link may have zero or infinite mass/inertia. (Prevents physics explosions).
3. **The Drop Test (T2.3):** When spawned 1 meter above the ground in Gazebo, the robot must land stably without wheels detaching or chassis flipping.
4. **Actuation (T2.4):** The left and right wheel joints must accept ROS 2 velocity commands.
5. **Sensor TF (T2.5):** The LiDAR sensor must be physically mounted above the chassis, and its TF frame must not intersect the chassis collision mesh (which would blind the laser).

## Step 3: The Autopilot (Software Agent)
1. **Node Initialization (T3.1):** The Python script must initialize the ROS 2 node without crashing.
2. **Subscription (T3.2):** The node must successfully subscribe to the `LaserScan` message type on the `/scan` topic.
3. **Publication (T3.3):** The node must successfully publish `Twist` messages to `/cmd_vel`.
4. **Brain in a Jar (T3.4):** When injected with mock LiDAR data indicating a wall 0.1m ahead, the node must publish a braking/turning command within 10 milliseconds.
5. **Latency (T3.5):** The main control loop execution time must be under 50ms to ensure real-time physics reactivity.

## Step 4: The CI/CD Referee (QA Agent)
1. **Orchestration (T4.1):** The `eval_pipeline.launch.py` must successfully boot Gazebo headless, the robot, the autopilot, and the referee node simultaneously.
2. **Timer Logic (T4.2):** The node must accurately track elapsed simulation time (`lap_time`).
3. **Collision Detection (T4.3):** The node must successfully intercept Gazebo physics contact messages to detect if the chassis hits a wall (`crashes = 1`).
4. **Auto-Teardown (T4.4):** Upon detecting a crash or a completed lap, the Referee must gracefully kill all ROS 2 nodes and shut down Gazebo.
5. **Artifact Generation (T4.5):** The node must output a formatted `report.json` file for the LLM Agentic Tuner to read.
