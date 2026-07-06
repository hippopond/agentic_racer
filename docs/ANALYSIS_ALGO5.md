# Algo 5: Ultra-Grip Equidistant Architecture

## 1. The Proportional-Derivative (PD) Failure
Our initial goal for Algo 5 was to safely raise the speed cap to `4.0 m/s` by completely eliminating understeer. To do this, we implemented a full **Proportional-Derivative (PD) controller**. The theory was that the Derivative term (`Kd`) would measure the rate of change of the distance to the walls, anticipate the upcoming curves, and violently snap the steering into the apex early.

However, the PD controller proved to be fundamentally flawed for **continuous curve wall-following**.
*   **The Artifact Explosion:** When the robot passed sharp inner corners at the waist of the track, the LiDAR beam would instantly slip past the edge of the wall, causing distance readings to jump from `10.0m` to `2.0m` in a single 0.1s frame. This created a massive, artificial `error_dot` spike, causing the robot to violently yank the steering wheel and spin out completely backwards off the track. We fixed this by clamping `error_diff` to `0.5m` per frame.
*   **The D-Term Understeer:** Even with the clamped artifact fix, the robot still understeered and crashed into the walls! Why? Because a PD controller is designed to reach a fixed straight line. As the robot approached the center of a continuous curve, the error started *decreasing*. Because the error was decreasing, the Derivative term became **negative** and actively fought the Proportional term to prevent "overshooting"! This caused the robot to "relax" its steering exactly when it needed it most, throwing it into the wall.

## 2. The Final Architecture: Ultra-Grip P-Controller
We completely nuked the PD controller and replaced it with an optimized High-Gain Proportional controller for maximum grip:
1.  **Removed the Derivative Term:** Stopped the algorithm from fighting itself during cornering.
2.  **Boosted Proportional Gain:** Cranked `Kp` to `2.5` so the robot aggressively attacks the center line without second-guessing.
3.  **Unlocked Steering Limits:** We realized we were artificially limiting the steering to `3.0 rad/s`. Differential drive robots don't have a mechanical steering rack, so we raised the max steering limit to **`4.0 rad/s`**. This allows the robot to carve a significantly tighter turning radius without having to sacrifice speed!
4.  **Max Speed:** Retained at `3.0 m/s` for optimal stability and no-tunneling guarantees, while maintaining the Anti-Pirouette floor at `1.5 m/s`.
