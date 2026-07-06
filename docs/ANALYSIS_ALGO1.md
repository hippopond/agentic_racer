# Algo 1: Pure Reactive Equidistant Wall-Follower (Telemetry Review)

## Architecture
Algo 1 strips away all localization, global odometry mapping, and complex path-planning in favor of a purely reactive, sensor-driven approach. It relies exclusively on the raw data stream from the `gpu_lidar` to maintain equal distance between the left and right walls of the track.

## Telemetry Report (`report.json`)
*   **Total Elapsed Time**: `99.5 seconds`
*   **Total Distance (Odometry)**: `37.15 meters`
*   **Crashes**: `0` (Flawless physical survival)
*   **Max Steering Jerk**: `9.08 rad/s²` (High twitchiness as it constantly corrects)
*   **Maximum Cross-Track Error (CTE)**: `> 8.5 meters` (Impossible physical metric)

## The Breakthrough Discovery: "Odometry Hallucinations"
Analysis of the `report.json` telemetry revealed a critical flaw in the robot's state estimation: **Severe Odometry Drift**. 

Because Algo 1 is purely reactive, we know for a fact the robot never physically left the track boundaries (it recorded 0 crashes). However, the internal Odometry coordinates (`x`, `y`) recorded the robot wandering off to `X = 14.9m`, well beyond the absolute physical boundary of the Kidney Track (`Maximum X = 6.375m`).

**Cause:** When the differential-drive robot corners at 1.0 m/s, the wheels slip. Because the robot lacks an EKF (Extended Kalman Filter) fusing an IMU with the wheel encoders, the raw odometry dead-reckoning assumes the wheels gripped perfectly. The internal map quickly drifts into the abyss, leading to impossible CTE values and "Wrong Way" hallucinations.

## Strategy for Algo 2
**CRITICAL RULE:** We **CANNOT** use global Odometry (`/odom`) or predefined global `(x, y)` waypoints (e.g., global Pure Pursuit) for racing line optimization. The internal map is fundamentally broken.

To beat Algo 1's speed without crashing, Algo 2 must be **100% Sensor-Local**:
1.  **Deepest Gap Targeting (Follow The Gap)**: Instead of finding the track center, Algo 2 should scan the LiDAR array to find the "deepest" open gap (the farthest point it can see before hitting a wall).
2.  **Apex Hugging**: By steering toward the deepest open space, the robot will naturally cut corners and hug the inside apex, drastically shortening the total distance traveled.
3.  **Local Kinematic Speed Control**: Speed should be proportional to the distance of the deepest gap. Long straightaway (far gap) = high speed. Short gap (approaching a wall) = heavy braking.

This approach achieves a highly optimized "racing line" purely through raw LiDAR geometry, completely bypassing the odometry drift problem!
