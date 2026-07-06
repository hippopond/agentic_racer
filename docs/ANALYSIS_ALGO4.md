# Algo 4: High-Grip Equidistant Architecture

## 1. The Gazebo Tunneling Phenomenon
When upgrading to a massive, curved kidney track (`track4.sdf`), we discovered a critical flaw in high-speed Gazebo simulation. If the robot approaches a jagged, discrete collision mesh (built from hundreds of rotated boxes) at speeds exceeding `3.0 m/s` with a shallow angle (understeer), the default Gazebo Harmonic physics solver fails to resolve the collision. The robot effectively "tunnels" or phases right through the track wall into the void.

**Solution:**
We hard-capped the maximum speed at `3.0 m/s` and increased the proportional steering gain (`Kp = 2.0`) to aggressively track the center line and prevent the shallow-angle understeer that triggers tunneling.

## 2. The Pirouette Loop Bug
To prevent tunneling, we implemented a heavy proportional braking penalty when the robot steers aggressively. However, at the narrow "waist" of the figure-8 track, steering spiked to maximum (`2.85 rad/s`). The heavy braking dropped the robot's forward speed to `0.5 m/s`. 
For a differential drive robot, `Turning Radius = Speed / Angular Velocity`.
At `0.5 m/s` and `2.85 rad/s`, the turning radius shrank to **0.17 meters**. Because the chassis is `0.5m` long, the robot began spinning in place in an infinitely tight donut (a "pirouette"), visually appearing as if it had stopped dead on the track without hitting anything.

**Solution (Anti-Pirouette Control):**
We implemented a guaranteed minimum forward velocity of `1.5 m/s`. This forces the robot to maintain forward momentum, causing it to carve a wider arc through the narrow waist instead of stalling into a pirouette.

## 3. The Final Architecture
Algo 4 strikes a perfect balance between speed and stability for complex, high-curvature tracks:
- **Peripheral Gaze:** Uses 45-degree offset cones (negative feedback zone) to balance wall distances.
- **Aggressive Center-Tracking:** `Kp = 2.0` ensures the robot dives into corners.
- **Dynamic Speed Window:** `speed = max(1.5, min(3.0, 1.0 + front_dist*0.6 - abs(steering)*0.8))`. This guarantees enough speed to prevent pirouettes while braking enough to prevent tunneling.
- **Lap Time:** Completed a blistering 11.05s lap on the Kidney Track.
