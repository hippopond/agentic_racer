# Algo 2: Predictive Equidistant Wall-Follower

## The Failure of "Follow The Gap" (FTG)
Our initial attempt at Algo 2 utilized a standard "Follow The Gap" architecture, which scans the LiDAR to find the deepest open space and steers directly toward it.

**The "Donut" Bug:**
In a narrow, walled track (2.0m wide), the absolute longest LiDAR ray is almost always the one running tangential to the walls (at the extreme edge of the robot's vision, near +/- 90 degrees). The robot locked onto this tangential ray and steered as hard as possible, trapping itself in a perpetual 360-degree donut inside the track.

When we mathematically clamped the steering to prevent the donut, the robot lost its mechanical ability to make sharp corners, resulting in it driving straight into the outside wall at the start line. 

**Conclusion:** Pure FTG is unstable in narrow tracks without complex Disparity Extenders. We pivoted to a more robust solution.

## The Solution: Predictive Equidistant Architecture
We abandoned FTG and evolved Algo 1 into a **Predictive Equidistant** algorithm. Instead of trying to find the center of the track strictly parallel to the rear axle (which causes twitching), it balances the walls *in front* of it.

### Step-by-Step Logic
1.  **Diagonal Prediction:** It extracts two diagonal 20-degree LiDAR cones pointing exactly 45 degrees to the left and 45 degrees to the right.
2.  **Early Apex Detection:** Because it looks diagonally forward, it "sees" the curve long before it enters it. As it approaches a left curve, the right diagonal ray hits the far wall while the left diagonal ray looks infinitely down the track.
3.  **Proportional Steering:** It calculates the error (`left_dist - right_dist`) and applies a proportional gain (`Kp = 0.8`). This causes the robot to smoothly steer into the apex before it even reaches the corner!
4.  **Predictive Braking:** It extracts a 20-degree cone directly straight ahead. If the path ahead is clear (> 3.0m) and steering is minimal, it punches the gas up to `3.0 m/s`. If the front wall gets close, it smoothly brakes down to `0.5 m/s`.

## Expected Performance
*   **Immune to Donuts**: By balancing opposite walls, it can never get trapped in a local minimum steering loop.
*   **Smooth Racing Line**: The predictive 45-degree angle naturally traces a wide, smooth racing line, drastically reducing the "Steering Jerk" seen in Algo 1.
*   **High Speed**: Unclamped steering and predictive braking allow it to confidently reach speeds of 3.0 m/s on straightaways.
