# Algo 3: Disparity Extender vs. Predictive Equidistant

## The F1Tenth Disparity Extender (Failed)
Our initial attempt at Algo 3 was implementing the F1Tenth gold-standard "Disparity Extender" algorithm. 
The algorithm scans for sudden jumps in LiDAR depth (disparities) and mathematically extends the closer wall over the further wall by half the width of the car to create a virtual safety bubble around corner apexes.

**The Flaw: Smooth Spline Tracks**
The Disparity Extender requires a depth disparity `> 0.5m` to trigger. Because the Kidney Track is built using perfectly smooth continuous splines, the wall distance changes smoothly. The algorithm rarely detected a disparity, bypassing the extender and falling back to raw Follow-The-Gap. 
When we applied heavy forward-weighting (`Cosine^4`) to prevent the FTG donut-spin bug, the robot fiercely prioritized going straight and understeered directly into the outside wall of the turn.

**Conclusion:** Disparity Extender is fantastic for sharp, angular tracks or doorways, but highly flawed for smooth, continuous curves without global odometry.

## The Final Architecture: Predictive Equidistant
We threw out the Follow-The-Gap architecture entirely and designed **Predictive Equidistant**.

### How it Works
Instead of searching for a single "gap" or target point, this algorithm actively balances the physical walls of the track.
1. **Diagonal Prediction:** It extracts two 20-degree LiDAR cones pointing exactly 45 degrees left and 45 degrees right.
2. **Pre-Apex Steering:** As the robot approaches a left curve, the right diagonal ray hits the outside wall, while the left diagonal ray shoots infinitely down the track. The error (`left - right`) instantly spikes, causing the robot to smoothly steer into the apex *long before* the robot physically reaches the corner.
3. **Anti-Donut Guarantee:** Because it requires opposite walls to balance, it is mathematically impossible for the robot to get trapped in a 360-degree local minimum spin.

### Performance
*   **Unclamped Steering:** Because it naturally traces wide lines, we removed the steering clamps, allowing it to utilize its full 2.5 rad/s turning radius for hairpin emergencies.
*   **Dynamic Braking:** It utilizes a straight-ahead LiDAR cone to dynamically punch the throttle to `3.0 m/s` on straightaways and brake to `0.5 m/s` when a wall approaches.
*   **Stability:** This architecture is flawlessly stable on smooth spline tracks, completing laps continuously without user intervention.
