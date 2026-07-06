import math
import os

R = 4.5
num_points = 100
track_width = 2.0
half_width = track_width / 2.0

# Offset the track so that the lowest point (where the tangent is +X) 
# exactly aligns with the robot's spawn point at (0.0, 1.75).
offset_x = 0.0
offset_y = 0.0

def get_point(t):
    x = R * (math.cos(t) - 0.25 * math.cos(2*t)) + offset_x
    y = R * math.sin(t) + offset_y
    return x, y

def get_tangent(t):
    dt = 0.01
    x1, y1 = get_point(t - dt)
    x2, y2 = get_point(t + dt)
    dx = x2 - x1
    dy = y2 - y1
    length = math.hypot(dx, dy)
    return dx/length, dy/length

sdf_links = []
for i in range(num_points):
    t = i * 2 * math.pi / num_points
    x, y = get_point(t)
    tx, ty = get_tangent(t)
    
    # Normal vector points inward (to the left of tangent)
    nx, ny = -ty, tx
    
    # Inner wall
    in_x = x + nx * half_width
    in_y = y + ny * half_width
    
    # Outer wall
    out_x = x - nx * half_width
    out_y = y - ny * half_width
    
    # Calculate box length (distance to next point)
    t_next = (i + 1) * 2 * math.pi / num_points
    x_next, y_next = get_point(t_next)
    in_next_x = x_next + nx * half_width
    in_next_y = y_next + ny * half_width
    out_next_x = x_next - nx * half_width
    out_next_y = y_next - ny * half_width
    
    dist_in = math.hypot(in_next_x - in_x, in_next_y - in_y)
    dist_out = math.hypot(out_next_x - out_x, out_next_y - out_y)
    
    yaw = math.atan2(ty, tx)
    
    sdf_links.append(f'''
      <!-- Segment {i} -->
      <link name="outer_{i}">
        <pose>{out_x:.4f} {out_y:.4f} 0.25 0 0 {yaw:.4f}</pose>
        <collision name="col"><geometry><box><size>{dist_out + 0.1:.4f} 0.2 0.5</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>{dist_out + 0.1:.4f} 0.2 0.5</size></box></geometry><material><ambient>1 0 0 1</ambient></material></visual>
      </link>
      <link name="inner_{i}">
        <pose>{in_x:.4f} {in_y:.4f} 0.25 0 0 {yaw:.4f}</pose>
        <collision name="col"><geometry><box><size>{dist_in + 0.1:.4f} 0.2 0.5</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>{dist_in + 0.1:.4f} 0.2 0.5</size></box></geometry><material><ambient>0 0 1 1</ambient></material></visual>
      </link>
    ''')

sdf_content = f'''<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="kidney_track">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation><range>1000</range><constant>0.9</constant><linear>0.01</linear><quadratic>0.001</quadratic></attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="track">
      <static>true</static>
      {"".join(sdf_links)}
    </model>
  </world>
</sdf>
'''

with open('track4.sdf', 'w') as f:
    f.write(sdf_content)

print("Generated track3.sdf successfully!")
