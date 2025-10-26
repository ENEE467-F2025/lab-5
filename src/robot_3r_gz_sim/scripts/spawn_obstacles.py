#!/usr/bin/env python3
import os
import subprocess
import math
import argparse

# Directory to store generated URDFs
output_dir = os.path.join(os.path.dirname(__file__), "urdf_generated")
os.makedirs(output_dir, exist_ok=True)

# parse CLI args
parser = argparse.ArgumentParser()
parser.add_argument('--robot_pose', type=float, nargs=3, default=[-0.52, 0.01, 0.1])
parser.add_argument('--robot_yaw', type=float, default=-0.06)
args = parser.parse_args()

robot_pose = [-0.52, 0.01, 0.1]
robot_yaw = -0.06

NUM_OBSTACLES = 10
RADIUS = 0.45           # distance from robot
OBS_Z = [1.0, 0.8]      # obstacle heights

positions = []
sizes = []
colors = []
center_z = 0.5
platform_height = 0.755
for i in range(NUM_OBSTACLES):
    angle = 2 * math.pi * i / NUM_OBSTACLES
    x = RADIUS * math.cos(angle)
    y = RADIUS * math.sin(angle)
    if i % 2 == 0:
        sx, sy, sz = 0.1, 0.1, OBS_Z[0]
        z = center_z * OBS_Z[i % 2]
    else:
        sx, sy, sz = 0.15, 0.15, OBS_Z[1]
        z = platform_height + center_z * OBS_Z[i % 2]

    sizes.append([sx, sy, sz])
    colors.append([1.0, 0.64, 0.0, 1.0])

    # transform relative to robot pose
    cos_yaw = math.cos(robot_yaw)
    sin_yaw = math.sin(robot_yaw)
    x_world = robot_pose[0] + cos_yaw * x - sin_yaw * y
    y_world = robot_pose[1] + sin_yaw * x + cos_yaw * y
    z_world = robot_pose[2] + z
    positions.append([x_world, y_world, z_world])

# Compute inertia values 
inertia_values = []
for sx, sy, sz in sizes:
    mass = 1.0
    ixx = mass / 12 * (sy**2 + sz**2)
    iyy = mass / 12 * (sx**2 + sz**2)
    izz = mass / 12 * (sx**2 + sy**2)
    inertia_values.append((ixx, iyy, izz))

# URDF Template
URDF_TEMPLATE = """<?xml version="1.0"?>
<robot name="{name}">
  <link name="{name}">
    <visual>
      <origin xyz="0 0 {sz_half}" rpy="0 0 0"/>
      <geometry>
        <box size="{sx} {sy} {sz}"/>
      </geometry>
      <material name="orange">
        <color rgba="{r} {g} {b} {a}"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 {sz_half}" rpy="0 0 0"/>
      <geometry>
        <box size="{sx} {sy} {sz}"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="1.0"/>
      <inertia ixx="{ixx}" ixy="0.0" ixz="0.0"
               iyy="{iyy}" iyz="0.0" izz="{izz}"/>
    </inertial>

    <!-- Tell Gazebo to treat this model as static -->
  <gazebo reference="{name}">
    <static>true</static>
  </gazebo>
  </link>
</robot>
"""

def generate_urdf_file(name, sx, sy, sz, color, ixx, iyy, izz):
    r, g, b, a = color
    urdf_str = URDF_TEMPLATE.format(
        name=name,
        sx=sx, sy=sy, sz=sz,
        sz_half=sz/2.0,
        r=r, g=g, b=b, a=a,
        ixx=ixx, iyy=iyy, izz=izz
    )
    out_path = os.path.join(output_dir, f"{name}.urdf")
    with open(out_path, 'w') as f:
        f.write(urdf_str)
    return out_path

def spawn_obstacle(model_file, name, pose):
    pose_str = "{} {} {} {} {} {}".format(*(float(p) for p in pose))
    args = [
        "ros2", "run", "ros_gz_sim", "create",
        "-file", model_file,
        "-name", name,
        "-pose", pose_str,
    ]
    print("Running:", " ".join(args))
    print(f"Spawn args -> file: {model_file}, name: {name}, pose: {pose}")
    subprocess.run(args, check=True)

def main():
    for i in range(NUM_OBSTACLES):
        name = f"obstacle_{i+1}"
        sx, sy, sz = sizes[i]
        color = colors[i]
        x, y, z = positions[i]
        ixx, iyy, izz = inertia_values[i]

        model_file = generate_urdf_file(name, sx, sy, sz, color, ixx, iyy, izz)
        pose = [x, y, z, 0.0, 0.0, 0.0]
        print(f"Spawning {name} at {pose}, size {(sx, sy, sz)} -> file: {model_file}")
        spawn_obstacle(model_file, name, pose)

if __name__ == "__main__":
    main()
