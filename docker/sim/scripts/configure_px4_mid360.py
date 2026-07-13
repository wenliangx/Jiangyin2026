#!/usr/bin/env python3
"""
configure_px4_mid360.py — 在 PX4 SITL 中创建 iris_mid360 无人机模型

功能:
  1. 从 PX4 内置的 iris 模型克隆出 iris_mid360 模型
  2. 在 SDF 模型中添加一个模拟的 Livox MID360 3D LiDAR 传感器
     - 水平 360°、垂直约 60° 的扫描范围
     - 720 个水平采样点、64 个垂直采样点
     - 测距范围 0.1m~40m，分辨率 2cm
     - 添加高斯噪声 (stddev=0.01m) 模拟真实传感器
     - 使用 velodyne_gazebo_plugins 作为 Gazebo 插件
     - 点云发布到 /mid360/points 话题
  3. 注册新的 1020 机型到 PX4 airframe 列表
  4. 修改 sitl_run.sh 使其加载 libgazebo_ros_api_plugin.so

运行时机: 在 Docker 构建阶段执行（Dockerfile 中 `RUN python3 configure_px4_mid360.py`）
"""
from pathlib import Path
import shutil


px4_home = Path("/opt/PX4-Autopilot")
models = px4_home / "Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"

# 1. 从 iris 模型克隆，创建 iris_mid360 目录
src = models / "iris" / "iris.sdf.jinja"
dst_dir = models / "iris_mid360"
dst_dir.mkdir(parents=True, exist_ok=True)
sdf = src.read_text()
sdf = sdf.replace('<model name="iris">', '<model name="iris_mid360">', 1)
sdf = sdf.replace("<model name='iris'>", "<model name='iris_mid360'>", 1)

# 2. 定义 MID360 LiDAR 传感器（插入到 </model> 标签前）
mid360 = r'''
    <link name="mid360_link">
      <pose>0 0 0.12 0 0 0</pose>         <!-- LiDAR安装在机身上方12cm处 -->
      <inertial>
        <mass>0.265</mass>                  <!-- LiDAR质量 ~265g -->
        <inertia>
          <ixx>0.0002</ixx> <ixy>0</ixy> <ixz>0</ixz>
          <iyy>0.0002</iyy> <iyz>0</iyz>
          <izz>0.0002</izz>
        </inertia>
      </inertial>
      <collision name="mid360_collision">
        <geometry>
          <cylinder>
            <radius>0.035</radius>         <!-- 半径 3.5cm -->
            <length>0.04</length>          <!-- 高度 4cm -->
          </cylinder>
        </geometry>
      </collision>
      <visual name="mid360_visual">
        <geometry>
          <cylinder>
            <radius>0.035</radius>
            <length>0.04</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.02 0.02 0.02 1</ambient>
          <diffuse>0.02 0.02 0.02 1</diffuse>
        </material>
      </visual>
      <!-- Gazebo Ray Sensor 模拟 LiDAR -->
      <sensor name="mid360" type="ray">
        <pose>0 0 0 0 0 0</pose>
        <always_on>true</always_on>
        <visualize>false</visualize>
        <update_rate>10</update_rate>      <!-- 10Hz 扫描频率 -->
        <ray>
          <scan>
            <horizontal>
              <samples>720</samples>        <!-- 水平分辨率 0.5° -->
              <resolution>1</resolution>
              <min_angle>-3.14159265359</min_angle>   <!-- -180° -->
              <max_angle>3.14159265359</max_angle>    <!-- +180° = 360° FOV -->
            </horizontal>
            <vertical>
              <samples>64</samples>         <!-- 垂直64线 -->
              <resolution>1</resolution>
              <min_angle>-0.12217304764</min_angle>   <!-- -7° -->
              <max_angle>0.90757121104</max_angle>    <!-- +52° = 约60° FOV -->
            </vertical>
          </scan>
          <range>
            <min>0.1</min>                  <!-- 最小测距 0.1m -->
            <max>40.0</max>                 <!-- 最大测距 40m -->
            <resolution>0.02</resolution>   <!-- 距离分辨率 2cm -->
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>          <!-- 1cm 标准差高斯噪声 -->
          </noise>
        </ray>
        <!-- 使用 velodyne_gazebo_plugins 发布点云到 /mid360/points -->
        <plugin name="gazebo_ros_mid360_controller" filename="libgazebo_ros_velodyne_laser.so">
          <topicName>/mid360/points</topicName>
          <frameName>mid360_link</frameName>
          <min_range>0.1</min_range>
          <max_range>40.0</max_range>
          <gaussianNoise>0.01</gaussianNoise>
          <organize_cloud>true</organize_cloud>    <!-- 组织为有序点云 -->
        </plugin>
      </sensor>
    </link>
    <!-- 固定关节：将 LiDAR 固定在 base_link 上 -->
    <joint name="mid360_joint" type="fixed">
      <parent>base_link</parent>
      <child>mid360_link</child>
      <pose>0 0 0 0 0 0</pose>
    </joint>
'''

if "</model>" not in sdf:
    raise RuntimeError("Unexpected iris.sdf layout: missing </model>")
sdf = sdf.replace("</model>", mid360 + "\n  </model>", 1)
(dst_dir / "iris_mid360.sdf.jinja").write_text(sdf)

# 3. 创建模型配置文件 model.config
(dst_dir / "model.config").write_text("""<?xml version="1.0"?>
<model>
  <name>iris_mid360</name>
  <version>1.0</version>
  <sdf version="1.6">iris_mid360.sdf</sdf>
  <author>
    <name>Codex</name>
    <email>codex@example.invalid</email>
  </author>
  <description>PX4 Iris quadrotor with a Livox MID360-like 3D lidar publishing /mid360/points.</description>
</model>
""")

# 4. 注册新的 1020 机型（airframe ID = 1020）
airframes = px4_home / "ROMFS/px4fmu_common/init.d-posix/airframes"
shutil.copyfile(
    airframes / "10015_gazebo-classic_iris",
    airframes / "1020_gazebo-classic_iris_mid360",
)
airframes_cmake = airframes / "CMakeLists.txt"
cmake = airframes_cmake.read_text()
cmake = cmake.replace(
    "\t1019_gazebo-classic_iris_dual_gps\n",
    "\t1019_gazebo-classic_iris_dual_gps\n\t1020_gazebo-classic_iris_mid360\n",
    1,
)
airframes_cmake.write_text(cmake)

# 5. 修改 sitl_run.sh 使 Gazebo 加载 ROS API 插件
#    这样 /mid360/points 话题才能被 ROS 感知
sitl_run = px4_home / "Tools/simulation/gazebo-classic/sitl_run.sh"
script = sitl_run.read_text()
script = script.replace(
    "gzserver $verbose $world_path $ros_args &",
    "gzserver -s libgazebo_ros_api_plugin.so $verbose $world_path $ros_args &",
    1,
)
sitl_run.write_text(script)
