#!/usr/bin/env python3
"""Spawn quadrotor model by waiting for Gazebo service then calling spawn_sdf_model."""
import rospy
import sys
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest


def main():
    rospy.init_node("spawn_quadrotor_helper")
    
    from geometry_msgs.msg import Pose as PoseMsg
    
    sdf_path = rospy.get_param("~sdf_path", "/ws/src/mid360_gazebo/models/quadrotor_mid360/model.sdf")
    model_name = rospy.get_param("~model_name", "quadrotor_mid360")
    x = rospy.get_param("~x", 0.0)
    y = rospy.get_param("~y", 0.0)
    z = rospy.get_param("~z", 1.0)
    
    # Read SDF file
    with open(sdf_path, "r") as f:
        sdf_xml = f.read()
    
    # Wait for Gazebo spawn service with longer timeout and better error handling
    max_wait = 60.0
    waited = 0.0
    rate = rospy.Rate(5)
    
    print(f"[spawn_helper] Waiting for /gazebo/spawn_sdf_model (max {max_wait}s)...", flush=True)
    while not rospy.is_shutdown() and waited < max_wait:
        try:
            if rospy.has_service("/gazebo/spawn_sdf_model"):
                print("[spawn_helper] Service found!", flush=True)
                break
        except Exception as e:
            pass
        
        if int(waited) % 5 == 0 and waited < max_wait:
            print(f"[spawn_helper] Still waiting... ({waited:.1f}s)", flush=True)
        
        rospy.sleep(0.2)
        waited += 0.2
    
    if rospy.is_shutdown():
        print("[spawn_helper] ROS shutdown while waiting", flush=True)
        return 1
    
    if waited >= max_wait:
        print("[spawn_helper] Timed out waiting for service", flush=True)
        # List available services for debugging
        try:
            import subprocess
            result = subprocess.run(['rosnode', 'list'], capture_output=True, text=True, timeout=5)
            print(f"[spawn_helper] Available nodes:\n{result.stdout}", flush=True)
        except Exception:
            pass
        return 1
    
    # Call spawn service with pose
    try:
        spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        pose = PoseMsg()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        
        response = spawn(
            model_name=model_name,
            xml=sdf_xml,
            robot_namespace="",
            reference_frame="world",
            pose=pose,
            timeout=10.0
        )
        
        if response.success:
            print(f"[spawn_helper] SUCCESS: Model '{model_name}' spawned at ({x}, {y}, {z})")
            return 0
        else:
            print(f"[spawn_helper] FAILED: {response.status_message}")
            return 1
    except rospy.ServiceException as e:
        print(f"[spawn_helper] Service call failed: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
