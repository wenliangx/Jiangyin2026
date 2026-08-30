#!/usr/bin/env python3
import argparse
import math
import socket
import sys
import time

import rospy
from gazebo_msgs.srv import GetLinkState, GetModelState
from geometry_msgs.msg import PoseStamped
from livox_ros_driver2.msg import CustomMsg
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2


def wait_message(topic, message_type, timeout=30.0):
    return rospy.wait_for_message(topic, message_type, timeout=timeout)


def pitch_from_quaternion(q):
    value = 2.0 * (q.w * q.y - q.z * q.x)
    return math.asin(max(-1.0, min(1.0, value)))


def send_command(command):
    payload = f"{command},0.000,0.000,0.000".encode()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.sendto(payload, ("127.0.0.1", 12001))


def check_mount():
    rospy.wait_for_service("/gazebo/get_link_state", timeout=20.0)
    get_link_state = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
    state = get_link_state("iris_mid360::mid360_link", "iris_mid360::base_link")
    if not state.success:
        raise RuntimeError(state.status_message)
    pitch_degrees = math.degrees(pitch_from_quaternion(state.link_state.pose.orientation))
    z = state.link_state.pose.position.z
    if abs(z - 0.12) > 0.03 or abs(pitch_degrees + 15.0) > 1.0:
        raise RuntimeError(f"unexpected MID360 mount: z={z:.3f}, pitch={pitch_degrees:.2f} deg")
    print(f"mount: z={z:.3f} m, pitch={pitch_degrees:.2f} deg")


def check_topics():
    mavros_state = wait_message("/mavros/state", State)
    if not mavros_state.connected:
        raise RuntimeError("MAVROS is not connected to PX4")

    cloud = wait_message("/mid360/points", PointCloud2)
    fields = {field.name for field in cloud.fields}
    if cloud.width * cloud.height < 1000 or not {"x", "y", "z", "intensity", "ring"} <= fields:
        raise RuntimeError(f"invalid simulated cloud: {cloud.width}x{cloud.height}, fields={fields}")

    livox = wait_message("/livox/lidar", CustomMsg)
    if livox.point_num < 1000 or len(livox.points) != livox.point_num:
        raise RuntimeError(f"invalid Livox bridge output: {livox.point_num} points")
    if max(point.offset_time for point in livox.points) < 50_000_000:
        raise RuntimeError("Livox per-point timestamps are not in nanoseconds")

    wait_message("/livox/imu", Imu)
    odometry = wait_message("/Odometry", Odometry, timeout=60.0)
    wait_message("/mavros/local_position/pose", PoseStamped)
    print(
        f"topics: cloud={cloud.width * cloud.height}, livox={livox.point_num}, "
        f"odom_z={odometry.pose.pose.position.z:.3f}, px4_mode={mavros_state.mode}"
    )


def flight_test(timeout=45.0):
    rospy.wait_for_service("/gazebo/get_model_state", timeout=20.0)
    get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    initial_truth = get_model_state("iris_mid360", "world").pose.position
    initial_odom = wait_message("/Odometry", Odometry).pose.pose.position
    initial = wait_message("/mavros/local_position/pose", PoseStamped).pose.position.z
    send_command(2)
    deadline = time.monotonic() + timeout
    last_z = initial
    while time.monotonic() < deadline:
        state = wait_message("/mavros/state", State, timeout=3.0)
        pose = wait_message("/mavros/local_position/pose", PoseStamped, timeout=3.0)
        last_z = pose.pose.position.z
        if state.armed and state.mode == "OFFBOARD" and last_z > 0.30:
            final_truth = get_model_state("iris_mid360", "world").pose.position
            final_odom = wait_message("/Odometry", Odometry).pose.pose.position
            truth_delta = (
                final_truth.x - initial_truth.x,
                final_truth.y - initial_truth.y,
                final_truth.z - initial_truth.z,
            )
            odom_delta = (
                final_odom.x - initial_odom.x,
                final_odom.y - initial_odom.y,
                final_odom.z - initial_odom.z,
            )
            errors = tuple(abs(odom - truth) for odom, truth in zip(odom_delta, truth_delta))
            if max(errors) > 0.12:
                raise RuntimeError(
                    "RA-LIO delta disagrees with Gazebo truth: "
                    f"truth={truth_delta}, odom={odom_delta}, abs_error={errors}"
                )
            print(f"flight: armed=True, mode=OFFBOARD, z={last_z:.3f} m")
            print(
                "localization: "
                f"truth_delta=({truth_delta[0]:.3f}, {truth_delta[1]:.3f}, {truth_delta[2]:.3f}), "
                f"odom_delta=({odom_delta[0]:.3f}, {odom_delta[1]:.3f}, {odom_delta[2]:.3f}), "
                f"max_error={max(errors):.3f} m"
            )
            return
    raise RuntimeError(f"hover test failed: initial_z={initial:.3f}, final_z={last_z:.3f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--flight", action="store_true", help="command NMPC hover and verify takeoff")
    args = parser.parse_args()

    rospy.init_node("sim_stack_smoke", anonymous=True)
    check_mount()
    check_topics()
    if args.flight:
        flight_test()
    print("SIM_STACK_SMOKE_OK")


if __name__ == "__main__":
    try:
        main()
    except Exception as error:  # noqa: BLE001 - command-line diagnostic
        print(f"SIM_STACK_SMOKE_FAILED: {error}", file=sys.stderr)
        raise
