#!/usr/bin/env python3
"""
Minimal Flag publisher for SUPER testing.
Publishes super_msgs::Flag messages to trigger SUPER planner.
Usage:
  rosrun mission_planner super_flag_pub.py _waypoints:="[0,0,1, 5,0,1, -5,0,1]"
"""
import rospy
import math
from super_msgs.msg import Flag
from geometry_msgs.msg import Point

rospy.init_node("super_flag_pub", anonymous=True)
pub = rospy.Publisher("/super/flag_waypoint", Flag, queue_size=10, latch=True)

# waypoints: x0,y0,z0, x1,y1,z1, ...
raw = rospy.get_param("~waypoints", "0,0,1, 5,0,1")
vals = [float(x) for x in raw.replace(",", " ").split()]
n = len(vals) // 3
rospy.loginfo("Publishing %d waypoints to /super/flag_waypoint", n)

rospy.sleep(2.0)  # wait for subscribers

for i in range(n):
    f = Flag()
    f.header.frame_id = "world"
    f.header.stamp = rospy.Time.now()
    f.id = i
    f.mode = 1        # normal speed
    f.is_map = 1      # enable obstacle mapping
    f.yaw = math.nan  # free terminal yaw
    f.desired_speed = 0.0
    f.position = Point(vals[3*i], vals[3*i+1], vals[3*i+2])
    pub.publish(f)
    rospy.loginfo("Flag %d -> (%.1f, %.1f, %.1f)", i, f.position.x, f.position.y, f.position.z)
    rospy.sleep(0.3)

rospy.loginfo("Done. SUPER should now be planning.")
rospy.spin()  # keep alive (latch retains last message)
