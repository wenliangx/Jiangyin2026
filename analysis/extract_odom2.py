#!/usr/bin/env python3
import rosbag, numpy as np

bag = rosbag.Bag('/home/flag/2026-08-13-16-22-43.bag')
cam = {'t': [], 'p': [], 'v': []}
t0 = None
for topic, msg, t in bag.read_messages(topics=['/Odometry']):
    ts = t.to_sec()
    if t0 is None: t0 = ts
    cam['t'].append(ts - t0)
    cam['p'].append([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
    cam['v'].append([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
np.savez('/home/flag/Jiangyin2026/analysis/cam_data.npz',
         ct=np.array(cam['t']), cp=np.array(cam['p']), cv=np.array(cam['v']))
print("camera odom msgs:", len(cam['t']), "span %.1f-%.1f" % (cam['t'][0], cam['t'][-1]))
