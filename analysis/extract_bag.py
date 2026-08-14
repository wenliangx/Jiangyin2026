#!/usr/bin/env python3
import rosbag, numpy as np, sys

bag = rosbag.Bag('/home/flag/2026-08-13-16-22-43.bag')

nmpc = {'t': [], 'ref': [], 'fdb': [], 'vfdb': [], 'att': [], 'bodyrate': [], 'thrust': []}
odom = {'t': [], 'p': [], 'v': []}

t0 = None
for topic, msg, t in bag.read_messages():
    ts = t.to_sec()
    if t0 is None: t0 = ts
    if topic == '/nmpc_state':
        nmpc['t'].append(ts - t0)
        nmpc['ref'].append([[msg.pos_ref[i].x, msg.pos_ref[i].y, msg.pos_ref[i].z] for i in range(9)])
        nmpc['fdb'].append([msg.pos_fdb.x, msg.pos_fdb.y, msg.pos_fdb.z])
        nmpc['vfdb'].append([msg.vel_fdb.x, msg.vel_fdb.y, msg.vel_fdb.z])
        nmpc['att'].append([msg.attitude_fdb.w, msg.attitude_fdb.x, msg.attitude_fdb.y, msg.attitude_fdb.z])
        nmpc['bodyrate'].append([msg.target.body_rate.x, msg.target.body_rate.y, msg.target.body_rate.z])
        nmpc['thrust'].append(msg.target.thrust)
    elif topic == '/mavros/local_position/odom':
        odom['t'].append(ts - t0)
        odom['p'].append([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        odom['v'].append([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])

np.savez('/home/flag/Jiangyin2026/analysis/bag_data.npz',
         nt=np.array(nmpc['t']), nref=np.array(nmpc['ref']), nfdb=np.array(nmpc['fdb']),
         nvfdb=np.array(nmpc['vfdb']), natt=np.array(nmpc['att']),
         nbr=np.array(nmpc['bodyrate']), nthr=np.array(nmpc['thrust']),
         ot=np.array(odom['t']), op=np.array(odom['p']), ov=np.array(odom['v']))
print("nmpc msgs:", len(nmpc['t']), " odom msgs:", len(odom['t']))
print("nmpc time span: %.2f - %.2f" % (nmpc['t'][0], nmpc['t'][-1]))
print("odom time span: %.2f - %.2f" % (odom['t'][0], odom['t'][-1]))
