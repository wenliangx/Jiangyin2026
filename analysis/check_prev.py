#!/usr/bin/env python3
import rosbag, numpy as np

for path in ['/home/flag/2026-08-13-16-20-03.bag', '/home/flag/2026-08-13-15-58-47.bag', '/home/flag/2026-08-13-15-36-25.bag']:
    bag = rosbag.Bag(path)
    nmpc_t, nmpc_fdb, nmpc_thr, nmpc_ref = [], [], [], []
    t0 = None
    for topic, msg, t in bag.read_messages(topics=['/nmpc_state']):
        ts = t.to_sec()
        if t0 is None: t0 = ts
        nmpc_t.append(ts - t0)
        nmpc_fdb.append([msg.pos_fdb.x, msg.pos_fdb.y, msg.pos_fdb.z])
        nmpc_thr.append(msg.target.thrust)
        nmpc_ref.append([msg.pos_ref[0].x, msg.pos_ref[0].y, msg.pos_ref[0].z])
    if not nmpc_t:
        print("%s: no nmpc_state" % path.split('/')[-1])
        continue
    t = np.array(nmpc_t); fdb = np.array(nmpc_fdb); thr = np.array(nmpc_thr); ref = np.array(nmpc_ref)
    # steady window: last 60% of flight, vz small
    m = t > t[-1]*0.6
    err = fdb[m] - ref[m]
    print("%s: n=%d span %.1fs | steady: z_err=%.3f x_err=%.3f y_err=%.3f | thrust mean=%.4f" %
          (path.split('/')[-1], len(t), t[-1], err[:,2].mean(), err[:,0].mean(), err[:,1].mean(), thr[m].mean()))
