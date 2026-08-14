#!/usr/bin/env python3
import numpy as np
from numpy import interp

d = np.load('/home/flag/Jiangyin2026/analysis/bag_data.npz')
c = np.load('/home/flag/Jiangyin2026/analysis/cam_data.npz')
nt, nfdb, ot, op = d['nt'], d['nfdb'], d['ot'], d['op']
ct, cp = c['ct'], c['cp']

# compare PX4 local odom vs camera vision odom during NMPC window (4.8-18.1s)
m = (ot > 4.5) & (ot < 18.5)
mc = (ct > 4.5) & (ct < 18.5)
cvx = interp(ot[m], ct[mc], cp[mc,0]); cvy = interp(ot[m], ct[mc], cp[mc,1]); cvz = interp(ot[m], ct[mc], cp[mc,2])
dx = op[m,0]-cvx; dy = op[m,1]-cvy; dz = op[m,2]-cvz
print("=== PX4 local odom - vision odom, NMPC window ===")
print("x: mean=%.3f std=%.3f drift=%.3f" % (dx.mean(), dx.std(), dx[-1]-dx[0]))
print("y: mean=%.3f std=%.3f drift=%.3f" % (dy.mean(), dy.std(), dy[-1]-dy[0]))
print("z: mean=%.3f std=%.3f drift=%.3f" % (dz.mean(), dz.std(), dz[-1]-dz[0]))

# slow-motion of each source in x
print("\n=== x position, both sources, 2s chunks ===")
for t0 in np.arange(4.8, 18.0, 2.0):
    m1 = (ot >= t0) & (ot < t0+2)
    m2 = (ct >= t0) & (ct < t0+2)
    if m1.sum() and m2.sum():
        print("  t=%4.1f-%4.1f: px4 x=%.3f  vision x=%.3f" % (t0, t0+2, op[m1,0].mean(), cp[m2,0].mean()))

# vision velocity noise
print("\n=== vision velocity during hover window ===")
mv = (ct > 6) & (ct < 18)
print("vx: mean=%.3f std=%.3f max=%.3f" % (c['cv'][mv,0].mean(), c['cv'][mv,0].std(), np.abs(c['cv'][mv,0]).max()))
