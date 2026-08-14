#!/usr/bin/env python3
import numpy as np

d = np.load('/home/flag/Jiangyin2026/analysis/bag_data.npz')
nt, nref, nfdb, nvfdb, natt = d['nt'], d['nref'], d['nfdb'], d['nvfdb'], d['natt']
nbr, nthr, ot, op, ov = d['nbr'], d['nthr'], d['ot'], d['op'], d['ov']

print("=== 1. NMPC solve rate ===")
dt = np.diff(nt)
print("median dt: %.1f ms, mean: %.1f ms, max: %.1f ms" % (np.median(dt)*1e3, dt.mean()*1e3, dt.max()*1e3))
print("samples with dt>25ms: %d/%d" % (np.sum(dt>0.025), len(dt)))

print("\n=== 2. Reference ===")
print("ref[0] first: (%.3f, %.3f, %.3f)  last: (%.3f, %.3f, %.3f)" %
      (*nref[0,0], *nref[-1,0]))
print("ref[8] first: (%.3f, %.3f, %.3f)  last: (%.3f, %.3f, %.3f)" %
      (*nref[0,8], *nref[-1,8]))

err = nfdb - nref[:, 0, :]   # error vs first horizon point
print("\n=== 3. Steady-state error (t>10s, vs ref[0]) ===")
mask = nt > 10.0
for i, ax in enumerate(['x', 'y', 'z']):
    e = err[mask, i]
    print("%s: mean=%.3f m, std=%.3f m, rms=%.3f m, max|e|=%.3f m" %
          (ax, e.mean(), e.std(), np.sqrt((e**2).mean()), np.abs(e).max()))

print("\n=== 4. Position std over windows (oscillation measure) ===")
for i, ax in enumerate(['x', 'y', 'z']):
    fdb_ax = nfdb[:, i]
    w1 = fdb_ax[(nt > 5) & (nt <= 10)]
    w2 = fdb_ax[(nt > 10) & (nt <= 14)]
    w3 = fdb_ax[(nt > 14)]
    print("%s std: 5-10s=%.4f  10-14s=%.4f  14-18s=%.4f (m)" %
          (ax, w1.std() if len(w1) else -1, w2.std() if len(w2) else -1,
           w3.std() if len(w3) else -1))

print("\n=== 5. Peak-to-peak error in steady window (t in [10,18]) ===")
m = (nt >= 10) & (nt <= 18)
for i, ax in enumerate(['x', 'y', 'z']):
    e = err[m, i]
    print("%s: p2p=%.3f m" % (ax, e.ptp()))

print("\n=== 6. Velocity fdb stats (steady window) ===")
for i, ax in enumerate(['vx', 'vy', 'vz']):
    v = nvfdb[m, i]
    print("%s: mean=%.3f std=%.3f max|v|=%.3f m/s" % (ax, v.mean(), v.std(), np.abs(v).max()))

print("\n=== 7. Thrust & body-rate commands (steady window) ===")
thr = nthr[m]
print("thrust: mean=%.3f std=%.4f min=%.3f max=%.3f (normalized)" %
      (thr.mean(), thr.std(), thr.min(), thr.max()))
for i, ax in enumerate(['wx', 'wy', 'wz']):
    br = nbr[m, i]
    print("%s: mean=%.4f std=%.4f max|.|=%.4f rad/s" % (ax, br.mean(), br.std(), np.abs(br).max()))

print("\n=== 8. Error zero-crossing rate (oscillation frequency, x/y) ===")
for i, ax in enumerate(['x', 'y']):
    e = err[m, i] - err[m, i].mean()
    crossings = np.sum((e[:-1] * e[1:]) < 0)
    dur = nt[m][-1] - nt[m][0]
    print("%s: %d zero crossings over %.1fs -> %.2f Hz" % (ax, crossings, dur, crossings/2/dur))

print("\n=== 9. Early response (first 3s of NMPC active) ===")
m0 = nt < nt[0] + 3.0
p0 = nfdb[0]
print("start pos: (%.3f, %.3f, %.3f)" % (p0[0], p0[1], p0[2]))
e0 = err[m0]
print("max error magnitude first 3s: %.3f m" % np.sqrt((e0**2).sum(axis=1)).max())
p3 = nfdb[m0][-1]
print("end of 3s: (%.3f, %.3f, %.3f)" % (p3[0], p3[1], p3[2]))
