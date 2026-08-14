#!/usr/bin/env python3
import numpy as np

d = np.load('/home/flag/Jiangyin2026/analysis/bag_data.npz')
nt, nref, nfdb, nvfdb, natt = d['nt'], d['nref'], d['nfdb'], d['nvfdb'], d['natt']
nbr, nthr, ot, op, ov = d['nbr'], d['nthr'], d['ot'], d['op'], d['ov']

def yaw(w, x, y, z):
    return np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

print("=== Yaw over NMPC window ===")
yw = yaw(natt[:,0], natt[:,1], natt[:,2], natt[:,3])
print("yaw first: %.3f rad (%.1f deg), last: %.3f rad (%.1f deg), total change: %.3f rad (%.1f deg)" %
      (yw[0], np.degrees(yw[0]), yw[-1], np.degrees(yw[-1]), yw[-1]-yw[0], np.degrees(yw[-1]-yw[0])))
print("mean wz command: %.4f rad/s" % nbr[:,2].mean())
print("yaw drift rate from data: %.4f rad/s" % ((yw[-1]-yw[0])/(nt[-1]-nt[0])))

print("\n=== Z trajectory during climb (first 5s of NMPC) ===")
m5 = nt < nt[0] + 5.0
for t, z in zip(nt[m5][::10], nfdb[m5,2][::10]):
    print("  t=%.1fs  z=%.3f" % (t, z))

print("\n=== Whole-flight odom trajectory ===")
print("odom z: min=%.3f max=%.3f" % (op[:,2].min(), op[:,2].max()))
print("odom xy extent: x [%.3f, %.3f], y [%.3f, %.3f]" % (op[:,0].min(), op[:,0].max(), op[:,1].min(), op[:,1].max()))
# split into 5s bins
bins = np.arange(0, 72, 5)
print("\n5s bins of odom z (median):")
for b in bins:
    m = (ot >= b) & (ot < b+5)
    if m.sum():
        print("  %2d-%2ds: z=%.3f (n=%d)" % (b, b+5, np.median(op[m,2]), m.sum()))

print("\n=== NMPC-window odom comparison (PX4 odom vs nmpc fdb) ===")
# interpolate odom onto nmpc times
from numpy import interp
opx = interp(nt, ot, op[:,0]); opy = interp(nt, ot, op[:,1]); opz = interp(nt, ot, op[:,2])
dx = nfdb[:,0]-opx; dy = nfdb[:,1]-opy; dz = nfdb[:,2]-opz
print("fdb vs odom: dx mean=%.4f max=%.4f | dy mean=%.4f max=%.4f | dz mean=%.4f max=%.4f" %
      (dx.mean(), np.abs(dx).max(), dy.mean(), np.abs(dy).max(), dz.mean(), np.abs(dz).max()))

print("\n=== odom velocity vs nmpc vel_fdb ===")
ovx = interp(nt, ot, ov[:,0]); ovy = interp(nt, ot, ov[:,1]); ovz = interp(nt, ot, ov[:,2])
dv = nvfdb[:,0]-ovx
print("vel x diff: mean=%.4f max=%.4f" % (dv.mean(), np.abs(dv).max()))

print("\n=== Error vs time in 2s chunks (x) ===")
err = nfdb - nref[:,0,:]
for t0 in np.arange(nt[0], nt[-1], 2.0):
    m = (nt >= t0) & (nt < t0+2)
    if m.sum():
        print("  t=%4.1f-%4.1f: x_err=%.3f, y_err=%.3f, z_err=%.3f" %
              (t0, t0+2, err[m,0].mean(), err[m,1].mean(), err[m,2].mean()))

print("\n=== Spectral analysis of x error (steady, t>10) ===")
m = nt > 10
e = err[m,0] - err[m,0].mean()
te = nt[m]
from numpy.fft import rfft, rfftfreq
fs = 50.0
E = np.abs(rfft(e - e.mean()))
f = rfftfreq(len(e), 1/fs)
order = np.argsort(E)[::-1][:5]
for i in order:
    print("  f=%.2f Hz  amp=%.4f m" % (f[i], 2*E[i]/len(e)))
