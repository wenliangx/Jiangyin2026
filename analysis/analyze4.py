#!/usr/bin/env python3
import numpy as np
from numpy.fft import rfft, rfftfreq

d = np.load('/home/flag/Jiangyin2026/analysis/bag_data.npz')
nt, nref, nfdb, nvfdb = d['nt'], d['nref'], d['nfdb'], d['nvfdb']
nbr = d['nbr']

err = nfdb - nref[:,0,:]
m = nt > 10
fs = 50.0

print("=== FFT of wy command (t>10s) ===")
w = nbr[m,1] - nbr[m,1].mean()
E = np.abs(rfft(w)); f = rfftfreq(len(w), 1/fs)
for i in np.argsort(E)[::-1][:5]:
    print("  f=%.2f Hz  amp=%.4f rad/s" % (f[i], 2*E[i]/len(w)))

print("\n=== FFT of x error (t>10s) ===")
e = err[m,0] - err[m,0].mean()
E = np.abs(rfft(e)); f = rfftfreq(len(e), 1/fs)
for i in np.argsort(E)[::-1][:5]:
    print("  f=%.2f Hz  amp=%.4f m" % (f[i], 2*E[i]/len(e)))

print("\n=== cross-correlation wy -> x_err (wy leads x_err by lag) ===")
wy = nbr[m,1]; xe = err[m,0]
wy = (wy - wy.mean()) / wy.std(); xe = (xe - xe.mean()) / xe.std()
n = len(wy)
for lag in [0, 2, 5, 10, 15, 20, 25, 30]:
    if lag < n:
        c = np.corrcoef(wy[:-lag], xe[lag:])[0,1]
        print("  lag %3d (%.1f ms): corr=%.3f" % (lag, lag*20, c))

print("\n=== x_err -> wy (x_err leads wy by lag) ===")
for lag in [0, 2, 5, 10, 15, 20, 25, 30]:
    if lag < n:
        c = np.corrcoef(xe[:-lag], wy[lag:])[0,1]
        print("  lag %3d (%.1f ms): corr=%.3f" % (lag, lag*20, c))

print("\n=== Decay of |x_err| envelope, last 10s (linear fit) ===")
t = nt[m]; xa = np.abs(err[m,0])
# moving average over 1s
win = 50
env = np.convolve(xa, np.ones(win)/win, mode='valid')
tt = t[win//2:win//2+len(env)]
print("  envelope start: %.3f m, end: %.3f m" % (env[0], env[-1]))
k = np.polyfit(tt, np.log(env+1e-6), 1)
print("  decay rate: %.3f 1/s -> time constant %.1f s" % (-k[0], -1/k[0]))

print("\n=== Compare: implied closed-loop acc from vx derivative ===")
m10 = nt > 10
vx = nvfdb[m10,0]
ax = np.gradient(vx, 0.02)
print("  mean |ax| = %.3f m/s^2, max |ax| = %.3f m/s^2" % (np.abs(ax).mean(), np.abs(ax).max()))
print("  (correcting 0.2m in ~4s needs ~0.1-0.2 m/s^2)" )
