#!/usr/bin/env python3
import numpy as np

d = np.load('/home/flag/Jiangyin2026/analysis/bag_data.npz')
nt, nref, nfdb, nvfdb, natt = d['nt'], d['nref'], d['nfdb'], d['nvfdb'], d['natt']
nbr, nthr = d['nbr'], d['nthr']

err = nfdb - nref[:,0,:]

print("=== correlation: wy cmd vs x error (should be positive: err_x>0 needs +wy? sign convention) ===")
m = nt > 5
c = np.corrcoef(nbr[m,1], err[m,0])[0,1]
print("corr(wy, x_err) = %.3f" % c)
c = np.corrcoef(nbr[m,0], err[m,1])[0,1]
print("corr(wx, y_err) = %.3f" % c)

print("\n=== lag analysis: wy cmd vs vx (impulse response) ===")
# sample at 50Hz, compute lagged correlation
m = (nt > 5) & (nt < 18)
wy = nbr[m,1]; vx = nvfdb[m,0]
lags = range(0, 20)
for lag in lags:
    c = np.corrcoef(wy[:-lag] if lag else wy, vx[lag:] if lag else vx)[0,1]
    print("  lag %2d (%.1fms): corr=%.3f" % (lag, lag*20, c))

print("\n=== timeline: t, x_err, wy, vx, y_err, wx (t=5-18s, every 0.4s) ===")
m = nt > 5
idx = np.where(m)[0][::20]
print("    t     x_err   vx     wy     y_err   vy     wx")
for i in idx:
    print("  %5.1f  %6.3f %6.3f %6.3f  %6.3f %6.3f %6.3f" %
          (nt[i], err[i,0], nvfdb[i,0], nbr[i,1], err[i,1], nvfdb[i,1], nbr[i,0]))

print("\n=== acc_z commanded implied: thrust / (hover_thrust/g) ===")
acc_imp = nthr * 9.8015 / 0.430
m = nt > 5
print("steady acc_z implied: mean=%.2f m/s^2 (g=9.80)" % acc_imp[m].mean())
