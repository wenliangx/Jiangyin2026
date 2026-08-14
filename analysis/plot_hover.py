#!/usr/bin/env python3
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# --- reference palette (dataviz skill) ---
SURFACE = '#fcfcfb'
INK = '#0b0b0b'
SEC = '#52514e'
MUTED = '#898781'
GRID = '#e1e0d9'
BASE = '#c3c2b7'
BLUE = '#2a78d6'
ORANGE = '#eb6834'
AQUA = '#1baf7a'

d = np.load('/home/flag/Jiangyin2026/analysis/bag_data.npz')
nt, nref, nfdb, nvfdb = d['nt'], d['nref'], d['nfdb'], d['nvfdb']
nbr, nthr = d['nbr'], d['nthr']

err = nfdb - nref[:, 0, :]

plt.rcParams.update({
    'font.family': 'sans-serif',
    'font.sans-serif': ['DejaVu Sans'],
    'figure.facecolor': SURFACE,
    'axes.facecolor': SURFACE,
    'axes.edgecolor': BASE,
    'axes.labelcolor': INK,
    'text.color': INK,
    'xtick.color': MUTED, 'ytick.color': MUTED,
    'axes.grid': True,
    'grid.color': GRID,
    'grid.linewidth': 0.8,
})

fig, axes = plt.subplots(2, 2, figsize=(11, 6.5), sharex=True)
fig.subplots_adjust(hspace=0.28, wspace=0.22)

t = nt - nt[0]

# 1. z: reference vs actual
ax = axes[0, 0]
ax.plot(t, nref[:, 0, 2], color=BLUE, lw=2, ls='--', label='z reference (0.40 m)')
ax.plot(t, nfdb[:, 2], color=ORANGE, lw=2, label='z actual')
ax.axvspan(10, 18, color=GRID, alpha=0.5)
ax.annotate('steady: -8.7 cm below ref', xy=(14, 0.335), xytext=(8.2, 0.42),
            color=SEC, fontsize=9)
ax.set_ylabel('height [m]')
ax.legend(loc='lower left', frameon=False, fontsize=8)
ax.set_title('Altitude hold', fontsize=10, loc='left', color=INK)

# 2. x error
ax = axes[0, 1]
ax.plot(t, err[:, 0], color=ORANGE, lw=2, label='x error')
ax.axhline(0, color=BASE, lw=1)
ax.annotate('wander ±0.2 m,\nslow recovery ~4 s', xy=(9.5, 0.21), xytext=(10.3, 0.12),
            color=SEC, fontsize=9)
ax.set_ylabel('x error [m]')
ax.legend(loc='lower right', frameon=False, fontsize=8)
ax.set_title('Horizontal hold (x)', fontsize=10, loc='left', color=INK)

# 3. thrust
ax = axes[1, 0]
ax.plot(t, nthr, color=AQUA, lw=2, label='thrust command')
ax.axhline(0.430, color=BLUE, lw=2, ls='--', label='nmpc_hover_thrust = 0.430')
ax.axhline(0.456, color=MUTED, lw=1, ls=':')
ax.annotate('measured hover thrust ≈ 0.456\n(param 6% too low)', xy=(13, 0.457),
            xytext=(5.5, 0.468), color=SEC, fontsize=9)
ax.set_ylabel('thrust [normalized]')
ax.set_ylim(0.42, 0.49)
ax.legend(loc='lower right', frameon=False, fontsize=8)
ax.set_title('Thrust command vs hover thrust param', fontsize=10, loc='left', color=INK)

# 4. pitch rate command
ax = axes[1, 1]
ax.plot(t, nbr[:, 1], color=ORANGE, lw=1.2, label='wy command')
ax.annotate('~1.6 Hz dither,\nnearly uncorrelated with x error', xy=(14.5, 0.45),
            xytext=(8.6, 0.52), color=SEC, fontsize=9)
ax.set_ylabel('pitch rate wy [rad/s]')
ax.set_xlabel('time since NMPC active [s]')
ax.legend(loc='lower right', frameon=False, fontsize=8)
ax.set_title('Pitch-rate command', fontsize=10, loc='left', color=INK)

fig.suptitle('Hover control — 2026-08-13-16-22-43.bag (MissionMachine NmpcHover, target (0, 0, 0.4))',
             fontsize=12, color=INK, y=0.98)
fig.savefig('/home/flag/Jiangyin2026/analysis/hover_analysis.png', dpi=150,
            bbox_inches='tight', facecolor=SURFACE)
print("saved hover_analysis.png")
