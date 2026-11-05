"""Plot actual ESP32 captures from the harmonic experiment."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import stft
from harmonic_probe import iq_from_words

root = Path('harmonic-captures')
on = json.loads((root/'transmitter-on-repeat/analysis.json').read_text())
best = on['rows'][0]['file']
items = [(root/'sweep-02/0002.npz', 'Initial transmitter ON: repeating chirps'),
         (root/'transmitter-off-control/0005.npz', 'Transmitter OFF: control'),
         (root/'transmitter-on-repeat'/best, 'Transmitter ON again: fresh capture')]
fig, axes = plt.subplots(3, 1, figsize=(10, 9), constrained_layout=True)
for ax, (path, title) in zip(axes, items):
    with np.load(path, allow_pickle=False) as data:
        x = iq_from_words(data['words'])
        meta = json.loads(str(data['metadata']))
    f, t, z = stft(x-np.mean(x), fs=16000000, nperseg=512, noverlap=448,
                   return_onesided=False, boundary=None)
    order = np.argsort(f)
    f, z = f[order], z[order]
    keep = (f > -200000) & (f < 2200000)
    # Per-panel noise reference exposes structure; colors are not calibrated dBm
    # and should not be used to compare absolute received power between panels.
    db = 20*np.log10(abs(z[keep])+1e-6)
    db -= np.median(db)
    ax.pcolormesh(t*1e6, f[keep]/1e6, db, shading='auto', cmap='magma', vmin=-2, vmax=22)
    ax.set(title=f'{title} ({path.name}, gain {meta["gain_index"]})',
           xlabel='Time in each 1.024 ms snapshot (µs)', ylabel='IQ offset (MHz)')
fig.suptitle('Original ESP32: experimental reception near requested 2744 MHz\nMeasured RF samples; chirp detection, no decoded packet', fontsize=13)
fig.savefig(root/'on-off-on.png', dpi=150)
print((root/'on-off-on.png').resolve())
