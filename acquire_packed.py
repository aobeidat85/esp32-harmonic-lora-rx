"""Exploratory harmonic-LoRa preamble search in timed, gapped ESP32 sign-IQ."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.signal import stft
from packed_capture import unpack

FS=2000000
T=128/500000

def reconstruct(blocks):
    starts=np.rint(blocks['start'].astype(np.float64)/120).astype(int)
    samples=unpack(blocks)
    x=np.zeros(starts[-1]+512,np.complex64)
    mask=np.zeros(len(x),bool)
    for a,y in zip(starts,samples):
        x[a:a+512]=y
        mask[a:a+512]=True
    return x,mask

def search(x):
    n=np.arange(len(x));t=n/FS
    # Nominal +1 MHz harmonic offset aliases to Nyquist at this decimation.
    x=x*(1-2*(n%2))
    window=np.hanning(2048)
    frames=np.lib.stride_tricks.sliding_window_view(x,2048)[::256]
    energies=np.sum(abs(frames*window)**2,axis=1)
    best=np.zeros(len(frames));phase=np.zeros(len(frames),int);freq=np.zeros(len(frames))
    bins=np.fft.fftfreq(2048,1/FS);allowed=abs(bins)<300000
    for offset in range(32):
        u=(t+offset*T/32)%T
        ref=np.exp(-3j*np.pi*(500000/T*u*u-500000*u))
        windows=np.lib.stride_tricks.sliding_window_view(x*ref,2048)[::256]
        fft=abs(np.fft.fft(windows*window,axis=1))**2
        fft[:,~allowed]=0
        k=np.argmax(fft,axis=1)
        scores=fft[np.arange(len(k)),k]/np.maximum(2048*energies,1e-9)
        better=scores>best
        best[better]=scores[better];phase[better]=offset;freq[better]=bins[k[better]]
    selected=[]
    for idx in np.argsort(best)[::-1]:
        stamp=(idx*256+1024)/FS
        if any(abs(stamp-r['center_s'])<.004 for r in selected):continue
        selected.append(dict(center_s=stamp,start_s=idx*256/FS,score=float(best[idx]),phase_grid=int(phase[idx]),residual_hz=float(freq[idx])))
        if len(selected)>=12:break
    return selected

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory');args=p.parse_args()
    results=[]
    for file in sorted(Path(args.directory).glob('*.npz')):
        with np.load(file,allow_pickle=False) as d:
            x,mask=reconstruct(d['blocks']);meta=json.loads(str(d['metadata']))
        candidates=search(x)
        row=dict(file=file.name,metadata=meta,candidates=candidates)
        results.append(row)
        print(file.name,'best',round(candidates[0]['score'],4),'time',round(candidates[0]['center_s'],5),'offset',candidates[0]['residual_hz'],flush=True)
    (Path(args.directory)/'acquisition.json').write_text(json.dumps(results,indent=2))
