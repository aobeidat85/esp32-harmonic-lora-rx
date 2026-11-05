"""Collect timestamped ESP32 frequency estimates across 150 ms RF windows."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time
import zlib
import numpy as np

DTYPE = np.dtype([('us','<u4'), ('f16','<f4'), ('f32','<f4'), ('coherence','<f4'), ('power','<f4')])

def collect(args):
    import serial
    out=Path(args.output)
    out.mkdir(parents=True,exist_ok=False)
    port=serial.Serial(port=None,baudrate=args.baud,timeout=3,write_timeout=3)
    port.dtr=False;port.rts=False;port.port=args.port
    results=[]
    with port:
        identity=''
        for _ in range(3):
            port.reset_input_buffer();port.write(b'INFO\n')
            identity=port.readline().decode(errors='replace').strip()
            if identity.startswith(tuple(f'HRXLORA{v} ESP32-D0WD-V3 ' for v in range(2,8))):break
        if not identity.startswith(tuple(f'HRXLORA{v} ESP32-D0WD-V3 ' for v in range(2,8))):raise ValueError(identity)
        port.write(b'STOP\n');assert port.readline().strip()==b'OK STOPPED'
        port.write(f'ARM {args.frequency} {args.gain}\n'.encode())
        reply=port.readline().strip()
        if reply!=b'OK ARMED':raise ValueError(reply)
        try:
            for attempt in range(args.count):
                port.write(b'TRACE 150\n')
                header=port.readline().split()
                if len(header)!=5 or header[0]!=b'TRACE':raise ValueError(header)
                n,elapsed,failed=map(int,header[1:4]);crc=int(header[4],16)
                if not 1<=n<=3276 or not 100000<=elapsed<=160000:raise ValueError(header)
                body=bytearray()
                while len(body)<n*20:
                    chunk=port.read(n*20-len(body))
                    if not chunk:raise ValueError('Short USB trace')
                    body.extend(chunk)
                if zlib.crc32(body)!=crc:
                    (out/f'usb-crc-failed-{attempt:03d}.bin').write_bytes(body)
                    print('Rejected USB CRC',flush=True);continue
                rows=np.frombuffer(body,dtype=DTYPE).copy()
                if not np.all(np.diff(rows['us'].astype(np.int64))>0):raise ValueError('Non-monotonic trace')
                if not all(np.all(np.isfinite(rows[name])) for name in ['f16','f32','coherence','power']):raise ValueError('Non-finite RF estimates')
                meta=dict(attempt=attempt,identity=identity,tuning_mhz=args.frequency,gain_index=args.gain,
                          received_utc=datetime.now(timezone.utc).isoformat(),elapsed_us=elapsed,failed_snapshots=failed,
                          count=n,median_spacing_us=float(np.median(np.diff(rows['us']))),
                          source='ESP32 hardware frequency estimates, discontinuous IQ snapshots')
                np.savez_compressed(out/f'{attempt:03d}.npz',rows=rows,metadata=json.dumps(meta))
                results.append(meta)
                print(f'Trace {attempt}: {n} observations, spacing {meta["median_spacing_us"]:.0f} us, coherent {np.mean(rows["coherence"]>.2):.1%}',flush=True)
                (out/'summary.json').write_text(json.dumps(results,indent=2))
        finally:
            port.write(b'STOP\n')

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baud',type=int,default=115200);p.add_argument('--port',default='COM10')
    p.add_argument('--frequency',type=int,default=2744,choices=range(2742,2748))
    p.add_argument('--gain',type=int,default=65,choices=range(82))
    p.add_argument('--count',type=int,default=10)
    p.add_argument('--output',required=True)
    collect(p.parse_args())
