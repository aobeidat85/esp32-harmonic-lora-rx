"""Preserve ESP32 buffered sign-IQ with individual snapshot cycle timestamps."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import zlib
import numpy as np

DTYPE=np.dtype([('start','<u4'),('duration','<u4'),('iq','u1',(128,))])

def unpack(blocks):
    values=(blocks['iq'][:,:,None] >> np.arange(0,8,2,dtype=np.uint8))&3
    values=values.reshape(len(blocks),512)
    return ((values&1).astype(np.float32)*2-1)+1j*((values>>1).astype(np.float32)*2-1)

def collect(args):
    import serial
    out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    port=serial.Serial(port=None,baudrate=args.baud,timeout=4,write_timeout=3)
    port.dtr=False;port.rts=False;port.port=args.port
    results=[]
    with port:
        if hasattr(port,'set_buffer_size'):port.set_buffer_size(rx_size=262144)
        identity=''
        for _ in range(3):
            port.reset_input_buffer();port.write(b'INFO\n');identity=port.readline().decode(errors='replace').strip()
            if re.match(r'HRXLORA\d+ ESP32-D0WD-V3 ',identity):break
        if not re.match(r'HRXLORA\d+ ESP32-D0WD-V3 ',identity):raise ValueError(identity)
        port.write(b'STOP\n');assert port.readline().strip()==b'OK STOPPED'
        port.write(f'ARM {args.frequency} {args.gain}\n'.encode());reply=port.readline().strip()
        if reply!=b'OK ARMED':raise ValueError(reply)
        try:
            for attempt in range(args.count):
                port.write(b'PACK 150\n');header=port.readline().split()
                if len(header)!=5 or header[0]!=b'PACK':raise ValueError(header)
                n,elapsed,failed=map(int,header[1:4]);crc=int(header[4],16)
                if not 1<=n<=600 or not 100000<=elapsed<=160000:raise ValueError(header)
                body=bytearray()
                while len(body)<n*136:
                    chunk=port.read(n*136-len(body))
                    if not chunk:raise ValueError('Short packed USB recording')
                    body.extend(chunk)
                if zlib.crc32(body)!=crc:
                    (out/f'usb-crc-failed-{attempt:03d}.bin').write_bytes(body)
                    print('Rejected USB CRC',flush=True);continue
                blocks=np.frombuffer(body,dtype=DTYPE).copy()
                starts=blocks['start'].astype(np.int64)/240
                if not np.all(np.diff(starts)>0):raise ValueError('Non-monotonic block timestamps')
                meta=dict(attempt=attempt,identity=identity,tuning_mhz=args.frequency,gain_index=args.gain,
                          received_utc=datetime.now(timezone.utc).isoformat(),elapsed_us=elapsed,failed_snapshots=failed,
                          count=n,median_spacing_us=float(np.median(np.diff(starts))),
                          nominal_capture_coverage=float(n*256/elapsed),
                          source='ESP32 hardware sign-IQ; unfiltered decimation x8; timed gaps preserved')
                np.savez_compressed(out/f'{attempt:03d}.npz',blocks=blocks,metadata=json.dumps(meta))
                results.append(meta);(out/'summary.json').write_text(json.dumps(results,indent=2))
                print(f'Packed {attempt}: {n} blocks, median interval {meta["median_spacing_us"]:.1f} us, coverage {meta["nominal_capture_coverage"]:.1%}',flush=True)
        finally:port.write(b'STOP\n')

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baud',type=int,default=115200);p.add_argument('--port',default='COM10');p.add_argument('--frequency',type=int,default=2744,choices=range(2742,2748))
    p.add_argument('--gain',type=int,default=65,choices=range(82));p.add_argument('--count',type=int,default=12)
    p.add_argument('--output',required=True)
    collect(p.parse_args())
