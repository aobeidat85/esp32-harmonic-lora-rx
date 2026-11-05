"""Operate the ESP32 decoder; this host only sends commands and saves reports."""
import argparse
import json
import re
from pathlib import Path
import time
import zlib
import numpy as np


class ReportStream:
    """Resynchronize a byte stream; accept only length-bounded CRC-valid reports."""
    def __init__(self):
        self.buffer=bytearray()
        self.stats=dict(unframed_bytes_skipped=0,crc_rejected_reports=0,malformed_headers=0)
    def read(self,port,deadline):
        while time.monotonic()<deadline:
            while True:
                marker=self.buffer.find(b'REPORT ')
                if marker<0:
                    if b'Guru Meditation' in self.buffer:
                        panic=bytes(self.buffer)+port.read(4096)
                        Path('harmonic-captures').mkdir(exist_ok=True)
                        Path('harmonic-captures/device-panic.bin').write_bytes(panic)
                        raise ValueError('ESP32 panic: '+panic.decode(errors='replace'))
                    if self.buffer.startswith(b'ERR ') and b'\n' in self.buffer:raise ValueError(self.buffer.split(b'\n')[0].decode(errors='replace'))
                    discard=max(0,len(self.buffer)-256)
                    self.stats['unframed_bytes_skipped']+=discard
                    if discard:del self.buffer[:discard]
                    break
                if marker:
                    self.stats['unframed_bytes_skipped']+=marker
                    del self.buffer[:marker]
                end=self.buffer.find(b'\n')
                if end<0:
                    if len(self.buffer)>80:
                        del self.buffer[0];self.stats['malformed_headers']+=1;continue
                    break
                match=re.fullmatch(rb'REPORT (\d{1,4}) ([0-9a-fA-F]{8})\r?\n',bytes(self.buffer[:end+1]))
                if not match or not 1<=int(match[1])<=1024:
                    del self.buffer[0];self.stats['malformed_headers']+=1;continue
                size=int(match[1]);total=end+1+size
                if len(self.buffer)<total:break
                body=bytes(self.buffer[end+1:total])
                if zlib.crc32(body)!=int(match[2],16):
                    del self.buffer[0];self.stats['crc_rejected_reports']+=1;continue
                del self.buffer[:total]
                return body.decode('ascii')
            chunk=port.read(max(1,min(getattr(port,'in_waiting',0),4096)))
            if chunk:self.buffer.extend(chunk)
        raise TimeoutError('No CRC-valid ESP32 report before deadline')

def main(args):
    import serial
    port=serial.Serial(port=None,baudrate=args.baud,timeout=.5,write_timeout=4)
    port.dtr=False;port.rts=False;port.port=args.port
    report=[]
    with port:
        if hasattr(port,'set_buffer_size'):port.set_buffer_size(rx_size=262144)
        port.reset_input_buffer()
        identity='';ready_until=time.monotonic()+45;next_query=0
        while time.monotonic()<ready_until:
            if time.monotonic()>=next_query:
                port.write(b'INFO\n');next_query=time.monotonic()+1.5
            response=port.readline()
            match=re.search(rb'HRXLORA(\d+) ESP32-D0WD-V3 \d+ \d+ 16000000 16380',response)
            if match and int(match[1])>=7:
                identity=match[0].decode();break
        if not identity:raise ValueError('No supported ESP32 identity; prior processing may still be running')
        time.sleep(.2);port.reset_input_buffer()
        port.write(b'STOP\n')
        stop_until=time.monotonic()+3
        while time.monotonic()<stop_until:
            if port.readline().rstrip().endswith(b'OK STOPPED'):break
        else:raise ValueError('Receiver did not acknowledge STOP')
        if int(identity.split()[0][7:])>=23:
            for command,ack in [(f'SEARCH {args.phases}',b'OK SEARCH'),(f'DSP {0 if args.legacy_dsp else 1}',b'OK DSP')]:
                port.write((command+'\n').encode())
                if port.readline().strip()!=ack:raise ValueError('DSP configuration not acknowledged')
        if int(identity.split()[0][7:])>=24:
            for command,ack in [(f'FFTDEC {args.decimation}',b'OK FFTDEC'),(f'FECCHOICES {args.fec_choices}',b'OK FECCHOICES'),(f'REFINE {int(args.fast_refine)}',b'OK REFINE')]:
                port.write((command+'\n').encode())
                if port.readline().strip()!=ack:raise ValueError('Fast-mode configuration not acknowledged')
        if int(identity.split()[0][7:])>=20:
            port.write(f'TRACK {0 if args.no_tracking else 1}\n'.encode())
            if port.readline().strip()!=b'OK TRACK':raise ValueError('Tracking configuration not acknowledged')
        elif args.no_tracking or args.save_captures:
            raise ValueError('Capture export and tracking selection require HRXLORA20 or later')
        if not args.replay:
            port.write(f'ARM {args.frequency} {args.gain}\n'.encode());reply=port.readline().strip()
            if reply!=b'OK ARMED':raise ValueError(reply)
        try:
            for attempt in range(args.count):
                if args.replay:
                    with np.load(args.replay,allow_pickle=False) as d:
                        blocks=d['blocks']
                        metadata=json.loads(str(d['metadata'])) if 'metadata' in d else {}
                    filtered=args.filtered or metadata.get('filtered',False)
                    body=blocks.tobytes()
                    port.write(f"{'REPLAYFILTER' if filtered else 'REPLAYPACK'} {len(blocks)} {zlib.crc32(body):08x}\n".encode())
                    if port.readline().strip()!=b'READY':raise ValueError('Replay not ready')
                    for i in range(0,len(body),128):port.write(body[i:i+128]);time.sleep(.001)
                else:port.write(f"{'DFILTER' if args.filtered else 'DECODE'} {args.window_ms}\n".encode())
                end=time.monotonic()+240
                retry_at=time.monotonic()+45 if int(identity.split()[0][7:])>=22 else end
                retried=False
                stream=ReportStream()
                while time.monotonic()<end:
                    try:line=stream.read(port,min(end,retry_at))
                    except TimeoutError:
                        if retried or retry_at>=end:raise
                        port.write(b'LASTREPORT\n');retried=True;retry_at=end
                        print('Requesting CRC-protected final report again',flush=True)
                        continue
                    print(line,flush=True)
                    report.append(dict(attempt=attempt,firmware_identity=identity,carrier_tracking=not args.no_tracking,final_report_retry=retried,line=line,transport=dict(stream.stats),report_crc32_verified=True))
                    if args.output:Path(args.output).write_text(json.dumps(report,indent=2))
                    if line.startswith('ONBOARD '):break
                else:raise TimeoutError('ESP32 processing did not return a report')
                if retried:
                    # Drain queued summary replies before the next capture/export.
                    # INFO is a FIFO barrier; it is handled after LASTREPORT.
                    barrier_end=time.monotonic()+6;next_query=0
                    while time.monotonic()<barrier_end:
                        if time.monotonic()>=next_query:
                            port.write(b'INFO\n');next_query=time.monotonic()+1.5
                        if re.search(rb'HRXLORA\d+ ESP32-D0WD-V3 \d+ \d+ 16000000 16380',port.readline()):break
                    else:raise TimeoutError('No completion barrier after report retry')
                if args.save_captures and not args.replay:
                    port.write(b'DUMPPACK\n');header=port.readline()
                    match=re.search(rb'PACK2 (\d+) ([01]) ([0-9a-fA-F]{8}) ([0-9a-fA-F]{8})\r?\n$',header)
                    if not match or not 1<=int(match[1])<=800:raise ValueError('Invalid capture export header')
                    prefix=match[0].rstrip(b'\r\n').rsplit(b' ',1)[0]
                    if zlib.crc32(prefix)!=int(match[4],16):raise ValueError('Capture export header CRC failed')
                    size=int(match[1])*136;body=bytearray();deadline=time.monotonic()+30
                    while len(body)<size and time.monotonic()<deadline:
                        body.extend(port.read(min(4096,size-len(body))))
                    if len(body)!=size or zlib.crc32(body)!=int(match[3],16):raise ValueError('Capture export data CRC failed; capture discarded')
                    from packed_capture import DTYPE
                    destination=Path(args.save_captures);destination.mkdir(parents=True,exist_ok=True)
                    meta=dict(source='ESP32 hardware capture; timed gaps preserved; not a replay',
                              filtered=bool(int(match[2])),firmware_identity=identity,frequency_mhz=args.frequency,gain=args.gain)
                    np.savez_compressed(destination/f'{attempt:03d}.npz',blocks=np.frombuffer(body,dtype=DTYPE),metadata=json.dumps(meta))
                    print(f'Saved CRC-verified capture {destination / f"{attempt:03d}.npz"}',flush=True)
                if args.stop_on_packet and re.search(r' packets=[1-9]\d*',line):break
        finally:port.write(b'STOP\n')

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',default='COM10');p.add_argument('--count',type=int,default=1)
    p.add_argument('--baud',type=int,default=115200)
    p.add_argument('--filtered',action='store_true')
    p.add_argument('--phases',type=int,default=32,choices=[1,2,4,8,16,32])
    p.add_argument('--legacy-dsp',action='store_true',help='Diagnostic: compare original FFT reconstruction')
    p.add_argument('--decimation',type=int,default=1,choices=[1,4])
    p.add_argument('--fec-choices',type=int,default=128,choices=[0,8,16,32,128],help='0 tries fast FEC then exhaustive FEC')
    p.add_argument('--fast-refine',action='store_true')
    p.add_argument('--no-tracking',action='store_true',help='Diagnostic: hold carrier estimate fixed during header and payload')
    p.add_argument('--save-captures',help='Diagnostic: save the exact live packed capture after on-device processing')
    p.add_argument('--stop-on-packet',action='store_true',help='Stop after a window reports a CRC-valid LoRa packet')
    p.add_argument('--window-ms',type=int,default=220,choices=[150,220])
    p.add_argument('--frequency',type=int,default=2744,choices=range(2742,2748));p.add_argument('--gain',type=int,default=65,choices=range(82))
    p.add_argument('--replay');p.add_argument('--output');main(p.parse_args())
