"""Monitor the autonomous ESP32 receiver. Host performs no RF demodulation.

By default this only listens and leaves reception running when it exits.
"""
import argparse
import json
from pathlib import Path
import time
import serial
from onboard_receive import ReportStream


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',default='COM10')
    p.add_argument('--seconds',type=float,default=60)
    p.add_argument('--output',default='harmonic-captures/continuous-live.json')
    p.add_argument('--start',action='store_true',help='Stop, configure the fast profile, and restart reception')
    p.add_argument('--phases',type=int,default=1,choices=[1,2,4,8,16,32])
    p.add_argument('--fec-choices',type=int,default=0,choices=[0,8,16,32,128],help='0 selects fast FEC with exhaustive fallback')
    args=p.parse_args()
    if args.seconds<=0:p.error('seconds must be positive')
    port=serial.Serial(port=None,baudrate=115200,timeout=.5,write_timeout=4)
    port.dtr=False;port.rts=False;port.port=args.port
    output=Path(args.output);output.parent.mkdir(parents=True,exist_ok=True)
    records=[];began=time.monotonic();stream=ReportStream();invalid=0
    # Keep the append log open. Reopening/truncating a OneDrive file for every
    # report can fail while the sync client is replacing its placeholder.
    with port,output.with_suffix('.jsonl').open('w',encoding='utf-8') as journal:
        port.set_buffer_size(rx_size=262144)
        if args.start:
            commands=[('STOP',b'OK STOPPED'),(f'SEARCH {args.phases}',b'OK SEARCH'),('DSP 1',b'OK DSP'),
                      ('FFTDEC 1',b'OK FFTDEC'),(f'FECCHOICES {args.fec_choices}',b'OK FECCHOICES'),
                      ('REFINE 1',b'OK REFINE'),('PAYLOADDEC 0',b'OK PAYLOADDEC'),('TRACK 1',b'OK TRACK'),
                      ('CONT 2744 65',b'OK CONTINUOUS')]
            for command,ack in commands:
                port.write((command+'\n').encode());deadline=time.monotonic()+8
                while time.monotonic()<deadline:
                    line=port.readline()
                    if ack in line:break
                    if b'ERR ' in line:raise ValueError(line.decode(errors='replace'))
                else:raise TimeoutError(f'No acknowledgement for {command}')
            began=time.monotonic()
        deadline=began+args.seconds
        while time.monotonic()<deadline:
            try:line=stream.read(port,deadline)
            except TimeoutError:break
            records.append(dict(elapsed_s=round(time.monotonic()-began,6),line=line,
                report_crc32_verified=True,transport=dict(stream.stats)))
            journal.write(json.dumps(records[-1])+'\n');journal.flush()
            if line.startswith('INVALID_CAPTURE'):
                invalid+=1
                if invalid<=3:print(line,flush=True)
            else:print(line,flush=True)
            if line.startswith('STREAM_ERROR'):raise RuntimeError(line)
    for attempt in range(10):
        try:output.write_text(json.dumps(records,indent=2)+'\n',encoding='utf-8');break
        except OSError:
            if attempt==9:raise
            time.sleep(.1)
    print(f'Saved {len(records)} CRC-verified reports; invalid captures={invalid}. Receiver left running.',flush=True)
    if not records:raise RuntimeError('No device reports received')


if __name__=='__main__':main()
