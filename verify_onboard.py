"""Run CRC acceptance/rejection checks on the actual ESP32, using replay data.

These checks exercise the on-device decoder. They do not establish live RF reception.
"""
import argparse
import json
from pathlib import Path
import re
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent
EXPECTED = '525410fd8725500b00011c00d3001646ea91612245a0002000000000000000000000000000576310'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='COM10')
    parser.add_argument('--version', type=int, default=36)
    parser.add_argument('--output-prefix', default='harmonic-captures/qualification-v36')
    parser.add_argument('--fec-choices',type=int,default=0,choices=[0,8,16,32,128])
    parser.add_argument('--conservative',action='store_true',help='Use the original full search and exhaustive FEC')
    args = parser.parse_args()
    prefix = Path(args.output_prefix).resolve()
    prefix.parent.mkdir(parents=True, exist_ok=True)
    cases = [
        ('independent-reference', 'sdr-transformed-harmonic.npz', 1, 1),
        ('payload-crc-negative', 'synthetic-rtcm-payload-crc-negative.npz', 0, 0),
        ('rtcm-crc-negative', 'synthetic-rtcm-crc-negative.npz', 1, 0),
        ('noise', 'synthetic-noise.npz', 0, 0),
    ]
    results = []
    for name, fixture, packets, frames in cases:
        output = prefix.with_name(prefix.name + '-' + name + '.json')
        command=[
            sys.executable, str(ROOT / 'onboard_receive.py'), '--port', args.port,
            '--replay', str(ROOT / 'harmonic-captures' / fixture), '--output', str(output),
        ]
        if not args.conservative:command+=['--phases','1','--fec-choices',str(args.fec_choices),'--fast-refine']
        subprocess.run(command,cwd=ROOT,check=True)
        records = json.loads(output.read_text())
        assert records and all(r['report_crc32_verified'] for r in records), name
        assert all(r['firmware_identity'].startswith(f'HRXLORA{args.version} ') for r in records), name
        lines = [r['line'] for r in records]
        summaries = [line for line in lines if line.startswith('ONBOARD ')]
        assert len(summaries) == 1, name
        summary = dict(re.findall(r'(\w+)=([^ ]+)', summaries[0]))
        assert summary['source'] == 'replay', name
        assert int(summary['packets']) == packets and int(summary['rtcm_frames']) == frames, name
        assert sum(line.startswith('LORA_CRC_OK ') for line in lines) == packets, name
        assert sum(line.startswith('RTCM_CRC_OK ') for line in lines) == frames, name
        if name == 'independent-reference':
            assert any(line.endswith('payload=' + EXPECTED) for line in lines), name
            assert any('type=1134 bytes=28 frame=2896' in line for line in lines), name
        if name == 'payload-crc-negative':
            assert int(summary['crc_failures']) > 0, 'Negative must exercise CRC, not just prefix rejection'
        results.append(dict(case=name, passed=True, report=str(output), summary=summary))
        print(f'CHECK PASS: {name}', flush=True)
    # Verify that a completed result survives a new serial connection and is
    # returned byte-for-byte with framing CRC, without another RF/DSP run.
    import serial
    from onboard_receive import ReportStream
    port=serial.Serial(port=None,baudrate=115200,timeout=.5,write_timeout=4)
    port.dtr=False;port.rts=False;port.port=args.port
    with port:
        stream=ReportStream()
        for attempt in range(3):
            port.write(b'LASTREPORT\n')
            try:cached=stream.read(port,time.monotonic()+2)
            except TimeoutError:continue
            assert cached==summaries[0], 'Cached final report differs from the completed noise result'
            break
        else:raise TimeoutError('Cached final report could not be recovered')
    results.append(dict(case='final-report-retry',passed=True,summary=cached))
    print('CHECK PASS: final-report-retry',flush=True)
    output = prefix.with_name(prefix.name + '-summary.json')
    output.write_text(json.dumps(dict(passed=True, firmware=f'HRXLORA{args.version}',
        limitation='Replay qualification only; not live bare-ESP32 RF evidence', cases=results), indent=2) + '\n')
    print(f'All four on-device replay checks and final-report retry passed: {output}', flush=True)


if __name__ == '__main__':
    main()
