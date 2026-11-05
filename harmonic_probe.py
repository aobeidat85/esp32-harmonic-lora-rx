"""Original-ESP32 third-harmonic LoRa acquisition experiment, not packet decoding."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import time
import time
import zlib
import numpy as np

FS = 16000000
COUNT = 16380

def iq_from_words(words):
    i = ((words >> 10) & 1023).astype(np.int32)
    q = (words & 1023).astype(np.int32)
    return (np.where(i >= 512, i-1024, i) + 1j*np.where(q >= 512, q-1024, q)).astype(np.complex64)

def chirp_scores(iq):
    # One ms spans approximately four SF7/BW500 symbols. A third harmonic
    # multiplies phase and sweep width by three, preserving the symbol period.
    # Discard the first 16 us as a fixed transient guard, then decimate to 8 MS/s.
    from scipy.signal import resample_poly
    x = resample_poly(iq[256:], 1, 2)
    x -= np.mean(x)
    fs = FS/2
    t = np.arange(len(x))/fs
    period = 128/500000
    window = np.hanning(len(x))
    energy = np.sum(abs(x*window)**2)
    if energy < 1e-10:
        return {'coherent_fraction': 0., 'error': 'zero energy'}
    freq = np.fft.fftfreq(len(x), 1/fs)
    allowed = (abs(freq) > 50000) & (abs(freq) < 3500000)
    best = {'coherent_fraction': 0.}
    for sign in (-1, 1):
        for phase in range(32):
            u = (t + phase*period/32) % period
            reference = np.exp(sign*3j*np.pi*(500000/period*u*u - 500000*u))
            spectrum = abs(np.fft.fft(x*window*np.conj(reference)))**2
            spectrum[~allowed] = 0
            k = int(np.argmax(spectrum))
            fraction = float(spectrum[k] / (len(x)*energy))
            if fraction > best['coherent_fraction']:
                best = dict(coherent_fraction=fraction, iq_slope_sign=sign,
                            phase_grid=phase, residual_frequency_hz=float(freq[k]))
    return best

def self_check():
    rng = np.random.default_rng(814)
    t = np.arange(COUNT)/FS
    u = (t + 6*128/500000/32) % (128/500000)
    signal = np.exp(3j*np.pi*(500000/(128/500000)*u*u-500000*u)+2j*np.pi*1000000*t)
    noise = (rng.standard_normal(COUNT)+1j*rng.standard_normal(COUNT))*.2
    positive = chirp_scores(signal+noise)
    negative = chirp_scores(noise)
    cw = chirp_scores(np.exp(2j*np.pi*1000000*t)+noise)
    assert positive['coherent_fraction'] > .08, positive
    assert negative['coherent_fraction'] < .08, negative
    assert cw['coherent_fraction'] < .08, cw
    print(json.dumps(dict(synthetic_chirp=positive, noise=negative, cw=cw), indent=2))

def collect(args):
    import serial
    directory = Path(args.output) if args.output else Path('harmonic-captures')/datetime.now().strftime('%Y%m%d-%H%M%S')
    directory.mkdir(parents=True, exist_ok=False)
    s = serial.Serial(port=None, baudrate=args.baud, timeout=4, write_timeout=4)
    s.dtr = False
    s.rts = False
    s.port = args.port
    records = []
    transport_errors = []
    with s:
        if hasattr(s,'set_buffer_size'):s.set_buffer_size(rx_size=262144)
        identity = ''
        for _ in range(3):
            s.reset_input_buffer()
            s.write(b'INFO\n')
            identity = s.readline().decode(errors='replace').strip()
            if re.match(r'HRXLORA\d+ ESP32-D0WD-V3 ',identity):
                break
        if not re.match(r'HRXLORA\d+ ESP32-D0WD-V3 ',identity):
            raise ValueError(f'Unexpected receiver firmware: {identity!r}')
        try:
            for gain in args.gains:
                for mhz in args.frequencies:
                    s.write(b'STOP\n')
                    if s.readline().strip() != b'OK STOPPED':
                        raise ValueError('STOP failed')
                    s.write(f'ARM {mhz} {gain}\n'.encode())
                    reply = s.readline().strip()
                    if reply != b'OK ARMED':
                        raise ValueError(f'ARM failed: {reply!r}')
                    for _ in range(args.count):
                        s.write(f'RAW {COUNT}\n'.encode())
                        # A damaged binary transfer can leave trailing bytes ahead
                        # of the next header. Resync only on a CRC-valid header.
                        deadline = time.monotonic()+12
                        while time.monotonic()<deadline:
                            header = s.readline()
                            match = re.search(rb'RAW2 (\d+) (\d+) ([0-9a-fA-F]{8}) (\d+) ([0-9a-fA-F]{8})\r?\n$', header)
                            if not match or int(match[1]) != COUNT:continue
                            prefix = match[0].rstrip(b'\r\n').rsplit(b' ', 1)[0]
                            if zlib.crc32(prefix) == int(match[5], 16):break
                        else:raise ValueError('No CRC-valid raw header before deadline')
                        payload = bytearray()
                        while len(payload) < COUNT*4:
                            part = s.read(COUNT*4-len(payload))
                            if not part:
                                raise ValueError('Short RF capture')
                            payload.extend(part)
                        if zlib.crc32(payload) != int(match[3], 16):
                            name = f'crc-failed-{len(transport_errors):04d}.bin'
                            (directory/name).write_bytes(payload)
                            transport_errors.append(dict(file=name, error='RF payload CRC mismatch; frame discarded', tuning_mhz=mhz, gain_index=gain))
                            (directory/'transport-errors.json').write_text(json.dumps(transport_errors, indent=2))
                            print('Discarded complete USB frame with bad CRC', flush=True)
                            continue
                        words = np.frombuffer(payload, dtype='<u4').copy()
                        iq = iq_from_words(words)
                        record = dict(index=len(records), tuning_mhz=mhz, gain_index=gain,
                                      trigger_uptime_us=int(match[4]), elapsed_us=int(match[2]),
                                      received_utc=datetime.now(timezone.utc).isoformat(),
                                      rms=float(np.sqrt(np.mean(abs(iq-np.mean(iq))**2))),
                                      clipped_fraction=float(np.mean((abs(iq.real)>=511)|(abs(iq.imag)>=511))),
                                      source='ESP32 hardware capture; RF origin unverified', identity=identity)
                        np.savez_compressed(directory/f'{len(records):04d}.npz', words=words, metadata=json.dumps(record))
                        records.append(record)
                        (directory/'capture-summary.json').write_text(json.dumps(records, indent=2))
                        if len(records)%10 == 0:
                            print(f'Captured {len(records)}: requested {mhz} MHz, gain {gain}, RMS {record["rms"]:.2f}', flush=True)
        finally:
            s.write(b'STOP\n')
    (directory/'capture-summary.json').write_text(json.dumps(records, indent=2))
    print(str(directory.resolve()), flush=True)

def analyze(args):
    rows = []
    for path in sorted(Path(args.directory).glob('*.npz')):
        with np.load(path, allow_pickle=False) as data:
            meta = json.loads(str(data['metadata']))
            score = chirp_scores(iq_from_words(data['words']))
        rows.append(dict(meta, score=score, file=path.name))
    rows.sort(key=lambda r:r['score']['coherent_fraction'], reverse=True)
    result = dict(captures=len(rows), candidate_threshold=.08,
                  candidates=sum(r['score']['coherent_fraction']>=.08 for r in rows),
                  limitation='Exploratory preamble-template search only. No payload or CRC decoded; RF frequency, coupling path and sample clock uncalibrated. Captures have gaps.', rows=rows)
    (Path(args.directory)/'analysis.json').write_text(json.dumps(result, indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='rows'}, indent=2))
    print(json.dumps(rows[:8], indent=2))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    sub.add_parser('self-check')
    cap = sub.add_parser('capture')
    cap.add_argument('--port', default='COM10')
    cap.add_argument('--baud', type=int, default=115200)
    cap.add_argument('--count', type=int, default=30, help='captures per frequency/gain')
    cap.add_argument('--frequencies', type=int, nargs='+', default=[2744,2745,2746], choices=range(2742,2748))
    cap.add_argument('--gains', type=int, nargs='+', default=[45,65])
    cap.add_argument('--output')
    ana = sub.add_parser('analyze')
    ana.add_argument('directory')
    args = parser.parse_args()
    if args.action == 'self-check': self_check()
    elif args.action == 'capture': collect(args)
    else: analyze(args)
