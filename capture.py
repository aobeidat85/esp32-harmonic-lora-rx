"""Receive-only RTL-SDR capture. Saves interleaved unsigned 8-bit IQ."""
import argparse
import ctypes as c
import json
import os
from pathlib import Path
from datetime import datetime, timezone

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--seconds', type=float, default=10)
p.add_argument('--frequency', type=int, default=915000000)
p.add_argument('--rate', type=int, default=2000000)
p.add_argument('--gain', type=int, default=200, help='tenths of dB')
p.add_argument('--dll', default=r'C:\ChatGPT\Radio\tools\rtl_433\librtlsdr.dll')
p.add_argument('--output', default='captures/heltec-915.cu8')
a = p.parse_args()
if a.seconds <= 0:
    p.error('seconds must be positive')
dll_path = Path(a.dll).resolve()
dll_dir = os.add_dll_directory(str(dll_path.parent))
d = c.CDLL(str(dll_path))
d.rtlsdr_open.argtypes = [c.POINTER(c.c_void_p), c.c_uint32]
for name, typ in [('rtlsdr_set_sample_rate', c.c_uint32), ('rtlsdr_set_center_freq', c.c_uint32), ('rtlsdr_set_tuner_gain_mode', c.c_int), ('rtlsdr_set_tuner_gain', c.c_int)]:
    getattr(d, name).argtypes = [c.c_void_p, typ]
d.rtlsdr_read_sync.argtypes = [c.c_void_p, c.c_void_p, c.c_int, c.POINTER(c.c_int)]
d.rtlsdr_reset_buffer.argtypes = [c.c_void_p]
d.rtlsdr_close.argtypes = [c.c_void_p]
def check(result):
    if result < 0:
        raise RuntimeError(f'RTL-SDR error {result}')
h = c.c_void_p()
check(d.rtlsdr_open(c.byref(h), 0))
out = Path(a.output)
out.parent.mkdir(parents=True, exist_ok=True)
total = 0
try:
    check(d.rtlsdr_set_sample_rate(h, a.rate))
    check(d.rtlsdr_set_center_freq(h, a.frequency))
    check(d.rtlsdr_set_tuner_gain_mode(h, 1))
    check(d.rtlsdr_set_tuner_gain(h, a.gain))
    check(d.rtlsdr_reset_buffer(h))
    buf = c.create_string_buffer(262144)
    n = c.c_int()
    # Discard startup transients.
    for _ in range(4):
        check(d.rtlsdr_read_sync(h, buf, len(buf), c.byref(n)))
    started = datetime.now(timezone.utc).isoformat()
    with out.open('wb') as f:
        while total < a.seconds * a.rate * 2:
            check(d.rtlsdr_read_sync(h, buf, len(buf), c.byref(n)))
            if n.value <= 0:
                raise RuntimeError('No samples returned')
            f.write(buf.raw[:n.value])
            total += n.value
finally:
    d.rtlsdr_close(h)
meta = dict(format='cu8', frequency_hz=a.frequency, sample_rate_hz=a.rate,
            gain_db=a.gain/10, bytes=total, seconds=total/(2*a.rate), started_utc=started,
            note='Host receive stream; sample continuity is not independently verified')
out.with_suffix('.json').write_text(json.dumps(meta, indent=2))
print(json.dumps(meta, indent=2))
