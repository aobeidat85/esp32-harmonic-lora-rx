"""Decode the Heltec 915 MHz SF7/BW500 bridge and validate reassembled RTCM3."""
import argparse
from collections import Counter
import json
from pathlib import Path
import struct
import sys
import numpy as np

sys.path.insert(0, str(Path(__file__).parent / 'vendor'))
from lora_decode import decode, DecodeError

def crc24q(data):
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc = (crc << 1) ^ (0x1864CFB if crc & 0x800000 else 0)
        crc &= 0xffffff
    return crc

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', nargs='?', default='captures/heltec-915.cu8')
    args = parser.parse_args()
    path = Path(args.capture)
    meta = json.loads(path.with_suffix('.json').read_text())
    fs = meta['sample_rate_hz']
    raw = np.fromfile(path, dtype=np.uint8).astype(np.float32) - 127.5
    iq = raw[::2] + 1j * raw[1::2]
    # Strong local transmitter: isolate power bursts, retaining quiet samples on
    # either side. This is not a general weak-signal or continuous-traffic scanner.
    block = 256
    power = np.mean(abs(iq[:len(iq)//block*block].reshape(-1, block))**2, axis=1)
    active = power > np.percentile(power, 10) * 4
    starts = np.flatnonzero(np.diff(active.astype(int), prepend=0) == 1)
    ends = np.flatnonzero(np.diff(active.astype(int), append=0) == -1) + 1
    counts = Counter()
    messages = Counter()
    pending = {}
    packets = []
    frames = bytearray()
    for start, end in zip(starts, ends):
        if (end-start)*block/fs < .005:
            continue
        counts['bursts_examined'] += 1
        first = max(0, start*block - 1024)
        last = min(len(iq), end*block + 1024)
        try:
            result = decode(iq[first:last], fs, sf=7, bw=500000)
        except DecodeError:
            counts['phy_decode_failed'] += 1
            continue
        if not result.get('crc_ok'):
            counts['phy_decode_failed'] += 1
            continue
        counts['lora_crc_valid'] += 1
        payload = result['payload']
        record = dict(time_s=first/fs+result['packet_start_s'], payload_hex=payload.hex(), lora_crc_ok=True)
        packets.append(record)
        if len(payload) < 12 or payload[:2] != b'RT':
            continue
        session, frame_id, index, count, length = struct.unpack('<IHBBH', payload[2:12])
        record.update(session=session, frame_id=frame_id, fragment=index, fragment_count=count, frame_length=length)
        if not 6 <= length <= 1029 or count != (length+219)//220 or index >= count:
            counts['invalid_bridge_header'] += 1
            continue
        expected_size = min(220, length-index*220)
        if len(payload)-12 != expected_size:
            counts['invalid_bridge_header'] += 1
            continue
        counts['bridge_fragments'] += 1
        key = session, frame_id
        entry = pending.setdefault(key, {'length': length, 'count': count, 'parts': {}})
        if entry['length'] != length or entry['count'] != count:
            counts['inconsistent_fragments'] += 1
            continue
        entry['parts'][index] = payload[12:]
        if len(entry['parts']) != count:
            continue
        frame = b''.join(entry['parts'][i] for i in range(count))
        del pending[key]
        if (len(frame) == length and frame[0] == 0xd3 and not frame[1] & 0xfc
                and ((frame[1]&3)<<8 | frame[2])+6 == length and crc24q(frame) == 0):
            message = frame[3]<<4 | frame[4]>>4
            messages[message] += 1
            counts['rtcm_crc_valid'] += 1
            frames.extend(frame)
            print(f'RTCM {message}: {length} bytes, frame {frame_id}, CRC-24Q OK', flush=True)
        else:
            counts['rtcm_validation_failed'] += 1
    counts['incomplete_frames'] = len(pending)
    path.with_suffix('.packets.jsonl').write_text(''.join(json.dumps(p)+'\n' for p in packets))
    path.with_suffix('.rtcm3').write_bytes(frames)
    summary = dict(source=str(path.resolve()), settings=dict(frequency_hz=meta['frequency_hz'], bandwidth_hz=500000, sf=7, coding_rate='4/5', sync_word='0x12'), counts=dict(counts), rtcm_message_counts=dict(messages), limitations='Power-triggered local-transmitter capture; incomplete edge packets and sample gaps can affect counts. Counts do not establish packet loss rate or GNSS RTK fix.')
    path.with_suffix('.decoded.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))

if __name__ == '__main__':
    main()
