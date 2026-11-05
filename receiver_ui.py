"""Local, read-only dashboard for CRC-verified ESP32 receive reports."""
import argparse
from collections import deque
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import threading
import time
import webbrowser

import serial
from onboard_receive import ReportStream

ROOT = Path(__file__).resolve().parent


def parse_report(line):
    kind, _, rest = line.strip().partition(' ')
    fields = dict(re.findall(r'(\w+)=([^ ]+)', rest))
    result = dict(kind=kind, fields=fields)
    if kind == 'LORA_CRC_OK':
        payload = bytes.fromhex(fields.get('payload', ''))
        result.update(payload_hex=payload.hex(), byte_count=len(payload),
                      text=''.join(chr(b) if 32 <= b < 127 else '.' for b in payload))
        if len(payload) >= 12 and payload[:2] == b'RT':
            result['bridge'] = dict(session=int.from_bytes(payload[2:6], 'little'),
                frame=int.from_bytes(payload[6:8], 'little'), fragment=payload[8],
                fragments=payload[9], frame_bytes=int.from_bytes(payload[10:12], 'little'))
    return result


class Receiver:
    def __init__(self, port, log_dir):
        self.port = port
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.events = deque(maxlen=300)
        self.status = {}
        self.connected = False
        self.error = None
        self.last_report = None
        self.counts = dict(packets=0, frames=0, reports=0, invalid_captures=0)
        self.transport = {}
        self.sequence = 0
        log_dir.mkdir(parents=True, exist_ok=True)
        self.log_path = log_dir / (datetime.now().strftime('dashboard-%Y%m%d-%H%M%S') + '.jsonl')

    def snapshot(self):
        with self.lock:
            return dict(port=self.port, connected=self.connected, error=self.error,
                last_report=self.last_report, counts=dict(self.counts), status=dict(self.status),
                transport=dict(self.transport), events=list(self.events), log=str(self.log_path))

    def accept(self, line, transport):
        parsed = parse_report(line)
        now = datetime.now(timezone.utc).isoformat()
        with self.lock:
            self.sequence += 1
            event = dict(id=self.sequence, time=now, line=line.strip(),
                         report_crc32_verified=True, **parsed)
            self.counts['reports'] += 1
            self.last_report = now
            self.transport = dict(transport)
            if parsed['kind'] == 'STREAM_STATUS':
                self.status = parsed['fields']
            else:
                self.events.appendleft(event)
            if parsed['kind'] == 'LORA_CRC_OK': self.counts['packets'] += 1
            if parsed['kind'] == 'RTCM_CRC_OK': self.counts['frames'] += 1
            if parsed['kind'] == 'INVALID_CAPTURE': self.counts['invalid_captures'] += 1
        return event

    def run(self):
        # Never assert DTR/RTS, send STOP, or change the receiver's RF settings.
        with self.log_path.open('a', encoding='utf-8') as journal:
            while not self.stop.is_set():
                try:
                    port = serial.Serial(port=None, baudrate=115200, timeout=.25)
                    port.dtr = False
                    port.rts = False
                    port.port = self.port
                    with port:
                        if hasattr(port, 'set_buffer_size'): port.set_buffer_size(rx_size=262144)
                        with self.lock:
                            self.connected = True
                            self.error = None
                        stream = ReportStream()
                        while not self.stop.is_set():
                            try:
                                line = stream.read(port, time.monotonic() + 1)
                            except TimeoutError:
                                continue
                            event = self.accept(line, stream.stats)
                            journal.write(json.dumps(event) + '\n')
                            journal.flush()
                except (serial.SerialException, OSError, ValueError) as exc:
                    with self.lock:
                        self.connected = False
                        self.error = str(exc)
                    self.stop.wait(2)
                finally:
                    with self.lock: self.connected = False


def make_handler(receiver):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            route = self.path.split('?', 1)[0]
            if route == '/api/state':
                body = json.dumps(receiver.snapshot()).encode()
                mime = 'application/json'
            elif route in ('/', '/index.html'):
                body = (ROOT / 'receiver_ui.html').read_bytes()
                mime = 'text/html; charset=utf-8'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args): pass
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='COM10', help='ESP32 receiver serial port')
    parser.add_argument('--http-port', default=9150, type=int)
    parser.add_argument('--no-browser', action='store_true')
    args = parser.parse_args()
    receiver = Receiver(args.port, ROOT / 'dashboard-logs')
    server = ThreadingHTTPServer(('127.0.0.1', args.http_port), make_handler(receiver))
    worker = threading.Thread(target=receiver.run, daemon=True)
    worker.start()
    url = f'http://127.0.0.1:{args.http_port}'
    print(f'ESP32 receive dashboard: {url}\nSerial: {args.port}; log: {receiver.log_path}', flush=True)
    if not args.no_browser: webbrowser.open(url)
    try:
        server.serve_forever(poll_interval=.25)
    except KeyboardInterrupt:
        pass
    finally:
        receiver.stop.set()
        server.server_close()
        worker.join(timeout=3)


if __name__ == '__main__': main()
