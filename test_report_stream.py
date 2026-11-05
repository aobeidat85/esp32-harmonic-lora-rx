import time
import unittest
import zlib
from onboard_receive import ReportStream

def frame(body):
    return f'REPORT {len(body)} {zlib.crc32(body):08x}\n'.encode()+body

class SerialBytes:
    def __init__(self,data,chunk=3):self.data=bytearray(data);self.chunk=chunk
    @property
    def in_waiting(self):return len(self.data)
    def read(self,size):
        n=min(size,self.chunk,len(self.data));value=bytes(self.data[:n]);del self.data[:n];return value

class ReportTests(unittest.TestCase):
    def test_partial_marker_and_binary_prefix(self):
        stream=ReportStream();port=SerialBytes(b'\xff'*64+b'broken phase text'+frame(b'ONBOARD packets=0'),1)
        self.assertEqual(stream.read(port,time.monotonic()+1),'ONBOARD packets=0')
        self.assertGreaterEqual(stream.stats['unframed_bytes_skipped'],64)
    def test_damaged_body_then_good_report(self):
        bad=bytearray(frame(b'LORA_CRC_OK bogus'));bad[-1]^=1
        stream=ReportStream();port=SerialBytes(bad+frame(b'ONBOARD packets=0'))
        self.assertEqual(stream.read(port,time.monotonic()+1),'ONBOARD packets=0')
        self.assertEqual(stream.stats['crc_rejected_reports'],1)
    def test_invalid_length_does_not_consume_next_frame(self):
        stream=ReportStream();port=SerialBytes(b'REPORT 9999 deadbeef\n'+frame(b'ONBOARD packets=0'))
        self.assertEqual(stream.read(port,time.monotonic()+1),'ONBOARD packets=0')
        self.assertEqual(stream.stats['malformed_headers'],1)
    def test_adjacent_reports_retain_buffer(self):
        stream=ReportStream();port=SerialBytes(frame(b'PROGRESS phase=7')+frame(b'ONBOARD packets=0'),4096)
        self.assertEqual(stream.read(port,time.monotonic()+1),'PROGRESS phase=7')
        self.assertEqual(stream.read(port,time.monotonic()+1),'ONBOARD packets=0')
    def test_split_device_error_is_preserved(self):
        stream=ReportStream();port=SerialBytes(b'ERR memory guard\n',1)
        with self.assertRaisesRegex(ValueError,'memory guard'):
            stream.read(port,time.monotonic()+1)

if __name__=='__main__':unittest.main()
