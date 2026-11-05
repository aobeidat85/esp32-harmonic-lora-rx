# Heltec RTCM reception at 915 MHz

Latest: [autonomous reception on the ESP32](CONTINUOUS.md). HRXLORA36 starts
receiving at boot, with capture and decoding on separate ESP32 cores. The live
autonomous pipeline has recovered a CRC-valid LoRa packet and RTCM1134 frame.
No receiving hardware was added. Evidence and limits are in `onboard-summary.json`.
The Nooelec setup below is the earlier independent reference baseline.

## Live dashboard

Run `./Receive-Dashboard.ps1` in PowerShell, then open
[the local receive dashboard](http://127.0.0.1:9150). It shows live LoRa payload
hex, bridge fragment details, validated RTCM message types/frame IDs, counters,
capture coverage and receiver health. Pause freezes only the display; acquisition
continues. JSON export saves the currently displayed events.

The dashboard reads COM10 at 115200 baud without changing receiver settings or
sending STOP. Close other programs using COM10 first. Optional parameters:
`./Receive-Dashboard.ps1 -Port COM10 -HttpPort 9150`. No extra packages beyond
the existing project requirements are needed. Keep the Python server running
while viewing the page; Ctrl+C stops the server and leaves the ESP32 receiving.
The page is served only on this computer's loopback interface.

Every CRC32-verified device report is appended under `dashboard-logs/`. The page
retains the newest 300 non-status events; packet/frame cards count arrivals since
this dashboard server started, while device totals appear in Receiver health.
No historical recordings or demo packets are injected into the live feed.
If USB disconnects, the dashboard shows an error and retries every two seconds.

Receive-only baseline for the ESP32 harmonic-radio experiments. The Nooelec
NESDR SMArt v5 receives the existing Heltec RTCM transmitter over the air.
This SDR baseline runs on the PC. The separate [ESP32-only harmonic experiment](EXPERIMENT.md)
uses experimental receive firmware on the original ESP32, on COM10.

Transmitter configuration: a Heltec RTCM UDP bridge, receiving MobileCM RTCM on UDP port 2202.

| Parameter | Value |
| --- | --- |
| Frequency | 915.000 MHz |
| LoRa bandwidth | 500 kHz |
| Spreading factor | 7 |
| Coding rate | 4/5 |
| Sync word | 0x12 |
| Header | Explicit |
| Payload CRC | Enabled |
| Capture rate | 2,000,000 complex samples/s |
| SDR gain | 20 dB |

MobileCM broadcasts RTCM to UDP port 2202. The Heltec validates each RTCM frame
and transmits it as one or more LoRa packets. Each packet contains a 12-byte
`RT` bridge header followed by up to 220 bytes of RTCM. The decoder validates
the LoRa CRC, reconstructs fragments by session and frame ID, and checks the
complete RTCM frame using CRC-24Q. Accepted frames are saved as binary RTCM3.

## Run

Close other software using the Nooelec and keep the existing transmitter running.
From PowerShell:

```powershell
./Receive-Heltec.ps1 -Seconds 10
```

This captures first, then decodes offline. It is not a live continuous receiver.
Each run saves timestamped files under `captures/`:

- `.cu8`: original unsigned 8-bit interleaved IQ
- `.json`: capture frequency, sample rate, gain, and UTC start time
- `.packets.jsonl`: LoRa CRC-valid payloads and bridge fragment metadata
- `.rtcm3`: reassembled, CRC-24Q-valid RTCM frames
- `.decoded.json`: packet/frame counters and RTCM message types

To repeat decoding of the first verified recording:

```powershell
# Run from the repository root
& .venv/Scripts/python.exe decode_capture.py captures/heltec-915.cu8
```

Create a virtual environment with Python 3.14:
`python -m venv .venv`, then `.venv/Scripts/python.exe -m pip install -r requirements.txt`.
The capture script uses the existing RTL-SDR library at
`C:/ChatGPT/Radio/tools/rtl_433/librtlsdr.dll`; pass `--dll` to `capture.py` to
use another compatible library and its dependent DLLs.

## Verified 2026-11-05

Windows detected the NESDR SMArt v5 and the driver successfully opened it.
Passive COM17 monitoring showed RTCM and LoRa transmit counters increasing with
zero reported errors. An 8.060928-second SDR capture at 915 MHz produced
CRC-valid LoRa packets with the expected `RT` header, and reconstructed RTCM
messages passed CRC-24Q, including types 1005, 1074, 1084, 1094, 1104, 1114, and 1134.
See `captures/heltec-915.decoded.json` for exact measured counts.

This power-triggered decoder is designed for the strong local test transmitter.
It can miss weak, overlapping, or uninterrupted traffic. Edge fragments may be
incomplete. USB sample continuity is not independently checked. Decoder counts
do not measure the link's packet loss rate and do not establish rover RTK fix.
The vendor decoder's CFO and SNR estimates are not used as calibrated RF measurements.

## Decoder source and license

`vendor/lora_decode.py` and `vendor/meshcore_decode.py` are unmodified copies
from [Alex Beal's lr2021-re](https://github.com/beala/lr2021-re), main branch
at retrieval on 2026-11-05, commit `0e71835dbc08315523dc209915beeb303d717194`.
MIT license retained in `vendor/LICENSE`. The PHY implementation follows
EPFL gr-lora_sdr. The MeshCore parser is an upstream import dependency; this
project interprets payloads as the Heltec RTCM bridge protocol.
