# Autonomous reception on the ESP32

HRXLORA36 starts reception automatically after boot. The original ESP32 captures
the Heltec's 915 MHz signal using the experimental harmonic path, without an
added receiver module. Capture runs on core1; acquisition, LoRa decoding, FEC,
LoRa CRC and RTCM assembly/CRC24Q run on core0. The host only monitors device
reports and exports their already-decoded bytes.

The autonomous pipeline recovered a fresh 40-byte LoRa packet and a valid
28-byte RTCM1134 frame, ID **47638**, using the retained exhaustive-FEC profile.
That run used HRXLORA34 with exhaustive FEC. HRXLORA36 first attempts fast FEC,
then retries exhaustive FEC when a valid header fails payload decoding. This
retains the successful fast and exhaustive paths; both CRC checks remain required.
Evidence: `harmonic-captures/continuous-v34-fec128.json` and
`harmonic-captures/continuous-v34-success.*`.

## Monitor

```powershell
& .venv/Scripts/python.exe continuous_receive.py --seconds 60 --output harmonic-captures/my-continuous-run.json
```

This listens without changing receiver state. Closing the program leaves the
ESP32 receiving. It saves CRC32-verified reports in JSONL as they arrive and
writes a JSON summary when the observation ends.

To restart with the retained profile:

```powershell
& .venv/Scripts/python.exe continuous_receive.py --start --seconds 60 --output harmonic-captures/my-restarted-run.json
```

Serial commands use 115200 baud. `STOP` stops acquisition, cancels and joins the
decoder, and drains queued reports before acknowledging. `CONT 2744 65` starts
autonomous reception again. `INFO`, `STATUS` and `LASTREPORT` work during reception.
Other configuration/diagnostic commands require STOP first. A reset restores the
boot profile; STOP is not persisted to flash.

## Retained profile and checks

| Parameter | Value |
| --- | --- |
| Transmitter | 915 MHz, SF7, BW500 kHz, CR4/5, sync 0x12, explicit header and CRC |
| ESP32 tuning request | 2744 MHz; experimental harmonic path, not a calibrated PLL measurement |
| Gain index | 65 |
| FFT | Full 512-point transforms |
| Search | One coarse timing phase with measured timing/CFO refinement |
| FEC | 128-state parity trellis; fast candidate search with exhaustive fallback |
| Output acceptance | LoRa payload CRC; complete RTCM frame additionally requires CRC24Q |

`LORA_CRC_OK` includes accepted payload bytes in hex. `RTCM_CRC_OK` identifies
the fully validated frame. Header candidates, preamble scores, and failure counts
are diagnostics. They are never emitted as accepted packets.

The optional `--fec-choices 16` forces the faster pruned search;
`--fec-choices 128` forces exhaustive FEC. The default `0` selects the bounded
fast/exhaustive fallback. Controls and the retained hardware recordings are used
to check that a speed change does not silently discard a previously decoded frame.

Run the on-device positive/negative checks with:

```powershell
& .venv/Scripts/python.exe verify_onboard.py
```

These checks temporarily stop reception and leave it stopped. Restart with the
monitor's `--start` option afterwards. They check an independent transformed SDR
reference, payload-CRC rejection, RTCM-CRC rejection, noise rejection and cached
report recovery. The transformed SDR waveform is a replay control and is separate
from the successful fresh RF recordings.

## Pipeline and limits

The retained hardware recording (`fix-v22-live-success.npz`, 220 ms) recovered
the same exact 40-byte payload and RTCM1134 frame 15954 in **3.343 seconds** on
HRXLORA36, versus **44.805 seconds** on HRXLORA22: about **13.4 times faster**.
This is an on-device replay benchmark; fresh autonomous RF reception is evidenced
separately by frame 47638 above. The monitor does no RF DSP.

The positive reference, payload-CRC negative, RTCM-CRC negative, noise negative
and cached-report recovery checks all passed on HRXLORA36 using the boot profile.
See `harmonic-captures/qualification-v36-summary.json`. An additional test that
removed a block from the real recording still failed packet CRC and correctly
produced no accepted data; larger sampling gaps remain a decoding limitation.

The capture core keeps a bounded ring. The decoder copies an immutable snapshot
when ready; the producer does not wait for decoding. Ring access uses an atomic
lease, and incoming records skipped during that short copy are counted. UART
reports use a separate queue/task. The RF loop's console poll is nonblocking,
so a UART flush cannot stall acquisition behind the peripheral mutex.

The current allocation is 650 packed records: a 325-record capture ring and a
325-record decoder snapshot. Candidate views advance every 45 new records.
Older views are skipped when DSP is busy. Their overlap means `dropped_windows`
is not a packet-loss count.

The measured autonomous capture coverage was about **71%**. The I/Q stream has
snapshot gaps, and the decoder cannot process every available view. This remains
best-effort reception; continuous delivery of all RTCM traffic to a rover has not
been established. No range or packet-loss-rate result is claimed.

`STREAM_STATUS` reports capture blocks, analyzed/skipped windows, ring-copy input
drops, report-queue drops, accepted packets/frames, measured coverage and decode
time. Read-only monitoring is optional; reception runs without the host.

The SRAM aperture reservation, linker overlap assertions, chip/core/clock checks,
DMA status/sample-count checks and sentinel remain enabled. RF acquisition uses
short 10 ms chunks. The Heltec firmware is unchanged.

For the initial successful capture and historical receiver details, see
[ONBOARD.md](ONBOARD.md). Current measurements and the installed image hash are
recorded in `onboard-summary.json`.

## Final installed-build observation

HRXLORA36 remained enabled through the final 90-second observation and a later
120-second passive observation. The last status showed 339 seconds of uptime,
944,551 captured blocks, 467 analyzed views, 71.33% coverage and zero report-queue
drops. Those HRXLORA36 observations recovered **no new valid packet**. The earlier
HRXLORA34 fresh-frame result above is separate evidence; the replay speedup does
not establish improved live packet yield. See `continuous-v36-final.json` and
`continuous-v36-soak.json` under `harmonic-captures/`. The monitor closed without
stopping reception.
