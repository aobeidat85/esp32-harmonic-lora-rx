# First verified on-device receiver (HRXLORA22)

For the current auto-start receiver, faster decoding, and simultaneous capture
and DSP, see [CONTINUOUS.md](CONTINUOUS.md). This page records the first successful
live build and its capture/replay diagnostics. `onboard-summary.json` tracks the
current installed firmware and measurements.

The first verified live firmware was **HRXLORA22**. It runs RF capture, chirp acquisition,
symbol recovery, soft FEC, dewhitening, LoRa CRC and RTCM assembly/CRC on the
original ESP32. The host sends commands and logs CRC-protected reports.

**Live bare-ESP32 reception succeeded:** one 40-byte LoRa packet passed its CRC,
and its RTCM1134 frame (28 bytes, frame ID 15954) passed CRC24Q on the ESP32.
The successful run is `harmonic-captures/fix-v22-live-g65.json`. The exact hardware
recording, decoded payload, RTCM3 frame and artifact hashes are saved as
`harmonic-captures/fix-v22-live-success.*`. The host independently rechecked the
exported RTCM checksum; it did not demodulate the RF samples.

This is a first successful window, not a measured reception rate. Its 220 ms
recording took 44.8 seconds to decode. Continuous RTCM delivery remains unverified.
Filtered capture now covers about 74% of a full 220 ms recording. Unfiltered
capture covers about 90%, filling this board's 650-block buffer in about 185 ms.
Neither path is continuous. Live header candidates and an RT prefix alone do not
establish a received packet.

The decoder now checks both carrier-offset classes, tracks residual carrier
error, combines six preamble spectra for weak acquisition, and avoids reporting
the same decoded RF packet twice. CR4/5 payload decoding uses a 128-state parity
trellis with whole-symbol spectral scores. No checksum bits are fitted.

All four on-device replay checks passed on HRXLORA22: exact reference payload
and RTCM1134 recovery, bad LoRa CRC rejection, bad RTCM CRC rejection after a
valid LoRa packet, and no accepted packet from noise. The reference waveform is
a transformed Nooelec recording with a synthetic third harmonic. It is **not
evidence of bare-ESP32 RF reception**. Qualification reports are under
`harmonic-captures/qualification-v22-*`. Retrieving a cached final report through
a new serial connection also passed. LASTREPORT preserves a CRC-protected summary
when UART corruption loses the initial response; an INFO barrier drains queued
replies before the next capture. The host requests this retry after 45 seconds.

Two genuine ESP32 captures are preserved under `harmonic-captures/fix-v20-live-iq/`.
Replaying the same recording with tracking disabled made prefix recovery worse.
Joint CR4/5 decoding increased prefix-valid attempts from three to six, but all
failed packet CRC. This is real ESP32-captured RF, decoded again on the ESP32;
it remains distinct from both fresh live reception and synthetic controls.
See `onboard-summary.json` for the latest live results and build hash.

## Current hardware and firmware

- Original ESP32-D0WD-V3 revision 3.1, COM10.
- Existing onboard antenna and USB cable; no extra receiving hardware.
- Heltec transmitter on COM17 is unchanged and supplies the 915 MHz test signal.
- Receiver requests 2744 MHz, gain index 65: an experimental third-harmonic path.
- UART now runs at **115200 baud**. A faster 500000-baud live result was corrupted
  and discarded. Both upload and report transfer were reduced to 115200 baud.
- This historical build remains idle until ARM/DECODE commands. It is not continuously
  receiving while decoding a previous recording or waiting for the host.

## Live operation

From this project directory:

```powershell
& .venv/Scripts/python.exe onboard_receive.py --port COM10 --filtered --window-ms 220 --gain 65 --count 4 --stop-on-packet --output harmonic-captures/my-onboard-run.json
```

The host confirms firmware identity, arms the receiver, and requests four
capture/decode cycles. On completion it stops RF acquisition and closes COM10.
The report distinguishes live/live-filtered from replay/replay-filtered sources.
`--stop-on-packet` ends a longer run once a window contains a CRC-valid packet.

`ONBOARD` reports measured processing time and acquisition/header/failure counts.
They are diagnostics, not received packets. `LORA_CRC_OK` is emitted only after
the payload CRC passes. `RTCM_CRC_OK` additionally requires a valid bridge header,
complete fragment assembly, RTCM length/header validation and CRC-24Q.
The bridge supports fragments up to 220 bytes and RTCM frames up to 1029 bytes.
Only one incomplete frame is retained; a different session/frame replaces it.

Packet payload hex is included in the LoRa report. The RTCM report currently
identifies the reconstructed type, length and frame ID. Continuous delivery to
a GNSS rover has not been implemented or verified.

Device result bodies use `REPORT <length> <crc32>` followed by exactly that many
ASCII bytes. The host rejects a bad CRC, malformed/truncated frame, or unframed
result from the receiver. A replay failure diagnostic is explicitly marked REPLAY.

## On-device method

The initial frequency-estimate mode measured about 595 us per observation,
slower than the 256 us SF7/BW500 symbol period, so it was not used for packet
decoding. The newer mode captures 4096-word nominal 16 MS/s RF snapshots and
stores sign-only I/Q at a decimation of eight, together with CPU-cycle timestamps.
The heap-allocated buffer holds 600–800 blocks depending on contiguous memory;
this board currently allocates 650. Filtered 220 ms capture retains about 639
blocks and 74% coverage. DSP runs after capture. The explicit gaps between
snapshots remain part of the recording; these are not continuous I/Q samples.

The DSP reconstructs short analysis windows from timestamped blocks, compensates
the nominal harmonic offset, and applies third-harmonic dechirping and 512-point
FFTs. Folding the harmonic frequency aliases and multiplying symbol indices by
43 modulo 128 recovers the fixed SF7 alphabet. Preamble energy selects finer
timing/frequency estimates before payload CRC checking. The sync-word pattern,
explicit header checksum and payload CRC remain required.

The header FEC decoder uses max-log bit confidence from measured FFT energies.
For CR4/5 payload blocks, whole-symbol scores feed a 128-state trellis that
enforces the seven even-parity rows across five symbols. Its seven recovered
nibbles retain the diagonal deinterleaver's ordering. Acquisition scratch memory
is reused after acquisition, so this does not consume the RF buffer.
CR4/5 uses parity across all four data bits;
CR4/6–4/8 use their Hamming parity equations. These rules were checked against
[EPFL's gr-lora_sdr encoder](https://github.com/tapparelj/gr-lora_sdr/blob/master/lib/hamming_enc_impl.cc).
The signal processing is fixed to the Heltec's SF7, BW500, sync-0x12 profile;
it is not a general-purpose LoRa receiver.

`DFILTER 220` selects the five-tap mixer/filter before decimation. Snapshot
capture overlaps packing the preceding block. Use `--filtered` to select this
path; the default DECODE command remains the unfiltered diagnostic path. The raw,
packed-I/Q and frequency-trace commands remain explicit diagnostics. Normal
decoding replies contain reports rather than RF arrays.

## Replay verification

Run all four acceptance/rejection checks on the connected ESP32:

```powershell
& .venv/Scripts/python.exe verify_onboard.py
```

The runner also verifies firmware identity, transport checksums, exact payload
bytes, one report per physical reference packet, and that the payload negative
actually exercises CRC rejection instead of only failing the RT prefix gate.

Replays exercise the same decoder on the actual ESP32, with RF capture stopped.
The fixture generator creates a known LoRa packet and its third-harmonic
waveform, quantizes it, and retains timestamped gaps. It labels its output
**SYNTHETIC; not RF evidence**. The RTCM fixture's payload bytes were taken from
the earlier validated Nooelec baseline, but its replay waveform is synthetic.

```powershell
& .venv/Scripts/python.exe onboard_receive.py --replay harmonic-captures/synthetic-rtcm.npz --output harmonic-captures/rtcm-test.json
& .venv/Scripts/python.exe onboard_receive.py --replay harmonic-captures/synthetic-crc-negative.npz --output harmonic-captures/crc-negative-test.json
& .venv/Scripts/python.exe onboard_receive.py --replay harmonic-captures/synthetic-noise.npz --output harmonic-captures/noise-test.json
```

The positive fixture expects its exact 40-byte bridge payload, LoRa CRC success,
and one 28-byte RTCM type 1134 frame. The CRC-negative fixture changes encoded
payload content while preserving its old CRC; the noise fixture contains no
packet. Neither negative should emit an accepted packet.

Earlier diagnostic fixtures/logs produced during development are retained as
history. The `onboard-final-*` reports qualify the historical HRXLORA7 build.
`fix-v12-independent-replay.json` qualifies the transformed SDR replay on HRXLORA12;
`fix-v16-five-tap-live.json` is an earlier live test. Current qualification uses
`qualification-v22-*`; earlier live and same-capture comparison reports are named
`fix-v19-*`, `fix-v20-*`, and `fix-v21-*`. The first CRC-valid fresh live result is
`fix-v22-live-g65.json`; earlier developmental logs do not establish that success.

For an exact-capture diagnostic, add `--save-captures <directory>` to a live run.
After on-device decoding, DUMPPACK exports the same packed buffer with header
and data CRC32 checks. It is saved only when both checks pass. The saved NPZ
includes whether filtering was used; replay selects REPLAYFILTER automatically.
`--no-tracking` permits a comparison on identical samples. Normal operation
does not export I/Q or demodulate on the host.

## Build and limits

`firmware/lib/HarmonicCapture` is an isolated GPL-3.0 copy of the existing
ESP32-Harmonic-RX capture backend, with new bounded capture methods. This project
does not edit the original library. The SRAM aperture reservation, linker
assertions, chip/core/clock checks and dump guard remain enabled. The backend
origin and SDK dependency are documented in `EXPERIMENT.md` and its source.
PHY bit operations also retain attribution to the MIT decoder in `vendor/`.

```powershell
& pio run -d firmware -e esp32 -t upload --upload-port COM10
```

No added hardware is needed for this experiment, but the signal path, actual PLL
lock, sample rate and harmonic origin remain uncalibrated. Gaps, one-bit I/Q,
downsampling and timing uncertainty can corrupt symbols. A passing
header alone does not establish a packet. Processing is sequential and can take
many seconds; a 220 ms requested capture window does not mean continuous reception across
that wall-clock time. No range, packet-loss-rate or GNSS RTK-fix result is claimed.
