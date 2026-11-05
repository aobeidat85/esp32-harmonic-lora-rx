# ESP32-only harmonic LoRa reception experiment

This records the initial chirp experiment. Current autonomous reception is
described in [CONTINUOUS.md](CONTINUOUS.md); the first successful decoder is
described in [ONBOARD.md](ONBOARD.md). Both supersede the initial
HRXLORA1 installation and serial settings below. The historical on/off/on
measurements remain unchanged.

Initial discovery stage, November 5, 2026: the bare original ESP32 captured chirps
consistent with the Heltec transmitter's third harmonic; this initial detector
did not decode packets. **The later HRXLORA22 receiver has now decoded a live
CRC-valid LoRa packet and RTCM1134 frame on the ESP32.** See [ONBOARD.md](ONBOARD.md)
for that result. The Nooelec results are a separate reference baseline.

## Hardware and setup

- Receiver: original ESP32-D0WD-V3 revision 3.1, COM10.
- Receiver input: existing board antenna. No receiver module or extra RF hardware.
- Transmitter: existing Heltec V4.3 on COM17.
- Transmitter firmware unchanged: MobileCM UDP RTCM bridge, 915 MHz, SF7,
  bandwidth 500 kHz, coding rate 4/5, sync word 0x12.
- Receiver requests: 2744, 2745, and 2746 MHz; manual gain indices 45 and 65.
- RF captures: 16,380 preserved uint32 IQ/metadata words at nominal 16 MS/s,
  about 1.024 ms per capture. Snapshots are separated by roughly 1.34 seconds
  while transmitting samples over USB at 500,000 baud.

The original ESP32 was identified before upload. At the user's request, no flash
backup was made. The isolated firmware in `firmware/` was built and flashed to
COM10 successfully, with upload hashes verified. The Heltec was not flashed.
The existing ESP32-Harmonic-RX source files were not modified for this experiment.
After the run the new receiver firmware remains installed, with its RF capture
stopped. It captures again only when instructed by the host.

## Results and controls

The acquisition template and its 0.08 coherent-fraction candidate threshold were
fixed before the first hardware capture. The template represents three times
the phase of an SF7/BW500 LoRa upchirp, with the original 256 us symbol period.
Both IQ slope signs and 32 timing phases are searched. This is an exploratory
detector, not a calibrated false-alarm probability or packet validation check.

| Run | Valid captures | Template candidates | Highest coherent fraction |
| --- | ---: | ---: | ---: |
| Initial ON sweep, three settings and two gains | 60 | 2 | 0.08929 |
| Heltec OFF, 2744 MHz / gain 65 | 15 | 0 | 0.003541 |
| Heltec ON again, 2744 MHz / gain 65 | 10 | 0 | 0.04769 |
| Fresh ON repeat, 2744 MHz / gain 65 | 25 | 2 | 0.14282 |

The first confirmation run stopped after ten valid captures because the next
USB payload failed CRC32. That corrupted frame was discarded. The host was then
updated to save/discard complete CRC-bad USB frames while retaining the known
serial boundary. The subsequent 25-capture run completed without a reported
CRC failure. All 110 accepted captures passed header and raw-payload CRC checks.

The user switched the Heltec off and back on for the control. Its COM17 USB
device disappeared during the off-state recording and reappeared before the
confirmation. On restart its serial diagnostics showed `frames=797`,
`packets=1126`, `errors=0`, confirming it had resumed transmission.

Initial capture `sweep-02/0002.npz` shows several repeated upward sweeps.
Two consecutive reset intervals measure approximately **256 us**. An exploratory
least-squares fit to its 320–480 us ridge gives a slope of **5.894 GHz/s** and
an implied per-symbol sweep of **1.509 MHz**. The third-harmonic hypothesis
predicts **5.859 GHz/s** and **1.500 MHz**. This fit interval was selected from
the discovery capture, so it is descriptive evidence rather than a held-out test.
The off/on confirmation uses the earlier fixed template instead.

![ESP32 captured chirps with transmitter on, an off-state control, and a fresh on-state recording](harmonic-captures/on-off-on.png)

These observations support reception of a Heltec-linked chirped waveform
consistent with its third harmonic. Absolute tuning, PLL lock, harmonic order,
and over-the-air versus conducted/nonlinear coupling are not independently
calibrated. The two boards share a PC USB connection; the power-off control
does not eliminate every possible conducted path. The Nooelec was not used to
collect these ESP32 raw samples.

## Limits of the initial detector

The existing short RF dump can contain about four LoRa symbols. A complete
Heltec packet lasts tens of milliseconds, so these discontinuous captures
cannot simply be concatenated into one packet. A two-tone FSK weather decoder
cannot decode LoRa's chirp-spread modulation.

The next implementation step is sustained acquisition or onboard chirp/symbol
extraction that retains timing across a packet. Payload bytes must then pass
the LoRa CRC and reconstructed RTCM CRC-24Q. Neither validation is claimed here.
Candidate counts are not a packet reception rate.

## Reproduce with the currently installed receiver firmware

From this directory, leave the Heltec transmitting:

```powershell
& .venv/Scripts/python.exe harmonic_probe.py self-check
& .venv/Scripts/python.exe harmonic_probe.py capture --port COM10 --count 25 --frequencies 2744 --gains 65 --output harmonic-captures/new-run
& .venv/Scripts/python.exe harmonic_probe.py analyze harmonic-captures/new-run
```

Choose a new output directory each time. Capture does not flash, reset, or modify
the transmitter. It requires the identified `HRXLORA1 ESP32-D0WD-V3` firmware
response and disarms the receiver after the run. Analysis runs on the PC; the
RF samples originate in the ESP32. `self-check` uses explicitly synthetic chirp,
noise, and CW data and does not count as RF reception evidence.

Saved evidence is under `harmonic-captures/`: original IQ words and metadata in
each `.npz`, per-run `capture-summary.json`, ranked `analysis.json`, transport
failure notes, the chirp fit, and `on-off-on.png`. The summary across all runs
is `experiment-summary.json` in this project directory.

## Source and build

`firmware/main.cpp` is adapted from the existing GPL-3.0-only
ESP32-Harmonic-RX `examples/ExperimentalSatelliteCapture` diagnostic, changing
the firmware identity and restricting ARM requests to 2742–2747 MHz.
`sram_guard.ld` and `firmware/LICENSE` retain the original safeguards and license.
The build uses the existing library at
`../ESP32-Harmonic-RX`, Arduino-ESP32 2.0.17,
PlatformIO espressif32 6.12.0, and a 240 MHz original ESP32. The exact local
library included preexisting LTE-related edits; this experiment preserved them.
Other ESP32 families are not supported by this firmware.

To deliberately rebuild/reinstall the experiment receiver on the verified board:

```powershell
& pio run -d firmware -e esp32
& pio run -d firmware -e esp32 -t upload --upload-port COM10
```
