# ESP32 harmonic LoRa receiver

Experimental reception of a 915 MHz Heltec LoRa transmitter on an **original ESP32**, without adding a LoRa receiver module. LoRa demodulation, FEC, packet CRC and RTCM frame assembly/CRC24Q run on the ESP32. A local browser dashboard displays received data over USB serial.

This is a research experiment using an undocumented harmonic capture path. Reception is intermittent, capture is gapped, and this is not an established continuous RTCM feed for a rover. A conventional LoRa radio remains the practical choice for a reliable link.

## Hardware and signal

- Original ESP32-D0WD-V3, 240 MHz; the backend checks chip, core clock and reserved SRAM. Other ESP32 variants are unsupported.
- Separate Heltec LoRa transmitter: 915 MHz, SF7, BW 500 kHz, CR4/5, sync 0x12, explicit header, payload CRC.
- Heltec payload protocol: 12-byte `RT` fragment header followed by RTCM3 data, up to 220 bytes per fragment.
- USB serial to a Windows PC; receiver defaults to COM10 at 115200 baud.
- Nooelec SDR is optional and was used only as an independent reference, not to process the ESP32's live RF samples.

The requested receiver tuning is 2744 MHz, an experimental harmonic setting rather than a calibrated RF/PLL measurement. Physical coupling, range and packet-loss rate have not been established.

## Build and flash

Install PlatformIO Core, then run from the repository root:

```powershell
pio run -d firmware -e esp32
pio run -d firmware -e esp32 -t upload --upload-port COM10
```

Close serial monitors and the dashboard server before flashing. Firmware HRXLORA36 starts reception at boot with capture on core 1 and decoding on core 0. The reserved SRAM aperture, linker assertions, DMA checks and sentinel are part of the implementation and must remain enabled.

## Open the dashboard

Create the Python environment (tested with Python 3.14):

```powershell
python -m venv .venv
.venv/Scripts/python.exe -m pip install -r requirements.txt
./Receive-Dashboard.ps1 -Port COM10
```

Open **http://127.0.0.1:9150**. The dashboard shows accepted LoRa payload hex, bridge fragments, validated RTCM message types/frame IDs, counters, coverage and receiver health. Pause freezes the display only; JSON export saves visible events.

The server reads CRC32-protected serial reports, reconnects after USB disconnection and appends logs under `dashboard-logs/`. It does not demodulate RF or change receiver settings. Closing it leaves the ESP32 running. Only one program can own the serial port at a time. The dashboard binds to this computer's loopback interface.

## Monitor without a browser

```powershell
.venv/Scripts/python.exe continuous_receive.py --seconds 60 --output harmonic-captures/my-run.json
```

Passive monitoring leaves the device running. `--start` explicitly restarts the retained profile. See [CONTINUOUS.md](CONTINUOUS.md) for commands and pipeline details.

## Validation and limitations

The same retained real ESP32 recording decoded in 3.343 seconds on HRXLORA36, versus 44.805 seconds on HRXLORA22: approximately 13.4x faster. This is a replay benchmark. Fresh autonomous RF reception separately produced a CRC-valid LoRa packet and RTCM1134 frame on HRXLORA34. Subsequent HRXLORA36 monitoring also reported accepted live packets and frames, with a new 50-byte CRC-valid fragment visible in the dashboard.

Measured capture coverage is about 71.3%; decoding skips stale candidate views while busy. Coverage is not a packet success rate, and skipped overlapping views are not a packet-loss count. Reliable delivery of all RTCM traffic and rover RTK fix are unproven.

```powershell
.venv/Scripts/python.exe -m unittest test_report_stream -v
```

Hardware replay qualification uses local recordings that are **not included** in this public repository. `verify_onboard.py` requires those fixtures and temporarily stops reception. [ONBOARD.md](ONBOARD.md) and [EXPERIMENT.md](EXPERIMENT.md) preserve historical development notes; referenced captures and per-machine JSON reports are local-only. See [SDR_REFERENCE.md](SDR_REFERENCE.md) for the optional PC/SDR baseline.

## License and attribution

GPL-3.0-only; see [LICENSE](LICENSE) and [firmware/THIRD_PARTY_NOTICES.md](firmware/THIRD_PARTY_NOTICES.md). The capture backend derives from [ESPARGOS/esp-sdr](https://github.com/ESPARGOS/esp-sdr), retaining its license. Unmodified reference decoders from [Alex Beal's lr2021-re](https://github.com/beala/lr2021-re) retain their MIT notice in [vendor/LICENSE](vendor/LICENSE).

Raw I/Q, RTCM/GNSS captures, serial logs, virtual environments, compiled firmware and machine-specific reports are excluded from publication.
