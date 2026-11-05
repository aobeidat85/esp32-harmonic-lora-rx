# Third-party notices

The isolated HarmonicCapture backend was copied and adapted from the user's
ESP32-Harmonic-RX project. Its original-ESP32 capture registers, reserved SRAM
aperture, receive setup and tuning derive from
[ESPARGOS/esp-sdr](https://github.com/ESPARGOS/esp-sdr), revision
`9b61e7465bee6da15209c5c09308043d08c46a1d`, particularly
`main/targets/esp32/receiver.c` and `sram_guard.ld`. This derivative firmware
retains GPL-3.0-only licensing; see `LICENSE`.

Onboard PHY bit operations were adapted from Alex Beal's MIT-licensed
`lr2021-re/tools/lora_decode.py`. The original notice, license and unmodified
reference implementation are retained in the parent project's `vendor/`.
Hamming/parity behavior was also checked against EPFL's GPL-3.0 gr-lora_sdr
encoder; this firmware implements the equations rather than vendoring that file.

The pinned Arduino-ESP32 2.0.17 SDK, Espressif PHY libraries, ESP-DSP and
PlatformIO toolchain retain their respective licenses. They are build
dependencies rather than source vendored here. ESP-DSP supplies the FFT.
