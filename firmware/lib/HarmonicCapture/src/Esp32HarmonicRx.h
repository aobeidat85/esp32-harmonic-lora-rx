// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <Arduino.h>

#ifndef ESP32_HARMONIC_RX_SNAPSHOT_SAMPLES
#define ESP32_HARMONIC_RX_SNAPSHOT_SAMPLES 128
#endif

// Separate RF diagnostics may attempt S-band or alternate-LO reception.
// Keep the qualified weather configuration's lower limit unchanged.
#ifndef ESP32_HARMONIC_RX_EXPERIMENTAL_MIN_TUNING_MHZ
#define ESP32_HARMONIC_RX_EXPERIMENTAL_MIN_TUNING_MHZ 2400
#endif
#ifndef ESP32_HARMONIC_RX_EXPERIMENTAL_ALTERNATE_LO
#define ESP32_HARMONIC_RX_EXPERIMENTAL_ALTERNATE_LO 0
#endif
static_assert(ESP32_HARMONIC_RX_EXPERIMENTAL_MIN_TUNING_MHZ == 2400 ||
              ESP32_HARMONIC_RX_EXPERIMENTAL_MIN_TUNING_MHZ == 2300 ||
              (ESP32_HARMONIC_RX_EXPERIMENTAL_MIN_TUNING_MHZ == 1842 &&
               ESP32_HARMONIC_RX_EXPERIMENTAL_ALTERNATE_LO == 1),
              "Lower tuning requires the isolated alternate-LO diagnostic");

#if !defined(CONFIG_IDF_TARGET_ESP32)
#error "ESP32-Harmonic-RX currently supports the original dual-core ESP32 only."
#endif

namespace esp32_harmonic_rx {

struct Config {
  uint16_t tuningMHz = 2744;       // A tuning attempt, not a PLL-lock measurement.
  uint8_t gainIndex = 65;          // PHY table index; not calibrated dB.
  float tone0Hz = 808500.0f;       // Positive physical offset from requested LO.
  float tone1Hz = 1018000.0f;
  bool minimalWifiBuffers = false; // Opt-in for standalone raw DSP; no Wi-Fi networking.
};

struct Observation {
  uint32_t timeUs;
  float tone0Power;
  float tone1Power;
};
struct FrequencyObservation {
  uint32_t timeUs;
  float frequency16Hz;
  float frequency32Hz;
  float coherence;
  float power;
};
struct PackedIqBlock {
  uint32_t startCycles;
  uint32_t captureCycles;
  uint8_t iq[128]; // 512 complex samples; two sign bits per sample, low bits first.
};
using CapturePublisher=void (*)(const PackedIqBlock &,void *);
static_assert(sizeof(Observation) == 12, "Observation wire layout must be 12 bytes");

enum class Error : uint8_t {
  None, NotReady, InvalidConfig, UnsupportedCpu, UnsupportedCore,
  UnsafeMemoryLayout, RadioInUse, WifiInitializationFailed,
  InvalidBuffer, CaptureFailed, MemoryGuard
};

struct CaptureResult {
  uint32_t copyCycles = 0;
  uint32_t processingCycles = 0;
  size_t count = 0;
  uint32_t elapsedUs = 0;
  uint32_t failedSnapshots = 0;
  bool truncated = false;
  Error error = Error::None;
};

class Receiver {
 public:
  Receiver() = default;
  ~Receiver() { end(); }
  Receiver(const Receiver &) = delete;
  Receiver &operator=(const Receiver &) = delete;
  static constexpr size_t SamplesPerObservation = ESP32_HARMONIC_RX_SNAPSHOT_SAMPLES;
  static_assert(SamplesPerObservation == 128 || SamplesPerObservation == 192,
                "Experimental lengths: 128 or 192");
  static constexpr size_t MaxObservations = 6144;
  static constexpr uint16_t MaxWindowMs = 150;
  static constexpr uint32_t NominalSampleRate = 16000000;

  bool begin(const Config &config = Config());
  void end();
  bool ready() const { return ready_; }
  Error lastError() const { return error_; }
  const Config &config() const { return config_; }
  bool setTuningMHz(uint16_t value);
  bool setGainIndex(uint8_t value);
  bool setToneOffsets(float tone0Hz, float tone1Hz);

  // Blocks for up to 150 ms and masks this core's interrupts during acquisition.
  // Call from the Arduino loop on core1, with a global/static output buffer.
  // Observations are timestamped snapshots, not a continuous I/Q stream.
  CaptureResult capture(Observation *output, size_t capacity, uint16_t windowMs = 150);
  CaptureResult captureFrequency(FrequencyObservation *output, size_t capacity, uint16_t windowMs = 150);
  CaptureResult capturePacked(PackedIqBlock *output, size_t capacity, uint16_t windowMs = 150, bool filtered = false,
                              CapturePublisher publisher=nullptr,void *context=nullptr,bool prepareRadio=true);
  // Experimental single contiguous dump. Preserves all 32 bits, including gain/AGC.
  // Nominal 16 MS/s only; no continuity between calls. Separate from weather capture.
  static constexpr size_t MaxRawSamples = 16380;
  // Optional monotonic timestamp immediately before triggering the dump.
  // Uptime is not UTC; sample-zero latency and oscillator accuracy are uncalibrated.
  // Optional target is device uptime, at most 3 seconds ahead. Zero captures now.
  // Scheduling waits with interrupts enabled; only the final <=80 us is critical.
  CaptureResult captureRaw(uint32_t *output, size_t count, uint64_t *startedUptimeUs = nullptr,
                           uint64_t targetUptimeUs = 0);
  // Experimental zero-copy lease of the reserved RF aperture. Read on core1;
  // valid only until the next capture/configuration/end on this Receiver.
  // Never share the view with another task or write to it.
  CaptureResult captureRawView(const uint32_t*& output, size_t count,
                              uint64_t* startedUptimeUs = nullptr, uint64_t targetUptimeUs = 0);
  static const char *errorMessage(Error error);

 private:
  Config config_;
  bool ready_ = false;
  Error error_ = Error::None;
  int16_t cosine_[2][SamplesPerObservation];
  int16_t sine_[2][SamplesPerObservation];
  void coefficients();
  void prepare();
  CaptureResult captureRawImpl(uint32_t* output, size_t count, uint64_t* startedUptimeUs,
                               uint64_t targetUptimeUs, bool borrowed);
  bool fail(Error error) { error_ = error; return false; }
};

}  // namespace esp32_harmonic_rx
