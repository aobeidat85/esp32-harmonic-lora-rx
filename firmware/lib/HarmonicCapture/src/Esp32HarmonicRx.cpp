// SPDX-License-Identifier: GPL-3.0-only
// Capture registers and reserved aperture adapted from ESPARGOS/esp-sdr,
// commit 9b61e7465bee6da15209c5c09308043d08c46a1d. See THIRD_PARTY_NOTICES.md.
#include "Esp32HarmonicRx.h"
#include <math.h>
#include "esp_heap_caps.h"
#include "esp_arduino_version.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "heap_memory_layout.h"
#include "soc/dport_access.h"
#include "soc/dport_reg.h"
#include "soc/rtc.h"
#include "soc/soc.h"

#if ESP_ARDUINO_VERSION_MAJOR != 2 || ESP_ARDUINO_VERSION_MINOR != 0 || ESP_ARDUINO_VERSION_PATCH != 17
#error "This private-PHY backend is qualified only with Arduino-ESP32 2.0.17. Use the pinned PlatformIO example."
#endif

SOC_RESERVE_MEMORY_REGION(0x3ffe8000, 0x3fff8000, harmonic_rx_dump);

extern "C" {
void set_chanfreq(unsigned, unsigned);
void rom_set_rf_freq_offset(unsigned, unsigned, int);
unsigned ram_chip_i2c_readReg(unsigned, unsigned, unsigned);
void ram_chip_i2c_writeReg(unsigned, unsigned, unsigned, unsigned);
void rom_pbus_workmode(void);
void rom_pbus_xpd_rx_on(int);
void rom_pbus_xpd_tx_off(void);
void rom_set_rxclk_en(int);
extern char _iram_end, _data_end, _bss_end;
}

namespace esp32_harmonic_rx {
namespace {
constexpr uint32_t DumpCtrl = 0x60033d90u;
constexpr uint32_t DumpStatus = 0x60033d94u;
constexpr uint32_t DumpBytes = 0x60033dc4u;
constexpr uint32_t RxGain = 0x3ff5c02cu;
constexpr uint32_t Sentinel = 0xa55aa55au;
constexpr uintptr_t ApertureStart = 0x3ffe8000u;
constexpr uintptr_t ApertureEnd = 0x3fff8000u;
volatile uint32_t *const samples = reinterpret_cast<volatile uint32_t *>(ApertureStart);
portMUX_TYPE captureMux = portMUX_INITIALIZER_UNLOCKED;
Receiver *radioOwner = nullptr;
inline uint32_t cycles() { uint32_t value; asm volatile("rsr.ccount %0" : "=a"(value)); return value; }
bool validTones(float a, float b) {
  return isfinite(a) && isfinite(b) && a > 0 && b > 0 && a < 7000000 && b < 7000000 && a != b;
}
}  // namespace

bool Receiver::begin(const Config &config) {
  if (ready_ || radioOwner) return fail(Error::RadioInUse);
  if (config.tuningMHz < ESP32_HARMONIC_RX_EXPERIMENTAL_MIN_TUNING_MHZ || config.tuningMHz > 2800 || config.gainIndex > 81 ||
      !validTones(config.tone0Hz, config.tone1Hz)) return fail(Error::InvalidConfig);
  if (getCpuFrequencyMhz() != 240) return fail(Error::UnsupportedCpu);
  if (xPortGetCoreID() != 1) return fail(Error::UnsupportedCore);
  if (reinterpret_cast<uintptr_t>(&_iram_end) > 0x400a8000u ||
      reinterpret_cast<uintptr_t>(&_data_end) > ApertureStart ||
      reinterpret_cast<uintptr_t>(&_bss_end) > ApertureStart) return fail(Error::UnsafeMemoryLayout);
  wifi_mode_t mode;
  if (esp_wifi_get_mode(&mode) == ESP_OK) return fail(Error::RadioInUse);
  esp_err_t event = esp_event_loop_create_default();
  if (event != ESP_OK && event != ESP_ERR_INVALID_STATE) return fail(Error::WifiInitializationFailed);
  wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
  if (config.minimalWifiBuffers) {
    init.static_rx_buf_num = 2;
    init.dynamic_rx_buf_num = 4;
    init.tx_buf_type = 1;
    init.static_tx_buf_num = 0;
    init.dynamic_tx_buf_num = 4;
    init.cache_tx_buf_num = 0;
    init.ampdu_rx_enable = 0;
    init.ampdu_tx_enable = 0;
    init.amsdu_tx_enable = 0;
    init.mgmt_sbuf_num = 6;
  }
  if (esp_wifi_init(&init) != ESP_OK) return fail(Error::WifiInitializationFailed);
  if (esp_wifi_set_storage(WIFI_STORAGE_RAM) != ESP_OK ||
      esp_wifi_set_mode(WIFI_MODE_NULL) != ESP_OK || esp_wifi_start() != ESP_OK ||
      esp_wifi_set_ps(WIFI_PS_NONE) != ESP_OK || esp_wifi_set_promiscuous(true) != ESP_OK ||
      esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    esp_wifi_stop(); esp_wifi_deinit();
    return fail(Error::WifiInitializationFailed);
  }
  config_ = config;
  coefficients(); prepare();
  radioOwner = this; ready_ = true; error_ = Error::None;
  return true;
}

void Receiver::end() {
  if (!ready_ || radioOwner != this) return;
  REG_WRITE(DumpCtrl, 0);
  esp_wifi_set_promiscuous(false); esp_wifi_stop(); esp_wifi_deinit();
  ready_ = false; radioOwner = nullptr;
}

bool Receiver::setTuningMHz(uint16_t value) {
  if (!ready_) return fail(Error::NotReady);
  if (value < ESP32_HARMONIC_RX_EXPERIMENTAL_MIN_TUNING_MHZ || value > 2800) return fail(Error::InvalidConfig);
  config_.tuningMHz = value; prepare(); error_ = Error::None; return true;
}
bool Receiver::setGainIndex(uint8_t value) {
  if (!ready_) return fail(Error::NotReady);
  if (value > 81) return fail(Error::InvalidConfig);
  config_.gainIndex = value; error_ = Error::None; return true;
}
bool Receiver::setToneOffsets(float a, float b) {
  if (!ready_) return fail(Error::NotReady);
  if (!validTones(a, b)) return fail(Error::InvalidConfig);
  config_.tone0Hz = a; config_.tone1Hz = b; coefficients(); error_ = Error::None; return true;
}

void Receiver::coefficients() {
  const float tones[] = {config_.tone0Hz, config_.tone1Hz};
  for (unsigned f = 0; f < 2; ++f) for (unsigned k = 0; k < SamplesPerObservation; ++k) {
    const float window = .5f - .5f * cosf(2 * M_PI * k / (SamplesPerObservation - 1));
    const float angle = 2 * M_PI * tones[f] * k / NominalSampleRate;
    cosine_[f][k] = lroundf(4096 * window * cosf(angle));
    sine_[f][k] = lroundf(4096 * window * sinf(angle));
  }
}

void Receiver::prepare() {
  REG_WRITE(DumpCtrl, 0);
  // Pin the normal LO divider, as in the upstream original-ESP32 backend.
  // A previous alternate-LO experiment must not leak into this profile.
  const unsigned lo = ram_chip_i2c_readReg(0x65, 4, 0);
  ram_chip_i2c_writeReg(0x65, 4, 0, lo & ~0x10u);
  set_chanfreq(2412, 0);
  const unsigned cap = ram_chip_i2c_readReg(0x62, 1, 0);
  ram_chip_i2c_writeReg(0x62, 1, 0, cap & ~0x80u);
  const unsigned xtal = rtc_clk_xtal_freq_get();
  // Upstream original-ESP32 alternate LO divides PLL frequency by 6/5.
  // Normal mode is retained for vendor calibration and all weather builds.
  const bool alternate = ESP32_HARMONIC_RX_EXPERIMENTAL_ALTERNATE_LO &&
                         config_.tuningMHz >= 1842 && config_.tuningMHz < 2210;
  const unsigned pllKHz = config_.tuningMHz * (alternate ? 1200u : 1000u);
  const int offset = ((pllKHz % 1000u) * 1024u + 500u) / 1000u;
  rom_set_rf_freq_offset(xtal == 26 ? 1 : xtal == 24 ? 2 : 0, pllKHz / 1000u, offset);
  rom_pbus_workmode(); rom_pbus_xpd_tx_off(); rom_pbus_xpd_rx_on(1); rom_set_rxclk_en(1);
  const unsigned activeLo = ram_chip_i2c_readReg(0x65, 4, 0);
  ram_chip_i2c_writeReg(0x65, 4, 0, (activeLo & ~0x10u) | (alternate ? 0x10u : 0u));
  const uint32_t gain = REG_READ(RxGain);
  REG_WRITE(RxGain, (gain & 0x7fffff) | (config_.gainIndex << 24) | BIT(23));
  delay(3);
}

CaptureResult Receiver::capture(Observation *output, size_t capacity, uint16_t windowMs) {
  CaptureResult result;
  if (!ready_) { result.error = Error::NotReady; return result; }
  if (getCpuFrequencyMhz() != 240) { result.error = Error::UnsupportedCpu; return result; }
  if (xPortGetCoreID() != 1) { result.error = Error::UnsupportedCore; return result; }
  const uintptr_t outStart = reinterpret_cast<uintptr_t>(output);
  const uintptr_t outEnd = outStart + capacity * sizeof(Observation);
  if (!output || !capacity || capacity > MaxObservations || windowMs < 10 || windowMs > MaxWindowMs ||
      outEnd < outStart || (outStart < ApertureEnd && outEnd > ApertureStart)) {
    result.error = Error::InvalidBuffer; return result;
  }
  prepare();
  for (unsigned k = 0; k < 512; ++k) samples[k] = Sentinel;
  const uint32_t owner = DPORT_REG_READ(DPORT_IRAM_DRAM_AHB_SEL_REG);
  const uint32_t byteSelect = REG_READ(DumpBytes);
  unsigned savedFilter[2];
  for (unsigned k = 0; k < 2; ++k) {
    savedFilter[k] = ram_chip_i2c_readReg(0x67, 1, 1 + k);
    ram_chip_i2c_writeReg(0x67, 1, 1 + k, (savedFilter[k] & ~127u) | 127u);
  }
  delayMicroseconds(10);
  const uint32_t dumpOwner = (owner & ~DPORT_MAC_DUMP_MODE_M) | (3 << DPORT_MAC_DUMP_MODE_S);
  REG_WRITE(DumpBytes, 0x03020100);
  portENTER_CRITICAL(&captureMux);
  const uint32_t started = cycles();
  while (static_cast<uint32_t>(cycles() - started) < windowMs * 240000u && result.count < capacity) {
    samples[SamplesPerObservation - 1] = Sentinel;
    REG_WRITE(DumpCtrl, 0); DPORT_REG_WRITE(DPORT_IRAM_DRAM_AHB_SEL_REG, dumpOwner);
    REG_WRITE(DumpCtrl, SamplesPerObservation | BIT(16) | BIT(31));
    const uint32_t began = cycles();
    REG_SET_BIT(DumpCtrl, BIT(19)); REG_CLR_BIT(DumpCtrl, BIT(19));
    while (!(REG_READ(DumpCtrl) & BIT(18)) && static_cast<uint32_t>(cycles() - began) < 48000u) {}
    const uint32_t ended = cycles(), status = REG_READ(DumpStatus), done = REG_READ(DumpCtrl);
    REG_WRITE(DumpCtrl, 0); DPORT_REG_WRITE(DPORT_IRAM_DRAM_AHB_SEL_REG, owner);
    if (samples[SamplesPerObservation] != Sentinel) { result.error = Error::MemoryGuard; break; }
    if (!(done & BIT(18)) || (status & 0x7fff) != SamplesPerObservation || samples[SamplesPerObservation - 1] == Sentinel) {
      ++result.failedSnapshots; continue;
    }
    int32_t real[2] = {0, 0}, imag[2] = {0, 0};
    for (unsigned k = 4; k < SamplesPerObservation; ++k) {
      const uint32_t word = samples[k];
      int32_t i = word & 1023, q = (word >> 10) & 1023;
      if (i >= 512) i -= 1024;
      if (q >= 512) q -= 1024;
      for (unsigned f = 0; f < 2; ++f) {
        const int32_t c = cosine_[f][k], s = sine_[f][k];
        real[f] += i * c - q * s; imag[f] += i * s + q * c;
      }
    }
    Observation &row = output[result.count++];
    row.timeUs = ((began - started) + (ended - began) / 2) / 240;
    const float a = real[0] / 4096.0f, b = imag[0] / 4096.0f;
    const float c = real[1] / 4096.0f, d = imag[1] / 4096.0f;
    row.tone0Power = a * a + b * b; row.tone1Power = c * c + d * d;
  }
  result.elapsedUs = (cycles() - started) / 240;
  REG_WRITE(DumpCtrl, 0); DPORT_REG_WRITE(DPORT_IRAM_DRAM_AHB_SEL_REG, owner); REG_WRITE(DumpBytes, byteSelect);
  portEXIT_CRITICAL(&captureMux);
  for (unsigned k = 0; k < 2; ++k) ram_chip_i2c_writeReg(0x67, 1, 1 + k, savedFilter[k]);
  result.truncated = result.count == capacity && result.elapsedUs < windowMs * 1000u;
  if (!result.count && result.error == Error::None) result.error = Error::CaptureFailed;
  error_ = result.error;
  return result;
}


// Experimental harmonic-LoRa frequency trace. Each row is one short IQ
// snapshot, with measured time; gaps are retained rather than concatenated.
CaptureResult Receiver::captureFrequency(FrequencyObservation *output, size_t capacity, uint16_t windowMs) {
  CaptureResult result;
  if (!ready_) { result.error = Error::NotReady; return result; }
  if (getCpuFrequencyMhz() != 240 || xPortGetCoreID() != 1) { result.error = Error::UnsupportedCpu; return result; }
  uintptr_t start = reinterpret_cast<uintptr_t>(output), end = start + capacity*sizeof(*output);
  if (!output || !capacity || capacity > 3276 || windowMs < 10 || windowMs > 150 || end < start || (start < ApertureEnd && end > ApertureStart)) {
    result.error = Error::InvalidBuffer; return result;
  }
  prepare();
  constexpr unsigned N = 512;
  int16_t iv[N], qv[N];
  for (unsigned k=0; k<N+4; ++k) samples[k]=Sentinel;
  unsigned savedFilter[2];
  for (unsigned k=0;k<2;++k) {
    savedFilter[k]=ram_chip_i2c_readReg(0x67,1,1+k);
    ram_chip_i2c_writeReg(0x67,1,1+k,(savedFilter[k]&~127u)|127u);
  }
  delayMicroseconds(10);
  uint32_t owner=DPORT_REG_READ(DPORT_IRAM_DRAM_AHB_SEL_REG), byteSelect=REG_READ(DumpBytes);
  uint32_t dumpOwner=(owner&~DPORT_MAC_DUMP_MODE_M)|(3<<DPORT_MAC_DUMP_MODE_S);
  REG_WRITE(DumpBytes,0x03020100);
  portENTER_CRITICAL(&captureMux);
  uint32_t beganWindow=cycles();
  while (uint32_t(cycles()-beganWindow)<windowMs*240000u && result.count<capacity) {
    samples[N-1]=Sentinel;
    REG_WRITE(DumpCtrl,0);
    DPORT_REG_WRITE(DPORT_IRAM_DRAM_AHB_SEL_REG,dumpOwner);
    REG_WRITE(DumpCtrl,N|BIT(16)|BIT(31));
    uint32_t began=cycles();
    REG_SET_BIT(DumpCtrl,BIT(19));REG_CLR_BIT(DumpCtrl,BIT(19));
    while (!(REG_READ(DumpCtrl)&BIT(18)) && uint32_t(cycles()-began)<48000u) {}
    uint32_t ended=cycles(),done=REG_READ(DumpCtrl),status=REG_READ(DumpStatus);
    REG_WRITE(DumpCtrl,0);
    DPORT_REG_WRITE(DPORT_IRAM_DRAM_AHB_SEL_REG,owner);
    if(samples[N]!=Sentinel){result.error=Error::MemoryGuard;break;}
    if(!(done&BIT(18))||(status&0x7fff)!=N||samples[N-1]==Sentinel){++result.failedSnapshots;continue;}
    int32_t si=0,sq=0;
    for(unsigned k=0;k<N;++k){
      uint32_t w=samples[k];int i=(w>>10)&1023,q=w&1023;
      if(i>=512)i-=1024;if(q>=512)q-=1024;
      iv[k]=i;qv[k]=q;si+=i;sq+=q;
    }
    int mi=si/int(N),mq=sq/int(N);
    int64_t energy=0,re[3]={0,0,0},im[3]={0,0,0};
    for(unsigned k=8;k<N;++k){
      int a=iv[k]-mi,b=qv[k]-mq;energy+=a*a+b*b;
      const unsigned lag[3]={4,16,32};
      for(unsigned j=0;j<3;++j)if(k+lag[j]<N){
        int c=iv[k+lag[j]]-mi,d=qv[k+lag[j]]-mq;
        re[j]+=a*c+b*d;im[j]+=a*d-b*c;
      }
    }
    float f4=atan2f(float(im[0]),float(re[0]))*(16000000.f/(2*M_PI*4));
    float f16=atan2f(float(im[1]),float(re[1]))*(16000000.f/(2*M_PI*16));
    f16+=roundf((f4-f16)/1000000.f)*1000000.f;
    float f32=atan2f(float(im[2]),float(re[2]))*(16000000.f/(2*M_PI*32));
    f32+=roundf((f16-f32)/500000.f)*500000.f;
    float coherence=sqrtf(float(re[1])*re[1]+float(im[1])*im[1])/(float(energy)+1);
    auto &row=output[result.count++];
    row.timeUs=((began-beganWindow)+(ended-began)/2)/240;
    row.frequency16Hz=f16;row.frequency32Hz=f32;row.coherence=coherence;row.power=float(energy)/(N-8);
  }
  result.elapsedUs=(cycles()-beganWindow)/240;
  REG_WRITE(DumpCtrl,0);DPORT_REG_WRITE(DPORT_IRAM_DRAM_AHB_SEL_REG,owner);REG_WRITE(DumpBytes,byteSelect);
  portEXIT_CRITICAL(&captureMux);
  for(unsigned k=0;k<2;++k)ram_chip_i2c_writeReg(0x67,1,1+k,savedFilter[k]);
  result.truncated=result.count==capacity && result.elapsedUs<windowMs*1000u;
  if(!result.count && result.error==Error::None)result.error=Error::CaptureFailed;
  error_=result.error;return result;
}


// Exploratory buffered sign-IQ. Decimation is unfiltered and intentionally
// retains aliasing; host knows the physical third-harmonic waveform model.

// Dedicated leaf kernel: no RF-controller state is live in these registers.
static inline int fastMul16(int a,int b){int result;asm("mul16s %0, %1, %2" : "=a"(result) : "a"(a),"a"(b));return result;}
static __attribute__((noinline)) void packHarmonic5(const uint32_t *__restrict__ input,uint8_t *__restrict__ output,int mi,int mq){
  unsigned packedByte=0;int dcI=mi*926,dcQ=mq*926;
  for(unsigned index=0;index<512;++index){
    unsigned base=index*5;uint32_t w=input[base+2];
    int re=(int32_t(w<<12)>>22)*256-dcI,im=(int32_t(w<<22)>>22)*256-dcQ;
    {
      uint32_t a=input[base+1],b=input[base+3];
      int ai=int32_t(a<<12)>>22,aq=int32_t(a<<22)>>22,bi=int32_t(b<<12)>>22,bq=int32_t(b<<22)>>22;
      re+=fastMul16(237,ai+bi)+fastMul16(98,bq-aq);im+=fastMul16(237,aq+bq)+fastMul16(98,ai-bi);
    }
    {
      uint32_t a=input[base],b=input[base+4];
      int ai=int32_t(a<<12)>>22,aq=int32_t(a<<22)>>22,bi=int32_t(b<<12)>>22,bq=int32_t(b<<22)>>22;
      re+=fastMul16(98,ai+bi)+fastMul16(237,bq-aq);im+=fastMul16(98,aq+bq)+fastMul16(237,ai-bi);
    }
    if(index&1){re=-re;im=-im;}
    packedByte|=((re>0?1:0)|(im>0?2:0))<<((index&3)*2);
    if((index&3)==3){output[index/4]=packedByte;packedByte=0;}
  }
}

CaptureResult Receiver::capturePacked(PackedIqBlock *output,size_t capacity,uint16_t windowMs,bool filtered,CapturePublisher publisher,void *context,bool prepareRadio){
  CaptureResult result;
  if(!ready_){result.error=Error::NotReady;return result;}
  if(getCpuFrequencyMhz()!=240 || xPortGetCoreID()!=1){result.error=Error::UnsupportedCpu;return result;}
  uintptr_t start=reinterpret_cast<uintptr_t>(output),end=start+capacity*sizeof(*output);
  if(!output||!capacity||capacity>800||windowMs<10||windowMs>220||end<start||(start<ApertureEnd&&end>ApertureStart)){
    result.error=Error::InvalidBuffer;return result;
  }
  constexpr unsigned N=4096;
  static uint32_t *workspace=nullptr;
  if(!workspace){
    workspace=static_cast<uint32_t*>(heap_caps_malloc(2560*sizeof(uint32_t),MALLOC_CAP_8BIT|MALLOC_CAP_INTERNAL));
    if(!workspace){result.error=Error::InvalidBuffer;return result;}
    uintptr_t workStart=reinterpret_cast<uintptr_t>(workspace),workEnd=workStart+2560*4;
    if((workStart<ApertureEnd && workEnd>ApertureStart)||(workStart<0x400b8000u && workEnd>0x400a8000u)){
      heap_caps_free(workspace);workspace=nullptr;result.error=Error::MemoryGuard;return result;
    }
  }
  if(prepareRadio)prepare();
  for(unsigned k=0;k<N+4;++k)samples[k]=Sentinel;
  unsigned savedFilter[2];
  for(unsigned k=0;k<2;++k){savedFilter[k]=ram_chip_i2c_readReg(0x67,1,1+k);ram_chip_i2c_writeReg(0x67,1,1+k,(savedFilter[k]&~127u)|127u);}
  delayMicroseconds(10);
  uint32_t owner=DPORT_REG_READ(DPORT_IRAM_DRAM_AHB_SEL_REG),byteSelect=REG_READ(DumpBytes);
  uint32_t dumpOwner=(owner&~DPORT_MAC_DUMP_MODE_M)|(3<<DPORT_MAC_DUMP_MODE_S);
  REG_WRITE(DumpBytes,0x03020100);portENTER_CRITICAL(&captureMux);
  uint32_t beganWindow=cycles();int mi=0,mq=0;
  auto kick=[&](){
    samples[N-1]=Sentinel;REG_WRITE(DumpCtrl,0);DPORT_REG_WRITE(DPORT_IRAM_DRAM_AHB_SEL_REG,dumpOwner);
    REG_WRITE(DumpCtrl,N|BIT(16)|BIT(31));uint32_t stamp=cycles();
    REG_SET_BIT(DumpCtrl,BIT(19));REG_CLR_BIT(DumpCtrl,BIT(19));return stamp;
  };
  uint32_t began=kick();
  while(true){
    while(!(REG_READ(DumpCtrl)&BIT(18))&&uint32_t(cycles()-began)<480000u){}
    uint32_t ended=cycles(),done=REG_READ(DumpCtrl),status=REG_READ(DumpStatus);
    REG_WRITE(DumpCtrl,0);delayMicroseconds(10);DPORT_REG_WRITE(DPORT_IRAM_DRAM_AHB_SEL_REG,owner);
    if(samples[N]!=Sentinel){result.error=Error::MemoryGuard;break;}
    if(!(done&BIT(18))||(status&0x7fff)!=N||samples[N-1]==Sentinel){++result.failedSnapshots;result.error=Error::CaptureFailed;break;}
    // Keep the processing workspace in data RAM; executable RAM was too slow.
    uint32_t copyBegan=cycles();
    if(filtered)for(unsigned k=0;k<512;++k){unsigned a=k*8,b=k*5;workspace[b]=samples[a];workspace[b+1]=samples[a+2];workspace[b+2]=samples[a+3];workspace[b+3]=samples[a+4];workspace[b+4]=samples[a+6];}
    else for(unsigned k=0;k<512;++k)workspace[k]=samples[k*8];
    result.copyCycles+=cycles()-copyBegan;
    auto &block=output[publisher?0:result.count];++result.count;
    block.startCycles=(publisher?began:began-beganWindow)+(filtered?45:0);block.captureCycles=ended-began;
    bool more=(publisher || result.count<capacity) && uint32_t(cycles()-beganWindow)<windowMs*240000u;
    // The next DMA dump owns only the reserved aperture. Process the copied
    // snapshot outside it while acquisition is running.
    uint32_t nextBegan=more?kick():0;
    uint32_t processingBegan=cycles();
    if(result.count==1){int32_t si=0,sq=0;for(unsigned k=0;k<512;++k){uint32_t w=workspace[k*(filtered?5:1)];si+=int32_t(w<<12)>>22;sq+=int32_t(w<<22)>>22;}mi=si/512;mq=sq/512;}
    if(filtered)packHarmonic5(workspace,block.iq,mi,mq);
    else for(unsigned k=0;k<128;++k){
      unsigned byte=0;
      for(unsigned j=0;j<4;++j){uint32_t w=workspace[k*4+j];int i=int32_t(w<<12)>>22,q=int32_t(w<<22)>>22;byte|=((i>mi?1:0)|(q>mq?2:0))<<(j*2);}
      block.iq[k]=byte;
    }
    result.processingCycles+=cycles()-processingBegan;
    if(publisher)publisher(block,context);
    if(!more)break;
    began=nextBegan;
  }
  result.elapsedUs=(cycles()-beganWindow)/240;
  REG_WRITE(DumpCtrl,0);DPORT_REG_WRITE(DPORT_IRAM_DRAM_AHB_SEL_REG,owner);REG_WRITE(DumpBytes,byteSelect);portEXIT_CRITICAL(&captureMux);
  for(unsigned k=0;k<2;++k)ram_chip_i2c_writeReg(0x67,1,1+k,savedFilter[k]);
  result.truncated=!publisher&&result.count==capacity&&result.elapsedUs<windowMs*1000u;
  if(!result.count&&result.error==Error::None)result.error=Error::CaptureFailed;
  error_=result.error;return result;
}

CaptureResult Receiver::captureRaw(uint32_t *output, size_t count, uint64_t *startedUptimeUs,
                                   uint64_t targetUptimeUs) {
  return captureRawImpl(output, count, startedUptimeUs, targetUptimeUs, false);
}

CaptureResult Receiver::captureRawView(const uint32_t*& output, size_t count,
                                      uint64_t* startedUptimeUs, uint64_t targetUptimeUs) {
  output = nullptr;
  CaptureResult result = captureRawImpl(nullptr, count, startedUptimeUs, targetUptimeUs, true);
  if (result.error == Error::None) output = reinterpret_cast<const uint32_t*>(ApertureStart);
  return result;
}

CaptureResult Receiver::captureRawImpl(uint32_t *output, size_t count, uint64_t *startedUptimeUs,
                                      uint64_t targetUptimeUs, bool borrowed) {
  CaptureResult result;
  if (!ready_) { result.error = Error::NotReady; return result; }
  if (getCpuFrequencyMhz() != 240) { result.error = Error::UnsupportedCpu; return result; }
  if (xPortGetCoreID() != 1) { result.error = Error::UnsupportedCore; return result; }
  const uintptr_t start = reinterpret_cast<uintptr_t>(output);
  const uintptr_t end = start + count * sizeof(uint32_t);
  if (count < 256 || count > MaxRawSamples || (!borrowed &&
      (!output || (start & 3u) || end < start || (start < ApertureEnd && end > ApertureStart)))) {
    result.error = Error::InvalidBuffer; return result;
  }
  prepare();
  const uint64_t scheduleNow = esp_timer_get_time();
  if (targetUptimeUs && (targetUptimeUs < scheduleNow + 300 ||
                        targetUptimeUs > scheduleNow + 3000000)) {
    result.error = Error::InvalidBuffer; return result;
  }
  for (unsigned k = 0; k < 16384; ++k) samples[k] = Sentinel;
  const uint32_t owner = DPORT_REG_READ(DPORT_IRAM_DRAM_AHB_SEL_REG);
  const uint32_t byteSelect = REG_READ(DumpBytes);
  unsigned savedFilter[2];
  for (unsigned k = 0; k < 2; ++k) {
    savedFilter[k] = ram_chip_i2c_readReg(0x67, 1, 1 + k);
    ram_chip_i2c_writeReg(0x67, 1, 1 + k, (savedFilter[k] & ~127u) | 127u);
  }
  delayMicroseconds(10);
  if (targetUptimeUs) {
    while (static_cast<int64_t>(targetUptimeUs - esp_timer_get_time()) > 3000) delay(1);
    while (static_cast<int64_t>(targetUptimeUs - esp_timer_get_time()) > 80) {}
  }
  portENTER_CRITICAL(&captureMux);
  DPORT_REG_WRITE(DPORT_IRAM_DRAM_AHB_SEL_REG,
      (owner & ~DPORT_MAC_DUMP_MODE_M) | (3 << DPORT_MAC_DUMP_MODE_S));
  REG_WRITE(DumpBytes, 0x03020100);
  REG_WRITE(DumpCtrl, count | BIT(16) | BIT(31)); // Original ESP32 16 MS/s clock.
  if (targetUptimeUs) {
    while (static_cast<uint64_t>(esp_timer_get_time()) < targetUptimeUs) {}
  }
  const uint64_t triggerUptimeUs = esp_timer_get_time();
  const uint32_t began = cycles();
  REG_SET_BIT(DumpCtrl, BIT(19)); REG_CLR_BIT(DumpCtrl, BIT(19));
  while (!(REG_READ(DumpCtrl) & BIT(18)) && static_cast<uint32_t>(cycles() - began) < 480000u) {}
  const uint32_t done = REG_READ(DumpCtrl), status = REG_READ(DumpStatus);
  result.elapsedUs = (cycles() - began) / 240;
  REG_WRITE(DumpCtrl, 0);
  // Match upstream's post-stop settling delay before returning the bank to the CPU.
  delayMicroseconds(10);
  DPORT_REG_WRITE(DPORT_IRAM_DRAM_AHB_SEL_REG, owner);
  REG_WRITE(DumpBytes, byteSelect);
  portEXIT_CRITICAL(&captureMux);
  for (unsigned k = 0; k < 2; ++k) ram_chip_i2c_writeReg(0x67, 1, 1 + k, savedFilter[k]);
  if (!(done & BIT(18)) || (status & 0x7fff) != count) {
    result.error = Error::CaptureFailed;
  } else {
    for (unsigned k = 0; k < 16384; ++k) {
      if ((k < count && samples[k] == Sentinel) || (k >= count && samples[k] != Sentinel)) {
        result.error = Error::MemoryGuard; break;
      }
    }
    if (result.error == Error::None) {
      if (!borrowed) for (size_t k = 0; k < count; ++k) output[k] = samples[k];
      if (startedUptimeUs) *startedUptimeUs = triggerUptimeUs;
      result.count = count;
    }
  }
  error_ = result.error;
  return result;
}

const char *Receiver::errorMessage(Error error) {
  switch (error) {
    case Error::None: return "OK";
    case Error::NotReady: return "Receiver is not initialized";
    case Error::InvalidConfig: return "Invalid tuning, gain, or tone offsets";
    case Error::UnsupportedCpu: return "CPU must run at 240 MHz";
    case Error::UnsupportedCore: return "Call the receiver from Arduino loop on core1";
    case Error::UnsafeMemoryLayout: return "RF capture aperture overlaps application memory";
    case Error::RadioInUse: return "The library requires exclusive ownership of the Wi-Fi radio";
    case Error::WifiInitializationFailed: return "Wi-Fi PHY initialization failed";
    case Error::InvalidBuffer: return "Invalid capture buffer or duration";
    case Error::CaptureFailed: return "No valid RF snapshots captured";
    case Error::MemoryGuard: return "RF capture aperture guard changed";
  }
  return "Unknown receiver error";
}
}  // namespace esp32_harmonic_rx
