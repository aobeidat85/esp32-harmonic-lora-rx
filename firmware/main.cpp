// SPDX-License-Identifier: GPL-3.0-only
// Experimental LoRa third-harmonic capture. Receive-only; no packet decoding claim.
#include <Esp32HarmonicRx.h>
#include <inttypes.h>
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "OnboardLoRa.h"
#include "ContinuousCapture.h"
using namespace esp32_harmonic_rx;
Receiver radio;
OnboardLoRa decoder;
SemaphoreHandle_t reportMutex=nullptr;
QueueHandle_t reportQueue=nullptr;
std::atomic<bool> reportAsync{false},reportTxBusy{false};
std::atomic<uint32_t> reportDrops{0},reportPending{0};
StaticSemaphore_t reportMutexStorage;
ContinuousCapture continuous(radio,decoder);
PackedIqBlock *packed=nullptr;
uint32_t *raw=nullptr;
FrequencyObservation *trace=nullptr;
unsigned packedCapacity=0;
unsigned lastPackedCount=0;
bool lastPackedFiltered=false;
char line[80];
size_t used = 0;
bool overflow = false;
bool startContinuous(unsigned mhz,unsigned gain){
  if(continuous.enabled || mhz<2742 || mhz>2747 || gain>81 || !packed)return false;
  lastPackedCount=0;
  if(!decoder.begin())return false;
  radio.end();Config cfg;cfg.tuningMHz=mhz;cfg.gainIndex=gain;cfg.minimalWifiBuffers=true;
  if(!radio.begin(cfg))return false;
  if(!continuous.start(packed,packedCapacity)){continuous.stop();radio.end();return false;}
  return true;
}

uint32_t crc32(const uint8_t *p, size_t n) {
  uint32_t crc = ~0u;
  while (n--) {
    crc ^= *p++;
    for (unsigned k = 0; k < 8; ++k) crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}
void command() {
  // Never wait for the decode task while holding its output mutex.
  if(!strcmp(line,"STOP") && !continuous.stop()){Serial.println("ERR decoder stop timeout");return;}
  ReportGuard commandGuard;
  if(continuous.enabled && strcmp(line,"INFO") && strcmp(line,"STOP") && strcmp(line,"STATUS") && strcmp(line,"LASTREPORT")){
    Serial.println("ERR continuous receiver active; STOP before reconfiguration");return;
  }
  unsigned mhz, gain, count;
  char extra;
  if (!strcmp(line, "INFO")) {
    Serial.printf("HRXLORA36 %s %u %u 16000000 16380 buffer=%u heap=%u reset=%u uptime_ms=%u stack=%u largest=%u dram=%u\n", ESP.getChipModel(),
                  ESP.getChipRevision(), unsigned(ESP.getEfuseMac() & 0xffffffffu),packedCapacity,ESP.getFreeHeap(),unsigned(esp_reset_reason()),millis(),unsigned(uxTaskGetStackHighWaterMark(nullptr)),heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),heap_caps_get_free_size(MALLOC_CAP_8BIT));
  } else if(!strcmp(line,"STATUS")){
    continuous.status();
  } else if(sscanf(line,"CONT %u %u %c",&mhz,&gain,&extra)==2){
    Serial.println(startContinuous(mhz,gain)?"OK CONTINUOUS":"ERR continuous initialization");
  } else if(!strcmp(line,"BENCH")){
    decoder.benchmark();
  } else if(sscanf(line,"DSP %u %c",&count,&extra)==1 && count<=1){
    decoder.fastDsp=count;Serial.println("OK DSP");
  } else if(sscanf(line,"SEARCH %u %c",&count,&extra)==1 && count && count<=32 && !(count&(count-1))){
    decoder.searchPhases=count;Serial.println("OK SEARCH");
  } else if(sscanf(line,"FFTDEC %u %c",&count,&extra)==1 && (count==1||count==4)){
    decoder.fftDecimation=count;Serial.println("OK FFTDEC");
  } else if(sscanf(line,"FECCHOICES %u %c",&count,&extra)==1 && (count==0||count==8||count==16||count==32||count==128)){
    decoder.fecChoices=count;Serial.println("OK FECCHOICES");
  } else if(sscanf(line,"REFINE %u %c",&count,&extra)==1 && count<=1){
    decoder.fastRefine=count;Serial.println("OK REFINE");
  } else if(sscanf(line,"PAYLOADDEC %u %c",&count,&extra)==1 && (count==0||count==1||count==4)){
    decoder.payloadDecimation=count;Serial.println("OK PAYLOADDEC");
  } else if(!strcmp(line,"LASTREPORT")){
    decoder.resendSummary();
  } else if(sscanf(line,"TRACK %u %c",&count,&extra)==1 && count<=1){
    decoder.carrierTracking=count;Serial.println("OK TRACK");
  } else if(!strcmp(line,"DUMPPACK") && lastPackedCount){
    const auto *bytes=reinterpret_cast<const uint8_t*>(packed);size_t size=lastPackedCount*sizeof(PackedIqBlock);
    char header[100];int length=snprintf(header,sizeof(header),"PACK2 %u %u %08x",lastPackedCount,lastPackedFiltered,unsigned(crc32(bytes,size)));
    Serial.printf("%s %08x\n",header,unsigned(crc32(reinterpret_cast<const uint8_t*>(header),length)));
    for(size_t offset=0;offset<size;offset+=64){Serial.write(bytes+offset,min(size_t(64),size-offset));delayMicroseconds(150);}
    Serial.flush();
  } else if (sscanf(line, "ARM %u %u %c", &mhz, &gain, &extra) == 2 &&
             mhz >= 2742 && mhz <= 2747 && gain <= 81 && !radio.ready()) {
    Config cfg;
    cfg.tuningMHz = mhz; cfg.gainIndex = gain; cfg.minimalWifiBuffers=true;
    if (radio.begin(cfg)) Serial.println("OK ARMED");
    else Serial.printf("ERR %s\n", Receiver::errorMessage(radio.lastError()));
  } else if (!strcmp(line, "STOP")) {
    radio.end(); Serial.println("OK STOPPED");
  } else if(sscanf(line,"DIAG %u %c",&count,&extra)==1){decoder.diagnose(count);
  } else if (sscanf(line,"DECODE %u %c",&count,&extra)==1 && radio.ready()) {
    lastPackedCount=0;
    if(!decoder.begin()){Serial.println("ERR FFT initialization");return;}
    auto result=radio.capturePacked(packed,packedCapacity,count);
    if(result.error!=Error::None){Serial.printf("ERR %s\n",Receiver::errorMessage(result.error));return;}
    lastPackedCount=result.count;lastPackedFiltered=false;
    decoder.process(packed,result.count,"live",false,result.copyCycles/240,result.processingCycles/240);
  } else if(sscanf(line,"DFILTER %u %c",&count,&extra)==1 && radio.ready()){
    lastPackedCount=0;
    if(!decoder.begin()){Serial.println("ERR FFT initialization");return;}
    auto result=radio.capturePacked(packed,packedCapacity,count,true);
    if(result.error!=Error::None){Serial.printf("ERR %s\n",Receiver::errorMessage(result.error));return;}
    lastPackedCount=result.count;lastPackedFiltered=true;
    decoder.process(packed,result.count,"live-filtered",true,result.copyCycles/240,result.processingCycles/240);
  } else if ((!strncmp(line,"REPLAYPACK ",11)||!strncmp(line,"REPLAYFILTER ",13)) && !radio.ready()) {
    lastPackedCount=0;bool filtered=!strncmp(line,"REPLAYFILTER ",13);
    unsigned checksum;
    if(sscanf(line+(filtered?13:11),"%u %x %c",&count,&checksum,&extra)!=2 || !count || count>packedCapacity){Serial.println("ERR replay arguments");return;}
    if(!decoder.begin()){Serial.println("ERR FFT initialization");return;}
    Serial.println("READY");Serial.flush();
    size_t bytes=count*sizeof(PackedIqBlock);
    if(Serial.readBytes(reinterpret_cast<char*>(packed),bytes)!=bytes || crc32(reinterpret_cast<uint8_t*>(packed),bytes)!=checksum){Serial.println("ERR replay transport");return;}
    decoder.process(packed,count,filtered?"replay-filtered":"replay",filtered);
  } else if (sscanf(line, "PACK %u %c", &count, &extra) == 1 && radio.ready()) {
    lastPackedCount=0;
    auto result = radio.capturePacked(packed, packedCapacity, count);
    if (result.error != Error::None) {
      Serial.printf("ERR %s\n", Receiver::errorMessage(result.error)); return;
    }
    const auto *bytes = reinterpret_cast<const uint8_t *>(packed);
    Serial.printf("PACK %u %u %u %08x\n", unsigned(result.count), result.elapsedUs,
                  result.failedSnapshots, unsigned(crc32(bytes, result.count * sizeof(PackedIqBlock))));
    for (size_t offset=0,size=result.count*sizeof(PackedIqBlock);offset<size;offset+=64) {
      Serial.write(bytes+offset,min(size_t(64),size-offset));delayMicroseconds(150);
    }
    Serial.flush();
  } else if (sscanf(line, "TRACE %u %c", &count, &extra) == 1 && radio.ready()) {
    lastPackedCount=0;
    auto result = radio.captureFrequency(trace, 3276, count);
    if (result.error != Error::None) {
      Serial.printf("ERR %s\n", Receiver::errorMessage(result.error)); return;
    }
    const auto *bytes = reinterpret_cast<const uint8_t *>(trace);
    Serial.printf("TRACE %u %u %u %08x\n", unsigned(result.count), result.elapsedUs,
                  result.failedSnapshots, unsigned(crc32(bytes, result.count * sizeof(FrequencyObservation))));
    for (size_t offset = 0, size = result.count * sizeof(FrequencyObservation); offset < size; offset += 64) {
      Serial.write(bytes + offset, min(size_t(64), size - offset));
      delayMicroseconds(150);
    }
    Serial.flush();
  } else if (sscanf(line, "RAW %u %c", &count, &extra) == 1 && radio.ready()) {
    lastPackedCount=0;
    uint64_t triggerUptimeUs = 0;
    auto result = radio.captureRaw(raw, count, &triggerUptimeUs);
    if (result.error != Error::None) {
      Serial.printf("ERR %s\n", Receiver::errorMessage(result.error)); return;
    }
    const auto *bytes = reinterpret_cast<const uint8_t *>(raw);
    char header[112];
    const int headerLength = snprintf(header, sizeof(header), "RAW2 %u %u %08x %" PRIu64,
        unsigned(result.count), result.elapsedUs,
        unsigned(crc32(bytes, result.count * 4)), triggerUptimeUs);
    Serial.printf("%s %08x\n", header,
        unsigned(crc32(reinterpret_cast<const uint8_t *>(header), headerLength)));
    Serial.write(bytes, result.count * 4);
    Serial.flush();
  } else Serial.println("ERR command");
}
void setup() {
  Serial.begin(115200);
  reportMutex=xSemaphoreCreateRecursiveMutexStatic(&reportMutexStorage);
  for(unsigned capacity : {800u,750u,700u,650u,600u}){
    packed=static_cast<PackedIqBlock*>(heap_caps_malloc(capacity*sizeof(PackedIqBlock),MALLOC_CAP_8BIT));
    if(packed){packedCapacity=capacity;break;}
  }
  raw=reinterpret_cast<uint32_t*>(packed);
  trace=reinterpret_cast<FrequencyObservation*>(packed);
  // Standalone receiver: host commands are optional after boot.
  decoder.searchPhases=1;decoder.fecChoices=0;decoder.fastRefine=true;
  if(!startContinuous(2744,65))Serial.println("ERR continuous auto-start");
}
bool readConsole(char &c){
  // HardwareSerial's own mutex may be held during a reporter's UART flush.
  // Never let a console poll stall RF acquisition behind that mutex.
  if(xSemaphoreTakeRecursive(reportMutex,0)!=pdTRUE)return false;
  bool available=Serial.available()>0;if(available)c=Serial.read();
  xSemaphoreGiveRecursive(reportMutex);return available;
}
void loop() {
  char c;
  while (readConsole(c)) {
    if (c == '\r') continue;
    if (c == '\n') {
      line[used] = 0;
      if (overflow){ReportGuard guard;Serial.println("ERR length");}else command();
      used = 0; overflow = false;
    } else if (used < sizeof(line) - 1 && !overflow) line[used++] = c;
    else overflow = true;
  }
  if(continuous.enabled){
    continuous.captureStep();
    static unsigned lastIdle=millis();
    if(millis()-lastIdle>=1000){lastIdle=millis();delay(1);}else taskYIELD();
  }else delay(1);
}
