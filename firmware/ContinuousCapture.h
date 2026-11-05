#pragma once
#include <atomic>
#include <esp_timer.h>
#include "OnboardLoRa.h"
using esp32_harmonic_rx::Receiver;
using esp32_harmonic_rx::Error;

// One core owns RF acquisition. The other owns immutable decode windows.
// The producer never waits for DSP: it replaces an unread window when needed.
class ContinuousCapture {
  enum {Free,Capturing,Ready,Decoding};
  struct Buffer {PackedIqBlock *data=nullptr;std::atomic<int> state{Free};unsigned count=0;uint32_t origin=0;};
  Buffer buffers_[2];Receiver &radio_;OnboardLoRa &decoder_;
  unsigned capacity_=0,overlap_=0,ringHead_=0,ringCount_=0,sinceOffer_=0;
  std::atomic_flag ringLock_=ATOMIC_FLAG_INIT;
  PackedIqBlock scratch_{};
  std::atomic<bool> stop_{true},workerRunning_{false};
  std::atomic<uint32_t> blocks_{0},windows_{0},dropped_{0},processed_{0},packets_{0},frames_{0},inputDrops_{0},lastDecodeUs_{0},maxDecodeUs_{0};
  int64_t startedUs_=0;bool prepared_=false;
  TaskHandle_t task_=nullptr;
  static void transmit(void*){
    ReportMessage message;
    while(true){
      if(xQueueReceive(reportQueue,&message,portMAX_DELAY)==pdTRUE){reportTxBusy.store(true);sendFrameNow(message.body);reportTxBusy.store(false);reportPending.fetch_sub(1);}
    }
  }
  static void publish(const PackedIqBlock &block,void *ctx){static_cast<ContinuousCapture*>(ctx)->put(block);}
  void put(const PackedIqBlock &block){
    if(stop_.load(std::memory_order_relaxed))return;
    blocks_.fetch_add(1,std::memory_order_relaxed);
    if(ringLock_.test_and_set(std::memory_order_acquire)){inputDrops_.fetch_add(1);return;}
    // Buffer 1 is a producer-only ring. Buffer 0 is an immutable DSP snapshot.
    auto &ring=buffers_[1];ring.data[ringHead_]=block;
    ringHead_=(ringHead_+1)%capacity_;if(ringCount_<capacity_)++ringCount_;
    if(ringCount_==capacity_ && ++sinceOffer_>=capacity_-overlap_){sinceOffer_=0;windows_.fetch_add(1,std::memory_order_release);}
    ringLock_.clear(std::memory_order_release);
  }
  static void worker(void *ctx){static_cast<ContinuousCapture*>(ctx)->consume();}
  void consume(){
    unsigned lastStatus=millis(),lastOffer=0;
    while(!stop_.load(std::memory_order_acquire)){
      bool found=false;
      unsigned offer=windows_.load(std::memory_order_acquire);
      if(offer>lastOffer && !ringLock_.test_and_set(std::memory_order_acquire)){
        offer=windows_.load(std::memory_order_acquire);
        auto &snapshot=buffers_[0];auto &ring=buffers_[1];unsigned tail=capacity_-ringHead_;
        memcpy(snapshot.data,ring.data+ringHead_,tail*sizeof(PackedIqBlock));
        memcpy(snapshot.data+tail,ring.data,ringHead_*sizeof(PackedIqBlock));
        ringLock_.clear(std::memory_order_release);
        uint32_t origin=snapshot.data[0].startCycles;
        for(unsigned n=0;n<capacity_;++n)snapshot.data[n].startCycles-=origin;
        snapshot.count=capacity_;snapshot.origin=origin;
        if(offer>lastOffer+1)dropped_.fetch_add(offer-lastOffer-1);lastOffer=offer;
        found=true;unsigned began=micros();
        decoder_.process(snapshot.data,snapshot.count,"continuous-filtered",true);
        unsigned elapsed=micros()-began;lastDecodeUs_.store(elapsed);
        if(elapsed>maxDecodeUs_.load())maxDecodeUs_.store(elapsed);
        processed_.fetch_add(1);packets_.fetch_add(decoder_.packets);frames_.fetch_add(decoder_.rtcmFrames);
      }
      // Let the core0 idle task run; RF capture on core1 continues meanwhile.
      if(!found || stop_.load())vTaskDelay(1);else taskYIELD();
      if(millis()-lastStatus>=1000){lastStatus=millis();status();}
    }
    decoder_.continuousMode=false;decoder_.cancellation=nullptr;
    workerRunning_.store(false,std::memory_order_release);vTaskDelete(nullptr);
  }
 public:
  std::atomic<bool> enabled{false};
  ContinuousCapture(Receiver &radio,OnboardLoRa &decoder):radio_(radio),decoder_(decoder){}
  bool start(PackedIqBlock *storage,unsigned totalCapacity){
    if(enabled || workerRunning_.load() || !radio_.ready() || !storage || totalCapacity<600)return false;
    uintptr_t a=reinterpret_cast<uintptr_t>(storage),z=a+totalCapacity*sizeof(*storage);
    if(a<0x3fff8000u && z>0x3ffe8000u)return false;
    if(!decoder_.begin())return false;
    // Reserve the DMA-copy workspace before allocating task stacks/queues.
    auto preflight=radio_.capturePacked(&scratch_,1,10,true);
    if(preflight.error!=Error::None)return false;
    capacity_=totalCapacity/2;overlap_=min(280u,capacity_-16);
    buffers_[0].data=storage;buffers_[1].data=storage+capacity_;
    for(auto &b:buffers_){b.count=0;b.state.store(Free);}
    buffers_[1].state.store(Capturing);ringHead_=ringCount_=sinceOffer_=0;prepared_=false;
    ringLock_.clear();
    blocks_=windows_=dropped_=processed_=packets_=frames_=inputDrops_=lastDecodeUs_=maxDecodeUs_=0;
    startedUs_=esp_timer_get_time();stop_.store(false);
    decoder_.clearStreamHistory();decoder_.continuousMode=true;decoder_.cancellation=&stop_;
    workerRunning_.store(true);
    if(xTaskCreatePinnedToCore(worker,"LoRaDecode",12288,this,2,&task_,0)!=pdPASS){
      workerRunning_.store(false);stop_.store(true);decoder_.continuousMode=false;decoder_.cancellation=nullptr;return false;
    }
    if(!reportQueue){
      reportQueue=xQueueCreate(8,sizeof(ReportMessage));
      if(!reportQueue || xTaskCreatePinnedToCore(transmit,"LoRaReports",3072,nullptr,1,nullptr,0)!=pdPASS){stop_.store(true);return false;}
    }
    reportDrops.store(0);reportAsync.store(true);
    enabled=true;return true;
  }
  bool stop(){
    enabled=false;stop_.store(true,std::memory_order_release);unsigned deadline=millis()+3000;
    while(workerRunning_.load(std::memory_order_acquire) && int32_t(deadline-millis())>0)delay(1);
    while(reportPending.load() && int32_t(deadline-millis())>0)delay(1);
    reportAsync.store(false);
    return !workerRunning_.load(std::memory_order_acquire);
  }
  void captureStep(){
    if(!enabled)return;
    auto result=radio_.capturePacked(&scratch_,1,10,true,publish,this,!prepared_);prepared_=true;
    if(result.error!=Error::None){
      stop();char body[160];snprintf(body,sizeof(body),"STREAM_ERROR code=%u message=%s",unsigned(result.error),Receiver::errorMessage(result.error));OnboardLoRa::emit(body);radio_.end();
    }
  }
  __attribute__((noinline)) void status(){
    uint64_t elapsed=max(int64_t(1),esp_timer_get_time()-startedUs_);
    char body[420];snprintf(body,sizeof(body),"STREAM_STATUS enabled=%u capture_core=1 decode_core=0 uptime_ms=%u blocks=%u windows=%u processed=%u dropped_windows=%u input_drops=%u packets=%u rtcm_frames=%u report_drops=%u coverage=%.4f last_decode_us=%u max_decode_us=%u buffer=%u overlap=%u",unsigned(enabled.load()),unsigned(elapsed/1000),blocks_.load(),windows_.load(),processed_.load(),dropped_.load(),inputDrops_.load(),packets_.load(),frames_.load(),reportDrops.load(),blocks_.load()*256.f/elapsed,lastDecodeUs_.load(),maxDecodeUs_.load(),capacity_,overlap_);OnboardLoRa::emit(body);
  }
};
