#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <atomic>
extern SemaphoreHandle_t reportMutex;
extern QueueHandle_t reportQueue;
extern std::atomic<bool> reportAsync,reportTxBusy;
extern std::atomic<uint32_t> reportDrops;
extern std::atomic<uint32_t> reportPending;
struct ReportGuard {
  bool locked=false;
  ReportGuard(){if(reportMutex){xSemaphoreTakeRecursive(reportMutex,portMAX_DELAY);locked=true;}}
  ~ReportGuard(){if(locked)xSemaphoreGiveRecursive(reportMutex);}
};
struct ReportMessage {char body[640];};
__attribute__((noinline)) inline void sendFrameNow(const char *body){
  uint32_t crc=~0u;size_t size=strlen(body);
  for(size_t i=0;i<size;++i){crc^=uint8_t(body[i]);for(int k=0;k<8;++k)crc=(crc>>1)^(0xedb88320u&(0u-(crc&1)));}
  ReportGuard guard;Serial.printf("REPORT %u %08x\n",unsigned(size),~crc);
  Serial.write(reinterpret_cast<const uint8_t*>(body),size);Serial.flush();
}
__attribute__((noinline)) inline void sendDeviceReport(const char *body){
  if(reportAsync.load() && reportQueue){
    ReportMessage message{};
    if(strlen(body)>=sizeof(message.body)){reportDrops.fetch_add(1);return;}
    strcpy(message.body,body);
    reportPending.fetch_add(1);
    if(xQueueSend(reportQueue,&message,0)!=pdTRUE){reportPending.fetch_sub(1);reportDrops.fetch_add(1);}
  }else sendFrameNow(body);
}
