// SPDX-License-Identifier: GPL-3.0-only
// Fixed-profile experimental harmonic LoRa decoder. CRC is required for packets.
// PHY bit operations adapted from vendor/lora_decode.py (MIT, Alex Beal);
// retain vendor/LICENSE. Acquisition and harmonic folding are experimental.
#pragma once
#include <Arduino.h>
#include <Esp32HarmonicRx.h>
#include <dsps_fft2r.h>
#include <math.h>
#include <atomic>
#include "ReportTransport.h"
using esp32_harmonic_rx::PackedIqBlock;

class OnboardLoRa {
  float *fft_=nullptr,*ref_=nullptr,*correction_=nullptr,*folded_=nullptr,*acquisition_=nullptr;
  const PackedIqBlock *blocks_=nullptr;
  unsigned count_=0;
  struct Peak {int bin=0; float fraction=0,frac=0;};
  bool initialized_=false;
  bool filtered_=false;
  uint8_t *rtcm_=nullptr;
  uint8_t *packetScratch_=nullptr;
  uint32_t session_=0;
  uint16_t frameId_=0,frameLength_=0;
  uint8_t fragmentMask_=0,fragmentCount_=0;
  unsigned transforms_=0;
  int totalSamples_=0;
  int acceptedStart_[8]={};
  char lastSummary_[400]={};
  uint8_t reverse7_[128]={};
  uint64_t fftCycles_=0,prepareCycles_=0,foldCycles_=0,trellisCycles_=0;
  uint32_t recentHashes_[32]={};unsigned recentIndex_=0;
  bool cancelled() const{return cancellation && cancellation->load(std::memory_order_relaxed);}
  static uint32_t ticks(){uint32_t c;asm volatile("rsr.ccount %0":"=a"(c));return c;}
  static constexpr float Pi=3.14159265358979323846f;
  static void report(const char *body){
    sendDeviceReport(body);
  }
  void bridge(const uint8_t *payload,unsigned length){
    if(length<12 || payload[0]!='R' || payload[1]!='T')return;
    uint32_t session=uint32_t(payload[2])|(uint32_t(payload[3])<<8)|(uint32_t(payload[4])<<16)|(uint32_t(payload[5])<<24);
    uint16_t id=payload[6]|(payload[7]<<8),size=payload[10]|(payload[11]<<8);
    unsigned index=payload[8],count=payload[9];
    if(size<6||size>1029||!count||count>5||count!=(size+219)/220||index>=count||length-12!=min(unsigned(220),size-index*220))return;
    if(!fragmentMask_ || session!=session_ || id!=frameId_ || size!=frameLength_ || count!=fragmentCount_){
      session_=session;frameId_=id;frameLength_=size;fragmentCount_=count;fragmentMask_=0;
    }
    memcpy(rtcm_+index*220,payload+12,length-12);fragmentMask_|=1<<index;
    if(fragmentMask_!=(1<<count)-1)return;
    fragmentMask_=0;
    if(rtcm_[0]!=0xd3||(rtcm_[1]&0xfc)||(((rtcm_[1]&3)<<8)|rtcm_[2])+6!=size)return;
    uint32_t crc=0;
    for(unsigned i=0;i<size;++i){crc^=uint32_t(rtcm_[i])<<16;for(int k=0;k<8;++k)crc=(crc<<1)^((crc&0x800000)?0x1864cfb:0);crc&=0xffffff;}
    if(crc)return;
    ++rtcmFrames;char line[160];snprintf(line,sizeof(line),"RTCM_CRC_OK source=%s type=%u bytes=%u frame=%u",source,(rtcm_[3]<<4)|(rtcm_[4]>>4),size,id);report(line);
  }
  static int mod(int x,int n){x%=n;return x<0?x+n:x;}
  int start(unsigned b) const {return int((blocks_[b].startCycles+60)/120);}
  void fill(int offset,int onlyBlock=-1) {
    memset(fft_,0,1024*sizeof(float));
    unsigned lo=0,hi=count_;
    while(lo<hi){unsigned mid=(lo+hi)/2;if(start(mid)+512<=offset)lo=mid+1;else hi=mid;}
    for(unsigned b=lo;b<count_ && start(b)<offset+512;++b){
      if(onlyBlock>=0 && b!=unsigned(onlyBlock))continue;
      int first=max(offset,start(b)),last=min(offset+512,start(b)+512);
      float rotation=filtered_?-2*Pi*float((blocks_[b].startCycles-45)%240)/240:
          -2*Pi*float(int64_t(blocks_[b].startCycles)-int64_t(start(b))*120)/240;
      float rc=cosf(rotation),rs=sinf(rotation);
      for(int n=first;n<last;++n){
        unsigned local=n-start(b),v=(blocks_[b].iq[local/4]>>((local%4)*2))&3;
        // Form the exact +/-1 values as integers before converting. This also
        // avoids a GCC 8.4 Xtensa ICE on conditional float-literal loads.
        int sign=1-2*int(!filtered_ && (n&1));
        float a=float((2*int(v&1)-1)*sign),bv=float((2*int((v>>1)&1)-1)*sign);
        fft_[2*(n-offset)]=a*rc-bv*rs;
        fft_[2*(n-offset)+1]=a*rs+bv*rc;
      }
    }
  }
  Peak peakLegacy(int offset,bool down=false,bool correct=false){
    // RF dump restarts do not establish inter-snapshot carrier phase.
    // Add per-snapshot powers, rather than coherently summing unknown phases.
    float energy=0;
    memset(folded_,0,128*sizeof(float));
    unsigned lo=0,hi=count_;
    while(lo<hi){unsigned mid=(lo+hi)/2;if(start(mid)+512<=offset)lo=mid+1;else hi=mid;}
    for(unsigned block=lo;block<count_ && start(block)<offset+512;++block){
      fill(offset,block);
      for(unsigned n=0;n<512;++n){
        float a=fft_[2*n],b=fft_[2*n+1];energy+=a*a+b*b;
        float c=ref_[2*n],d=down?-ref_[2*n+1]:ref_[2*n+1];
        float r=a*c+b*d,i=b*c-a*d;
        if(correct){float e=correction_[2*n],f=correction_[2*n+1];float rr=r*e+i*f;i=i*e-r*f;r=rr;}
        fft_[2*n]=r;fft_[2*n+1]=i;
      }
      dsps_fft2r_fc32(fft_,512);dsps_bit_rev_fc32(fft_,512);++transforms_;
      for(unsigned n=0;n<512;++n)folded_[n%128]+=fft_[2*n]*fft_[2*n]+fft_[2*n+1]*fft_[2*n+1];
    }
    Peak result;
    for(int k=1;k<128;++k)if(folded_[k]>folded_[result.bin])result.bin=k;
    result.fraction=folded_[result.bin]/(512*energy+1e-9f);
    float l=folded_[mod(result.bin-1,128)],c=folded_[result.bin],r=folded_[mod(result.bin+1,128)];
    result.frac=.5f*(l-r)/(l-2*c+r-1e-9f);
    result.frac=max(-.5f,min(.5f,result.frac));
    return result;
  }
  Peak peak(int offset,bool down=false,bool correct=false){
    if(cancelled())return Peak{};
    if(!fastDsp)return peakLegacy(offset,down,correct);
    memset(folded_,0,128*sizeof(float));unsigned energySamples=0,lo=0,hi=count_;
    while(lo<hi){unsigned mid=(lo+hi)/2;if(start(mid)+512<=offset)lo=mid+1;else hi=mid;}
    for(unsigned block=lo;block<count_ && start(block)<offset+512;++block){
      uint32_t began=ticks();memset(fft_,0,1024*sizeof(float));
      int blockStart=start(block),first=max(offset,blockStart),last=min(offset+512,blockStart+512);
      // A constant rotation of one snapshot cannot change its FFT powers.
      // Decode and dechirp only the observed samples, with no reconstruction pass.
      first=offset+((first-offset+fftDecimation-1)/fftDecimation)*fftDecimation;
      for(int global=first;global<last;global+=fftDecimation){
        ++energySamples;
        unsigned n=global-offset,local=global-blockStart,v=(blocks_[block].iq[local/4]>>((local%4)*2))&3;
        float c=ref_[2*n],d=down?-ref_[2*n+1]:ref_[2*n+1];
        if(correct){float e=correction_[2*n],f=correction_[2*n+1];float next=c*e-d*f;d=d*e+c*f;c=next;}
        float a=(v&1)?c:-c,b=(v&2)?d:-d;
        float r=a+b,i=((v&2)?c:-c)-((v&1)?d:-d);
        if(!filtered_ && (global&1)){r=-r;i=-i;}
        fft_[2*(n/fftDecimation)]=r;fft_[2*(n/fftDecimation)+1]=i;
      }
      prepareCycles_+=uint32_t(ticks()-began);began=ticks();
      dsps_fft2r_fc32(fft_,512/fftDecimation);++transforms_;
      fftCycles_+=uint32_t(ticks()-began);began=ticks();
      // FFT output is bit reversed. Groups of four adjacent entries have the
      // same bin modulo 128: reverse9(4*j+q)%128 == reverse7(j).
      // Fold directly, avoiding the complete bit-reversal pass.
      for(unsigned j=0;j<128;++j){
        const float *p=fft_+(fftDecimation==4?2:8)*j;
        float power=p[0]*p[0]+p[1]*p[1];
        if(fftDecimation==1)power+=p[2]*p[2]+p[3]*p[3]+p[4]*p[4]+p[5]*p[5]+p[6]*p[6]+p[7]*p[7];
        folded_[reverse7_[j]]+=power;
      }
      foldCycles_+=uint32_t(ticks()-began);
    }
    Peak result;for(int k=1;k<128;++k)if(folded_[k]>folded_[result.bin])result.bin=k;
    result.fraction=folded_[result.bin]/((1024.f/fftDecimation)*energySamples+1e-9f);
    float l=folded_[mod(result.bin-1,128)],c=folded_[result.bin],r=folded_[mod(result.bin+1,128)];
    result.frac=max(-.5f,min(.5f,.5f*(l-r)/(l-2*c+r-1e-9f)));
    return result;
  }
  void setCorrection(float bins){
    unsigned stride=fastDsp?fftDecimation:1;
    float angle=2*Pi*bins*stride/512,stepR=cosf(angle),stepI=sinf(angle),r=1,i=0;
    for(unsigned n=0;n<512;n+=stride){correction_[2*n]=r;correction_[2*n+1]=i;float next=r*stepR-i*stepI;i=i*stepR+r*stepI;r=next;}
  }
  float zeroScore(int offset,bool down=false){
    if(cancelled())return 0;
    float power=0;unsigned observed=0,lo=0,hi=count_;
    while(lo<hi){unsigned mid=(lo+hi)/2;if(start(mid)+512<=offset)lo=mid+1;else hi=mid;}
    for(unsigned block=lo;block<count_ && start(block)<offset+512;++block){
      int base=start(block),first=max(offset,base),last=min(offset+512,base+512);float re=0,im=0;
      first=offset+((first-offset+fftDecimation-1)/fftDecimation)*fftDecimation;
      for(int global=first;global<last;global+=fftDecimation){
        unsigned n=global-offset,local=global-base,v=(blocks_[block].iq[local/4]>>((local%4)*2))&3;
        float c=ref_[2*n],d=down?-ref_[2*n+1]:ref_[2*n+1],e=correction_[2*n],f=correction_[2*n+1];
        float cr=c*e-d*f,ci=d*e+c*f;
        float r=((v&1)?cr:-cr)+((v&2)?ci:-ci),i=((v&2)?cr:-cr)-((v&1)?ci:-ci);
        if(!filtered_&&(global&1)){r=-r;i=-i;}
        re+=r;im+=i;++observed;
      }
      power+=re*re+im*im;
    }
    return power/((1024.f/fftDecimation)*observed+1e-9f);
  }
  static unsigned gray(unsigned x){return x^(x>>1);}
  static unsigned hamming(unsigned cw,unsigned cr){
    unsigned c[8]={},d[4];for(unsigned i=0;i<cr+4;++i)c[i]=(cw>>(cr+3-i))&1;
    for(int i=0;i<4;++i)d[i]=c[3-i];
    unsigned parity=0;for(unsigned i=0;i<cr+4;++i)parity+=c[i];
    if(cr>=3 && !(cr==4 && parity%2==0)){
      unsigned syndrome=(c[0]^c[1]^c[2]^c[4])+2*(c[1]^c[2]^c[3]^c[5])+4*(c[0]^c[1]^c[3]^c[6]);
      if(syndrome==5)d[3]^=1;if(syndrome==7)d[2]^=1;if(syndrome==3)d[1]^=1;if(syndrome==6)d[0]^=1;
    }
    return (d[0]<<3)|(d[1]<<2)|(d[2]<<1)|d[3];
  }
  static void deinterleave(const unsigned *symbols,unsigned sf,unsigned width,uint8_t *nibbles,unsigned cr){
    unsigned cw[7]={};
    for(unsigned i=0;i<width;++i)for(unsigned j=0;j<sf;++j){unsigned row=mod(int(i)-int(j)-1,sf);cw[row]|=((symbols[i]>>(sf-1-j))&1)<<(width-1-i);}
    for(unsigned i=0;i<sf;++i)nibbles[i]=hamming(cw[i],cr);
  }
  void bitLikelihoods(float *out,unsigned sf){
    float zero[7],one[7];for(unsigned j=0;j<sf;++j)zero[j]=one[j]=-1e30f;
    float scale=0;
    for(unsigned k=0;k<128;++k){
      unsigned raw=(k*43)%128,value=gray(mod(int(raw)-1,128)/(sf==5?4:1));
      float metric=folded_[k];scale+=metric/128;
      for(unsigned j=0;j<sf;++j){if((value>>(sf-1-j))&1)one[j]=max(one[j],metric);else zero[j]=max(zero[j],metric);}
    }
    for(unsigned j=0;j<sf;++j)out[j]=(one[j]-zero[j])/(scale+1e-9f);
  }
  static void softDeinterleave(float llr[8][7],unsigned sf,unsigned width,uint8_t *nibbles){
    float bits[7][8]={};
    for(unsigned i=0;i<width;++i)for(unsigned j=0;j<sf;++j)bits[mod(int(i)-int(j)-1,sf)][i]=llr[i][j];
    for(unsigned row=0;row<sf;++row){
      float best=-1e30f;unsigned choice=0;
      for(unsigned nibble=0;nibble<16;++nibble){
        unsigned c[8];for(unsigned k=0;k<4;++k)c[k]=(nibble>>k)&1;
        c[4]=c[0]^c[1]^c[2];c[5]=c[1]^c[2]^c[3];c[6]=c[0]^c[1]^c[3];c[7]=(c[0]+c[1]+c[2]+c[3]+c[4]+c[5]+c[6])&1;
        if(width==5)c[4]=c[0]^c[1]^c[2]^c[3]; // CR4/5 uses overall parity.
        float score=0;for(unsigned k=0;k<width;++k)score+=(c[k]?1.f:-1.f)*bits[row][k];
        if(score>best){best=score;choice=nibble;}
      }
      nibbles[row]=choice;
    }
  }
  static bool headerOk(const uint8_t *h){
    auto b=[&](unsigned n,unsigned k){return (h[n]>>k)&1;};
    unsigned c4=b(0,3)^b(0,2)^b(0,1)^b(0,0);
    unsigned c3=b(0,3)^b(1,3)^b(1,2)^b(1,1)^b(2,0);
    unsigned c2=b(0,2)^b(1,3)^b(1,0)^b(2,3)^b(2,1);
    unsigned c1=b(0,1)^b(1,2)^b(1,0)^b(2,2)^b(2,1)^b(2,0);
    unsigned c0=b(0,0)^b(1,1)^b(2,3)^b(2,2)^b(2,1)^b(2,0);
    return ((c4<<4)|(c3<<3)|(c2<<2)|(c1<<1)|c0)==((h[3]&1)<<4|h[4]);
  }
  static uint16_t crc16(const uint8_t *p,unsigned n){
    uint16_t crc=0;while(n--){uint8_t b=*p++;for(int k=0;k<8;++k){bool flip=((crc>>8)^(b))&128;crc=(crc<<1)^(flip?0x1021:0);b<<=1;}}return crc;
  }
  void trackCarrier(const Peak &p,float initial,float &tracked){
    if(!carrierTracking || p.fraction<=.035f)return;
    float step=max(-.04f,min(.04f,.15f*p.frac));
    tracked=max(initial-2.f,min(initial+2.f,tracked+step));setCorrection(tracked);
  }
  void jointCr1(int dataStart,unsigned symbol,float initial,float &tracked,uint8_t *out){
    // CR4/5 imposes seven independent even-parity rows across five LoRa
    // symbols. A 128-state trellis finds the most likely valid block using
    // whole-symbol spectral energies, preserving their bit correlations.
    // Acquisition is finished, so its scratch space is available here.
    float *metric=acquisition_,*dp=metric+640,*next=dp+128;
    uint8_t *path=reinterpret_cast<uint8_t*>(next+128);
    for(unsigned col=0;col<5;++col){
      auto p=peak(dataStart+(symbol+col)*512,false,true);float total=0;
      for(unsigned k=0;k<128;++k)total+=folded_[k];
      for(unsigned k=0;k<128;++k){
        unsigned value=gray(mod(int(k*43)-1,128)),contribution=0;
        for(unsigned j=0;j<7;++j)contribution|=((value>>(6-j))&1)<<mod(int(col)-int(j)-1,7);
        metric[col*128+contribution]=folded_[k]*128/(total+1e-9f);
      }
      trackCarrier(p,initial,tracked);
    }
    uint32_t trellisBegan=ticks();
    for(unsigned state=0;state<128;++state){dp[state]=metric[state];path[state]=0;}
    for(unsigned col=1;col<5;++col){
      uint8_t choices[128];unsigned width=0;
      if(fecChoices==128){for(unsigned value=0;value<128;++value)choices[value]=value;width=128;}
      else for(unsigned value=0;value<128;++value){
        unsigned rank=width;while(rank && metric[col*128+value]>metric[col*128+choices[rank-1]]){if(rank<fecChoices)choices[rank]=choices[rank-1];--rank;}
        if(rank<fecChoices){choices[rank]=value;if(width<fecChoices)++width;}
      }
      if(width<128){
        // Missing samples can make a whole symbol's likelihoods equal. Keep
        // every tie at the beam boundary: parity can recover an erased column,
        // whereas arbitrarily retaining its first 16 values destroys that fact.
        float cutoff=metric[col*128+choices[width-1]],epsilon=max(1e-7f,fabsf(cutoff)*1e-6f);
        width=0;for(unsigned value=0;value<128;++value)if(metric[col*128+value]>=cutoff-epsilon)choices[width++]=value;
      }
      for(unsigned state=0;state<128;++state){
        float best=-1e30f;unsigned choice=0;
        for(unsigned option=0;option<width;++option){
          unsigned contribution=choices[option],prev=state^contribution;
          float score=dp[prev]+metric[col*128+contribution];
          if(score>best){best=score;choice=prev;}
        }
        next[state]=best;path[col*128+state]=choice;
      }
      float *swap=dp;dp=next;next=swap;if(fecChoices==128)delay(1);
    }
    unsigned state=0,cw[7]={};
    for(int col=4;col>=0;--col){
      unsigned prev=path[col*128+state],contribution=state^prev;
      for(unsigned row=0;row<7;++row)cw[row]|=((contribution>>row)&1)<<(4-col);
      state=prev;
    }
    for(unsigned row=0;row<7;++row)out[row]=hamming(cw[row],1);
    trellisCycles_+=uint32_t(ticks()-trellisBegan);
  }
  bool decodeAt(int dataStart,float cfo){
    if(fecChoices)return decodeOnce(dataStart,cfo);
    unsigned before=headers;fecChoices=16;
    bool accepted=decodeOnce(dataStart,cfo);
    if(!accepted && headers>before && !cancelled()){
      fecChoices=128;accepted=decodeOnce(dataStart,cfo);
    }
    fecChoices=0;return accepted;
  }
  bool decodeOnce(int dataStart,float cfo){
    if(cancelled())return false;
    float tracked=cfo;setCorrection(tracked);
    float llr[8][7];uint8_t header[7]={};
    for(int k=0;k<8;++k){auto p=peak(dataStart+k*512,false,true);bitLikelihoods(llr[k],5);trackCarrier(p,cfo,tracked);}
    softDeinterleave(llr,5,8,header);
    unsigned length=(header[0]<<4)|header[1],cr=header[2]>>1;bool hasCrc=header[2]&1;
    if(!headerOk(header)||cr<1||cr>4||length<2||!hasCrc)return false;
    if(continuousMode && (cr!=1 || length<18 || length>232))return false;
    ++headers;
    if(headers<=8 && !continuousMode){char diagnostic[180];snprintf(diagnostic,sizeof(diagnostic),"HEADER_CANDIDATE source=%s start_us=%d length=%u cr=%u remaining_us=%d",source,dataStart/2,length,cr,(totalSamples_-dataStart)/2);report(diagnostic);}
    uint8_t *nibble=packetScratch_;memset(nibble,0,520);unsigned need=2*length+4,have=0,symbol=8;
    while(have<need){
      if(cancelled())return false;
      if(dataStart+int(symbol+cr+4)*512>totalSamples_){++truncated;return false;}
      uint8_t out[7];
      if(cr==1)jointCr1(dataStart,symbol,cfo,tracked,out);
      else{
        for(unsigned k=0;k<cr+4;++k){auto p=peak(dataStart+(symbol+k)*512,false,true);bitLikelihoods(llr[k],7);trackCarrier(p,cfo,tracked);}
        softDeinterleave(llr,7,cr+4,out);
      }
      for(int k=0;k<7 && have<need;++k)nibble[have++]=out[k];
      symbol+=cr+4;
      if(have>=4){
        unsigned first=((nibble[1]^15)<<4)|(nibble[0]^15),second=((nibble[3]^15)<<4)|(nibble[2]^14);
        if(first!='R'||second!='T'){++prefixRejected;return false;}
      }
    }
    uint8_t *payload=packetScratch_+520,ws=255;
    for(unsigned j=0;j<length;++j){payload[j]=((nibble[2*j+1]^(ws>>4))<<4)|(nibble[2*j]^(ws&15));ws=(ws<<1)|(((ws>>7)^(ws>>5)^(ws>>4)^(ws>>3))&1);}
    uint16_t rx=nibble[2*length]|(nibble[2*length+1]<<4)|(nibble[2*length+2]<<8)|(nibble[2*length+3]<<12);
    uint16_t calc=crc16(payload,length-2)^payload[length-1]^(payload[length-2]<<8);
    if(calc!=rx && calc!=uint16_t((rx>>8)|(rx<<8))){
      ++crcFailures;
      if(!strcmp(source,"replay") && crcFailures<=3){Serial.printf("REPLAY_CRC_FAILURE start=%d cfo=%.4f length=%u calculated=%04x received=%04x payload=",dataStart,cfo,length,calc,rx);for(unsigned j=0;j<length;++j)Serial.printf("%02x",payload[j]);Serial.println();}
      return false;
    }
    if(cancelled())return false;
    if(continuousMode){
      uint32_t hash=2166136261u;for(unsigned j=0;j<length;++j)hash=(hash^payload[j])*16777619u;
      for(unsigned j=0;j<32;++j)if(recentHashes_[j]==hash){++duplicatePackets;return true;}
      recentHashes_[recentIndex_++%32]=hash;
    }
    // Different acquisition phases can resolve to the same RF packet. Count
    // that measured start once, even when a second hypothesis also passes CRC.
    for(unsigned j=0;j<packets;++j)if(abs(dataStart-acceptedStart_[j])<512)return true;
    if(packets>=8)return true;
    acceptedStart_[packets]=dataStart;++packets;
    char *packetLine=reinterpret_cast<char*>(packetScratch_+775);int position=snprintf(packetLine,620,"LORA_CRC_OK source=%s length=%u payload=",source,length);
    for(unsigned j=0;j<length;++j)position+=snprintf(packetLine+position,620-position,"%02x",payload[j]);
    report(packetLine);bridge(payload,length);
    return true;
  }
  bool refineAndDecode(int syncStart,float referenceUp){
    if(cancelled())return false;
    // Upchirps alone cannot distinguish CFO from symbol timing. Measure both
    // slopes; the third harmonic leaves three discrete timing ambiguities.
    auto unwrap=[&](float value,float reference){while(value-reference>64)value-=128;while(value-reference< -64)value+=128;return value;};
    float uSum=0,weight=0;
    for(int k=1;k<=3;++k){auto p=peak(syncStart-k*512);float u=unwrap(p.bin+p.frac,referenceUp);if(fabsf(u-referenceUp)<2){uSum+=u*p.fraction;weight+=p.fraction;}}
    if(weight<=0)return false;
    float up=uSum/weight;
    auto d1=peak(syncStart+1024,true),d2=peak(syncStart+1536,true);
    auto down=d1.fraction>d2.fraction?d1:d2;
    float downBin=unwrap(down.bin+down.frac,up);
    float cfo=(up+downBin)*.5f;
    cfo-=64*floorf((cfo+32)/64);
    for(int carrierClass=0;carrierClass<2;++carrierClass){
    float classCfo=cfo+64*carrierClass;
    float delta=(up-classCfo)/.75f;
    const float ambiguity=128/.75f;
    delta-=ambiguity*floorf((delta+ambiguity*.5f)/ambiguity);
    const int branch[3]={0,-1,1};
    for(int branchIndex=0;branchIndex<3;++branchIndex){
      if(cancelled())return false;
      float timingDelta=delta+branch[branchIndex]*ambiguity;
      if(fabsf(timingDelta)>256)continue;
      int nominal=syncStart-int(lroundf(timingDelta));
      if(nominal<3*512)continue;
      float best=-1,bestCfo=classCfo;int bestStart=nominal;
      for(int shift=-2;shift<=2;++shift)for(int fine=-1;fine<=1;++fine){
        float trial=classCfo+.25f*fine;setCorrection(trial);float score=0;
        int t=nominal+shift;
        if(fastRefine){for(int k=1;k<=3;++k)score+=zeroScore(t-k*512);score+=max(zeroScore(t+1024,true),zeroScore(t+1536,true));}
        else{for(int k=1;k<=3;++k){auto p=peak(t-k*512,false,true);if(p.bin==0)score+=p.fraction;}auto a=peak(t+1024,true,true),b=peak(t+1536,true,true);score+=max(a.bin==0?a.fraction:0.f,b.bin==0?b.fraction:0.f);}
        if(score>best){best=score;bestCfo=trial;bestStart=t;}
      }
      // At the selected integer timing, estimate the residual UP-chirp CFO.
      // This also absorbs the remaining sub-sample timing offset for payload FFTs.
      float upBest=-1,payloadCfo=bestCfo;
      for(int fine=-4;fine<=4;++fine){
        float trial=bestCfo+.125f*fine;setCorrection(trial);float score=0;
        if(fastRefine){for(int k=1;k<=3;++k)score+=zeroScore(bestStart-k*512);}
        else{for(int k=1;k<=3;++k){auto p=peak(bestStart-k*512,false,true);if(p.bin==0)score+=p.fraction;}}
        if(score>upBest){upBest=score;payloadCfo=trial;}
      }
      if(decodeAt(bestStart+2176,payloadCfo))return true;
    }
    }
    return false;
  }

 public:
  bool carrierTracking=true;
  bool fastDsp=true;
  unsigned searchPhases=32;
  unsigned fftDecimation=1,fecChoices=128;
  bool fastRefine=false;
  unsigned payloadDecimation=0;
  bool continuousMode=false;
  std::atomic<bool> *cancellation=nullptr;
  unsigned duplicatePackets=0;
  void clearStreamHistory(){memset(recentHashes_,0,sizeof(recentHashes_));recentIndex_=duplicatePackets=0;}
  static void emit(const char *body){report(body);}
  void benchmark(){
    if(!count_){Serial.println("ERR no recording loaded");return;}
    unsigned legacyUs=0,fastUs=0;float worst=0;unsigned mismatches=0;bool saved=fastDsp;unsigned savedDecimation=fftDecimation;fftDecimation=1;
    setCorrection(-15.25f);
    for(unsigned test=0;test<32;++test){
      int offset=(test*1709)%max(1,totalSamples_-512);bool down=test&1,correct=test&2;
      fastDsp=false;unsigned began=micros();auto a=peak(offset,down,correct);legacyUs+=micros()-began;
      memcpy(acquisition_,folded_,128*sizeof(float));
      fastDsp=true;began=micros();auto b=peak(offset,down,correct);fastUs+=micros()-began;
      float scale=0,error=0;for(unsigned k=0;k<128;++k){scale=max(scale,acquisition_[k]);error=max(error,fabsf(acquisition_[k]-folded_[k]));}
      worst=max(worst,error/(scale+1e-9f));if(a.bin!=b.bin)++mismatches;delay(1);
    }
    fastDsp=saved;fftDecimation=savedDecimation;char body[220];snprintf(body,sizeof(body),"DSP_BENCH cases=32 legacy_us=%u fast_us=%u max_relative_energy_error=%.8f peak_mismatches=%u",legacyUs,fastUs,worst,mismatches);report(body);
  }
  void resendSummary(){
    ReportGuard guard;
    if(lastSummary_[0])report(lastSummary_);else Serial.println("ERR no completed report");
  }
  void diagnose(unsigned offset){
    if(!blocks_ || offset+32*512>unsigned(totalSamples_)){Serial.println("ERR diagnostic range");return;}
    for(unsigned k=0;k<32;++k){auto up=peak(offset+k*512),down=peak(offset+k*512,true);
      Serial.printf("SYMBOL source=%s index=%u bin=%d fraction=%.5f down_bin=%d down_fraction=%.5f\n",source,k,up.bin,up.fraction,down.bin,down.fraction);}
    Serial.println("DIAG_DONE");
  }
  unsigned headers=0,packets=0,crcFailures=0,truncated=0,syncs=0,rtcmFrames=0,prefixRejected=0;
  float strongest=0;
  const char *source="live";
  bool begin(){
    if(initialized_)return true;
    if(!fft_){
      fft_=static_cast<float*>(malloc(4480*sizeof(float)+1032));
      if(!fft_)return false;
      ref_=fft_+1024;correction_=ref_+1024;folded_=correction_+1024;
      rtcm_=reinterpret_cast<uint8_t*>(fft_+3200);
      acquisition_=reinterpret_cast<float*>(rtcm_+1032);
    }
    if(!packetScratch_){packetScratch_=static_cast<uint8_t*>(malloc(1536));if(!packetScratch_)return false;}
    if(dsps_fft2r_init_fc32(nullptr,512)!=ESP_OK)return false;
    for(int n=0;n<512;++n){float t=n/4.f,phase=3*Pi*(t*t/128-t);ref_[2*n]=cosf(phase);ref_[2*n+1]=sinf(phase);}
    for(unsigned n=0;n<128;++n){unsigned x=n,r=0;for(unsigned k=0;k<7;++k){r=(r<<1)|(x&1);x>>=1;}reverse7_[n]=r;}
    initialized_=true;return true;
  }
  void process(const PackedIqBlock *blocks,unsigned count,const char *origin,bool filtered=false,unsigned copyUs=0,unsigned packUs=0){
    {ReportGuard guard;lastSummary_[0]=0;}
    if(!count || count>800){Serial.println("ERR block count");return;}
    // In background mode a snapshot copy may finish after DMA completed.
    // captureCycles measures polling latency, not the hardware sample count.
    // The producer still verifies DMA status, exact N and the aperture sentinel.
    for(unsigned b=0;b<count;++b)if(blocks[b].startCycles>64000000u || blocks[b].captureCycles<50000u || blocks[b].captureCycles>(continuousMode?4800000u:480000u) || (b && blocks[b].startCycles<=blocks[b-1].startCycles)){
      char body[220];snprintf(body,sizeof(body),"INVALID_CAPTURE index=%u start=%u previous=%u duration=%u first=%u last=%u",b,blocks[b].startCycles,b?blocks[b-1].startCycles:0,blocks[b].captureCycles,blocks[0].startCycles,blocks[count-1].startCycles);report(body);return;
    }
    source=origin;filtered_=filtered;blocks_=blocks;count_=count;totalSamples_=start(count-1)+512;
    headers=packets=crcFailures=truncated=syncs=transforms_=rtcmFrames=prefixRejected=0;strongest=0;
    fftCycles_=prepareCycles_=foldCycles_=trellisCycles_=0;
    unsigned began=micros();
    unsigned acquisitionDecimation=fftDecimation;
    struct Candidate{int offset;float cfo,quality;};
    Candidate candidates[48];unsigned candidateCount=0;
    auto retain=[&](int offset,float cfo,float quality){
      int weakest=-1;unsigned nearby=0;
      for(unsigned j=0;j<candidateCount;++j)if(abs(candidates[j].offset-offset)<256){
        if(abs(candidates[j].offset-offset)<8){if(quality>candidates[j].quality)candidates[j]={offset,cfo,quality};return;}
        ++nearby;if(weakest<0||candidates[j].quality<candidates[weakest].quality)weakest=j;
      }
      if(nearby>=3){if(quality>candidates[weakest].quality)candidates[weakest]={offset,cfo,quality};return;}
      if(candidateCount<48)candidates[candidateCount++]={offset,cfo,quality};
      else{
        unsigned weakest=0;for(unsigned j=1;j<48;++j)if(candidates[j].quality<candidates[weakest].quality)weakest=j;
        if(quality>candidates[weakest].quality)candidates[weakest]={offset,cfo,quality};
      }
    };
    for(int phase=0;phase<32 && !cancelled();phase+=32/searchPhases){
      int run=0,baseline=-1;float baselineFrac=0;
      float *history=acquisition_,*sum=history+768,*current=sum+128;
      memset(acquisition_,0,1024*sizeof(float));unsigned historyCount=0,historySlot=0;
      for(int offset=phase*16;offset+8*512<totalSamples_ && !cancelled();offset+=512){
        auto p=peak(offset);strongest=max(strongest,p.fraction);
        float power=0;for(unsigned k=0;k<128;++k)power+=folded_[k];
        for(unsigned k=0;k<128;++k)current[k]=folded_[k]/(power+1e-9f);
        // Integrate six preamble spectra. A weak symbol need not win its FFT
        // by itself; the two known sync offsets still have to be observed.
        if(historyCount==6){
          unsigned base=0;for(unsigned k=1;k<128;++k)if(sum[k]>sum[base])base=k;
          float mean=sum[base]/6,first=current[(base+24)%128];
          if(mean>.016f && first>.0125f){
            auto secondPeak=peak(offset+512);float secondPower=0;
            for(unsigned k=0;k<128;++k)secondPower+=folded_[k];
            float second=folded_[(base+48)%128]/(secondPower+1e-9f);
            if(second>.0125f){
              auto down=peak(offset+1024,true);
              if(down.fraction>.018f){
                float l=sum[(base+127)%128],c=sum[base],r=sum[(base+1)%128];
                float fractional=max(-.5f,min(.5f,.5f*(l-r)/(l-2*c+r-1e-9f)));
                ++syncs;retain(offset,base+fractional,mean+first+second+down.fraction);
              }
            }
          }
        }
        if(baseline>=0 && run>=4 && mod(p.bin-baseline,128)==24 && p.fraction>.025f){
          auto sync2=peak(offset+512);
          if(mod(sync2.bin-baseline,128)==48 && sync2.fraction>.025f){
            auto down=peak(offset+1024,true);
            if(down.fraction>.025f){
              ++syncs;
              // The two sync symbols are followed by 2.25 downchirps.
              retain(offset,baseline+baselineFrac,p.fraction+sync2.fraction+down.fraction);
            }
          }
        }
        if(p.fraction>.025f){
          int distance=mod(p.bin-baseline+64,128)-64;
          if(baseline>=0 && abs(distance)<=1){
            ++run;baselineFrac=.75f*baselineFrac+.25f*(distance+p.frac);
            if(baselineFrac>.5f){baseline=mod(baseline+1,128);baselineFrac-=1;}
            if(baselineFrac<-.5f){baseline=mod(baseline-1,128);baselineFrac+=1;}
          }
          else{baseline=p.bin;baselineFrac=p.frac;run=1;}
        }else{run=0;baseline=-1;}
        for(unsigned k=0;k<128;++k){unsigned j=historySlot*128+k;sum[k]+=current[k]-history[j];history[j]=current[k];}
        historySlot=(historySlot+1)%6;if(historyCount<6)++historyCount;
        if((transforms_&127)==0)delay(1);
      }
      if(!continuousMode && ((phase+32/searchPhases)%8)==0){char progress[128];snprintf(progress,sizeof(progress),"PROGRESS source=%s phase=%d candidates=%u",source,phase,candidateCount);report(progress);}
      delay(1);
    }
    // Decode each measured event once, using its strongest physical timing
    // candidates. Avoid re-decoding the same packet at all 32 phase grids.
    unsigned acquisitionUs=micros()-began;bool used[48]={};
    if(payloadDecimation)fftDecimation=payloadDecimation;
    while(packets<8 && !cancelled()){
      int first=-1;for(unsigned j=0;j<candidateCount;++j)if(!used[j]&&(first<0||candidates[j].offset<candidates[first].offset))first=j;
      if(first<0)break;
      int groupStart=candidates[first].offset;
      for(unsigned attempt=0;attempt<3;++attempt){
        int best=-1;
        for(unsigned j=0;j<candidateCount;++j)if(!used[j]&&abs(candidates[j].offset-groupStart)<256&&(best<0||candidates[j].quality>candidates[best].quality))best=j;
        if(best<0)break;used[best]=true;
        if(refineAndDecode(candidates[best].offset,candidates[best].cfo))break;
        delay(1);
      }
      for(unsigned j=0;j<candidateCount;++j)if(abs(candidates[j].offset-groupStart)<256)used[j]=true;
    }
    char profile[300];snprintf(profile,sizeof(profile),"DSP_PROFILE fast=%u phases=%u decimation=%u payload_decimation=%u choices=%u refine=%u acquisition_us=%u fft_us=%u prepare_us=%u fold_us=%u trellis_us=%u",fastDsp,searchPhases,acquisitionDecimation,fftDecimation,fecChoices,fastRefine,acquisitionUs,unsigned(fftCycles_/240),unsigned(prepareCycles_/240),unsigned(foldCycles_/240),unsigned(trellisCycles_/240));if(!continuousMode)report(profile);
    ReportGuard summaryGuard;
    snprintf(lastSummary_,sizeof(lastSummary_),"ONBOARD source=%s blocks=%u capture_span_us=%d coverage=%.4f copy_us=%u pack_us=%u decode_us=%u transforms=%u strongest=%.5f syncs=%u headers=%u prefix_rejected=%u crc_failures=%u truncated=%u packets=%u rtcm_frames=%u",source,count,totalSamples_/2,count*512.f/totalSamples_,copyUs,packUs,micros()-began,transforms_,strongest,syncs,headers,prefixRejected,crcFailures,truncated,packets,rtcmFrames);if(!continuousMode || packets)report(lastSummary_);
    fftDecimation=acquisitionDecimation;
  }
};
