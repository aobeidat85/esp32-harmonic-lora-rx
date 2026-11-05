"""Generate explicitly SYNTHETIC LoRa harmonic replay data for firmware checks."""
import json
import argparse
from pathlib import Path
import sys
import numpy as np
from packed_capture import DTYPE
sys.path.insert(0,'vendor')
from lora_decode import _hamming,_header_ok,_deinterleave,_gray,_crc16,_whitening

def codeword(nibble,cr):
    c=[(nibble>>i)&1 for i in range(4)]
    c += [c[0]^c[1]^c[2],c[1]^c[2]^c[3],c[0]^c[1]^c[3]]
    c += [sum(c)%2]
    if cr==1:c[4]=c[0]^c[1]^c[2]^c[3]
    value=0
    for bit in c[:cr+4]:value=(value<<1)|bit
    assert _hamming(value,cr)==nibble
    return value

def symbols(nibbles,sf,cr):
    width=cr+4
    cw=[codeword(v,cr) for v in nibbles]
    rows=[]
    for i in range(width):
        value=0
        for j in range(sf):value=(value<<1)|((cw[(i-j-1)%sf]>>(width-1-i))&1)
        rows.append(value)
    assert [_hamming(c,cr) for c in _deinterleave(rows,sf,width)]==nibbles
    inverse={_gray(k):k for k in range(1<<sf)}
    return [(inverse[v]*(4 if sf==5 else 1)+1)%128 for v in rows]

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--payload-hex')
    parser.add_argument('--output',default='harmonic-captures/synthetic-decoder.npz')
    parser.add_argument('--corrupt-body',action='store_true')
    args=parser.parse_args()
    payload=bytes.fromhex(args.payload_hex) if args.payload_hex else b'RT-ESP32-TEST'
    assert 2<=len(payload)<=232
    head=[len(payload)>>4,len(payload)&15,3,0,0]
    for checksum in range(32):
        head[3]=checksum>>4;head[4]=checksum&15
        if _header_ok(head):break
    assert _header_ok(head)
    h=symbols(head,5,4)
    nib=[]
    for byte,ws in zip(payload,_whitening(len(payload))):nib += [(byte^ws)&15,(byte^ws)>>4]
    crc=_crc16(payload[:-2])^payload[-1]^(payload[-2]<<8)
    nib += [(crc>>(4*i))&15 for i in range(4)]
    # Preserve the RT prefix so the negative exercises payload CRC rejection,
    # rather than only the receiver's early bridge-prefix gate.
    if args.corrupt_body:nib[4 if len(payload)>2 else 0]^=1
    while len(nib)%7:nib.append(0)
    data=[]
    for k in range(0,len(nib),7):data += symbols(nib[k:k+7],7,1)
    rng=np.random.default_rng(546)
    n=300000;t=np.arange(n)/2000000
    signal=(rng.normal(size=n)+1j*rng.normal(size=n))*.03
    pos=10000
    def append(symbol=0,down=False,length=512):
        nonlocal pos
        q=(np.arange(length)/4+symbol)%128
        phase=3*np.pi*(q*q/128-q)
        phase-=phase[0]
        if down:phase=-phase
        signal[pos:pos+length]+=np.exp(1j*phase+2j*np.pi*914000*t[pos:pos+length]);pos+=length
    for _ in range(8):append()
    append(8);append(16)
    append(down=True);append(down=True);append(down=True,length=128)
    for value in h+data:append(value)
    starts=np.arange(0,149500,369.25)*2
    blocks=np.zeros(len(starts),dtype=DTYPE)
    for idx,start in enumerate(starts):
        a=round(start);x=signal[a:a+512]
        values=((x.real>0).astype(np.uint8)|((x.imag>0).astype(np.uint8)<<1)).reshape(-1,4)
        blocks['iq'][idx]=np.sum(values<<np.arange(0,8,2,dtype=np.uint8),axis=1)
        blocks['start'][idx]=a*120;blocks['duration'][idx]=61440
    path=Path(args.output)
    np.savez_compressed(path,blocks=blocks,metadata=json.dumps(dict(source='SYNTHETIC; not RF evidence',payload_hex=payload.hex(),expected_crc_ok=not args.corrupt_body)))
    print(path)

if __name__=='__main__':main()
