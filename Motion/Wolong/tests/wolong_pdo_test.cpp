#include <stdint.h>
#include <cstring>
#include <cassert>
#include <cstdio>
typedef uint8_t BYTE;
typedef uint16_t WORD;
typedef uint32_t DWORD;
#include "../MotEcat/WolongPdo.h"
int main() {
    assert(sizeof(int)==4 && sizeof(short)==2);
    const BYTE tx[25]={
      0x34,0x12, 0x27,0x04, 0x09,
      0xFE,0xFF,0xFF,0xFF,
      0x03,0x00,
      0x78,0x56,0x34,0x12,
      0xFD,0xFF,0xFF,0xFF,
      0xEF,0xCD,0xAB,0x89,
      0x9C,0xFF};
    WOLONG_TX_DATA v; WolongDecodeTx(tx,&v);
    assert(v.errorCode==0x1234 && v.statusword==0x0427 && v.mode==9);
    assert(v.position==-2 && v.probeStatus==3 && v.probePosition==0x12345678);
    assert(v.followingError==-3 && v.digitalInputs==0x89ABCDEFUL && v.torque==-100);
    BYTE guarded[17]; memset(guarded,0xAA,sizeof(guarded));
    WolongEncodeRx(guarded+1,0x000F,8,1000,-2,0x0101,0x12345678);
    const BYTE expected[15]={0x0F,0x00,8,0xE8,0x03,0xFE,0xFF,0xFF,0xFF,1,1,0x78,0x56,0x34,0x12};
    assert(memcmp(guarded+1,expected,15)==0);
    assert(guarded[0]==0xAA && guarded[16]==0xAA);
    assert(WolongLayoutMatches(160,200,240,120));
    assert(!WolongLayoutMatches(161,200,240,120));
    assert(!WolongLayoutMatches(160,208,240,120));
    assert(!WolongLayoutMatches(160,200,240,128));
    assert(!WolongLayoutMatches(65536UL*8,200,240,120));
    puts("PASS: all Tx/Rx fields, signed values, output boundaries and layout checks");
}
