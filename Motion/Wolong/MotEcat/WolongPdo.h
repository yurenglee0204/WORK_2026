#ifndef WOLONG_PDO_H
#define WOLONG_PDO_H
/* Project-local type: confirm availability in the deployed Motion/PLC system. */
#define TYPE_WOLONG 21
#define WOLONG_TX_BYTES 25
#define WOLONG_RX_BYTES 15
/* Set to 1 ONLY after filling the IDs from the actual ESI/device. */
#define WOLONG_IDENTIFICATION_ENABLED 0
#define WOLONG_VENDOR_ID 0UL
#define WOLONG_PRODUCT_CODE 0UL
#if WOLONG_IDENTIFICATION_ENABLED && (WOLONG_VENDOR_ID == 0 || WOLONG_PRODUCT_CODE == 0)
#error Fill WOLONG_VENDOR_ID and WOLONG_PRODUCT_CODE before enabling identification.
#endif
/* Windows EV355 x86: 16-bit WORD/short and 32-bit DWORD/int, little-endian. */
static WORD WolongReadU16(const BYTE *p) { WORD v; memcpy(&v,p,2); return v; }
static short WolongReadS16(const BYTE *p) { short v; memcpy(&v,p,2); return v; }
static int WolongReadS32(const BYTE *p) { int v; memcpy(&v,p,4); return v; }
static DWORD WolongReadU32(const BYTE *p) { DWORD v; memcpy(&v,p,4); return v; }
static void WolongWriteU16(BYTE *p, WORD v) { memcpy(p,&v,2); }
static void WolongWriteS32(BYTE *p, int v) { memcpy(p,&v,4); }
struct WOLONG_TX_DATA {
    WORD errorCode, statusword;
    BYTE mode;
    int position;
    WORD probeStatus;
    int probePosition, followingError;
    DWORD digitalInputs;
    short torque;
};
static void WolongDecodeTx(const BYTE *p, WOLONG_TX_DATA *v) {
    v->errorCode = WolongReadU16(p+0);    /* 603F */
    v->statusword = WolongReadU16(p+2);   /* 6041 */
    v->mode = p[4];                      /* 6061 */
    v->position = WolongReadS32(p+5);     /* 6064 */
    v->probeStatus = WolongReadU16(p+9);  /* 60B9 */
    v->probePosition = WolongReadS32(p+11); /* 60BA */
    v->followingError = WolongReadS32(p+15); /* 60F4 */
    v->digitalInputs = WolongReadU32(p+19); /* 60FD */
    v->torque = WolongReadS16(p+23);      /* 6077 */
}
static void WolongEncodeRx(BYTE *p, WORD control, BYTE mode,
                           WORD maxTorque, int position,
                           WORD probeFunction, int velocity) {
    WolongWriteU16(p+0,control);         /* 6040 */
    p[2] = mode;                       /* 6060 */
    WolongWriteU16(p+3,maxTorque);       /* 6072 */
    WolongWriteS32(p+5,position);        /* 607A */
    WolongWriteU16(p+9,probeFunction);   /* 60B8 */
    WolongWriteS32(p+11,velocity);       /* 60FF */
}
static bool WolongLayoutMatches(DWORD inOffsetBits, DWORD inSizeBits,
                                DWORD outOffsetBits, DWORD outSizeBits) {
    return (inOffsetBits % 8 == 0) && (outOffsetBits % 8 == 0)
        && (inSizeBits == WOLONG_TX_BYTES * 8)
        && (outSizeBits == WOLONG_RX_BYTES * 8)
        && (inOffsetBits / 8 <= 65535UL)
        && (outOffsetBits / 8 <= 65535UL);
}
#endif
