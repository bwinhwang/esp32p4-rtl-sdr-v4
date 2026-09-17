#include "mode-s.h"
#include <stdio.h>
#include <time.h>

#define MODE_S_PREAMBLE_US 8 // microseconds
#define MODE_S_LONG_MSG_BITS 112
#define MODE_S_SHORT_MSG_BITS 56
#define MODE_S_FULL_LEN (MODE_S_PREAMBLE_US + MODE_S_LONG_MSG_BITS)

#define MODE_S_ICAO_CACHE_TTL 60 // Time to live of cached addresses.

/* 129x129 entries of (i,q) -> magnitude. dump1090 sizes this as a malloc()
 * byte count (129*129*sizeof(uint16_t)); as an array bound that trailing 2 is
 * an element count and doubles it, leaving the upper half untouched. */
static uint16_t maglut[129 * 129];
static int maglut_initialized = 0;

// =============================== Initialization ===========================

void mode_s_init(mode_s_t *self)
{
    int i, q;

    self->fix_errors = 1;
    self->check_crc = 1;
    self->aggressive = 0;

    self->stat_goodcrc = 0;
    self->stat_badcrc = 0;
    self->stat_fixed = 0;

    // Allocate the ICAO address cache. We use two uint32_t for every entry
    // because it's a addr / timestamp pair for every entry
    memset(&self->icao_cache, 0, sizeof(self->icao_cache));

    // Populate the I/Q -> Magnitude lookup table. It is used because sqrt or
    // round may be expensive and may vary a lot depending on the libc used.
    //
    // We scale to 0-255 range multiplying by 1.4 in order to ensure that every
    // different I/Q pair will result in a different magnitude value, not losing
    // any resolution.
    if (!maglut_initialized)
    {
        for (i = 0; i <= 128; i++)
        {
            for (q = 0; q <= 128; q++)
            {
                maglut[i * 129 + q] = round(sqrt(i * i + q * q) * 360);
            }
        }
        maglut_initialized = 1;
    }
}

// ===================== Mode S detection and decoding  =====================

// Parity table for MODE S Messages.
//
// The table contains 112 elements, every element corresponds to a bit set in
// the message, starting from the first bit of actual data after the preamble.
//
// For messages of 112 bit, the whole table is used. For messages of 56 bits
// only the last 56 elements are used.
//
// The algorithm is as simple as xoring all the elements in this table for
// which the corresponding bit on the message is set to 1.
//
// The latest 24 elements in this table are set to 0 as the checksum at the end
// of the message should not affect the computation.
//
// Note: this function can be used with DF11 and DF17, other modes have the CRC
// xored with the sender address as they are reply to interrogations, but a
// casual listener can't split the address from the checksum.
uint32_t mode_s_checksum_table[] = {
    0x3935ea, 0x1c9af5, 0xf1b77e, 0x78dbbf, 0xc397db, 0x9e31e9, 0xb0e2f0, 0x587178,
    0x2c38bc, 0x161c5e, 0x0b0e2f, 0xfa7d13, 0x82c48d, 0xbe9842, 0x5f4c21, 0xd05c14,
    0x682e0a, 0x341705, 0xe5f186, 0x72f8c3, 0xc68665, 0x9cb936, 0x4e5c9b, 0xd8d449,
    0x939020, 0x49c810, 0x24e408, 0x127204, 0x093902, 0x049c81, 0xfdb444, 0x7eda22,
    0x3f6d11, 0xe04c8c, 0x702646, 0x381323, 0xe3f395, 0x8e03ce, 0x4701e7, 0xdc7af7,
    0x91c77f, 0xb719bb, 0xa476d9, 0xadc168, 0x56e0b4, 0x2b705a, 0x15b82d, 0xf52612,
    0x7a9309, 0xc2b380, 0x6159c0, 0x30ace0, 0x185670, 0x0c2b38, 0x06159c, 0x030ace,
    0x018567, 0xff38b7, 0x80665f, 0xbfc92b, 0xa01e91, 0xaff54c, 0x57faa6, 0x2bfd53,
    0xea04ad, 0x8af852, 0x457c29, 0xdd4410, 0x6ea208, 0x375104, 0x1ba882, 0x0dd441,
    0xf91024, 0x7c8812, 0x3e4409, 0xe0d800, 0x706c00, 0x383600, 0x1c1b00, 0x0e0d80,
    0x0706c0, 0x038360, 0x01c1b0, 0x00e0d8, 0x00706c, 0x003836, 0x001c1b, 0xfff409,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000};

uint32_t mode_s_checksum(unsigned char *msg, int bits)
{
    uint32_t crc = 0;
    int offset = (bits == 112) ? 0 : (112 - 56);
    int j;

    for (j = 0; j < bits; j++)
    {
        int byte = j / 8;
        int bit = j % 8;
        int bitmask = 1 << (7 - bit);

        // If bit is set, xor with corresponding table entry.
        if (msg[byte] & bitmask)
            crc ^= mode_s_checksum_table[j + offset];
    }
    return crc; // 24 bit checksum.
}

// Given the Downlink Format (DF) of the message, return the message length in
// bits.
int mode_s_msg_len_by_type(int type)
{
    /* DF16-31 are all 112 bits. dump1090 listed 16/17/19/20/21 only, which
     * truncated DF18 (TIS-B/ADS-R/non-transponder ADS-B) and DF24-31
     * (Comm-D) to 56 bits, so their CRC could never pass. */
    return (type & 0x10) ? MODE_S_LONG_MSG_BITS : MODE_S_SHORT_MSG_BITS;
}

// Try to fix single bit errors using the checksum. On success modifies the
// original buffer with the fixed version, and returns the position of the
// error bit. Otherwise if fixing failed -1 is returned.
int fix_single_bit_errors(unsigned char *msg, int bits)
{
    int j;
    unsigned char aux[MODE_S_LONG_MSG_BITS / 8];

    for (j = 0; j < bits; j++)
    {
        int byte = j / 8;
        int bitmask = 1 << (7 - (j % 8));
        uint32_t crc1, crc2;

        memcpy(aux, msg, bits / 8);
        aux[byte] ^= bitmask; // Flip j-th bit.

        crc1 = ((uint32_t)aux[(bits / 8) - 3] << 16) |
               ((uint32_t)aux[(bits / 8) - 2] << 8) |
               (uint32_t)aux[(bits / 8) - 1];
        crc2 = mode_s_checksum(aux, bits);

        if (crc1 == crc2)
        {
            // The error is fixed. Overwrite the original buffer with the
            // corrected sequence, and returns the error bit position.
            memcpy(msg, aux, bits / 8);
            return j;
        }
    }
    return -1;
}

// Similar to fix_single_bit_errors() but try every possible two bit
// combination. This is very slow and should be tried only against DF17
// messages that don't pass the checksum, and only in Aggressive Mode.
int fix_two_bits_errors(unsigned char *msg, int bits)
{
    int j, i;
    unsigned char aux[MODE_S_LONG_MSG_BITS / 8];

    for (j = 0; j < bits; j++)
    {
        int byte1 = j / 8;
        int bitmask1 = 1 << (7 - (j % 8));

        // Don't check the same pairs multiple times, so i starts from j+1
        for (i = j + 1; i < bits; i++)
        {
            int byte2 = i / 8;
            int bitmask2 = 1 << (7 - (i % 8));
            uint32_t crc1, crc2;

            memcpy(aux, msg, bits / 8);

            aux[byte1] ^= bitmask1; // Flip j-th bit.
            aux[byte2] ^= bitmask2; // Flip i-th bit.

            crc1 = ((uint32_t)aux[(bits / 8) - 3] << 16) |
                   ((uint32_t)aux[(bits / 8) - 2] << 8) |
                   (uint32_t)aux[(bits / 8) - 1];
            crc2 = mode_s_checksum(aux, bits);

            if (crc1 == crc2)
            {
                // The error is fixed. Overwrite the original buffer with the
                // corrected sequence, and returns the error bit position.
                memcpy(msg, aux, bits / 8);
                // We return the two bits as a 16 bit integer by shifting 'i'
                // on the left. This is possible since 'i' will always be
                // non-zero because i starts from j+1.
                return j | (i << 8);
            }
        }
    }
    return -1;
}

// Hash the ICAO address to index our cache of MODE_S_ICAO_CACHE_LEN elements,
// that is assumed to be a power of two.
uint32_t icao_cache_has_addr(uint32_t a)
{
    // The following three rounds wil make sure that every bit affects every
    // output bit with ~ 50% of probability.
    a = ((a >> 16) ^ a) * 0x45d9f3b;
    a = ((a >> 16) ^ a) * 0x45d9f3b;
    a = ((a >> 16) ^ a);
    return a & (MODE_S_ICAO_CACHE_LEN - 1);
}

// Add the specified entry to the cache of recently seen ICAO addresses. Note
// that we also add a timestamp so that we can make sure that the entry is only
// valid for MODE_S_ICAO_CACHE_TTL seconds.
void add_recently_seen_icao_addr(mode_s_t *self, uint32_t addr)
{
    uint32_t h = icao_cache_has_addr(addr);
    self->icao_cache[h * 2] = addr;
    self->icao_cache[h * 2 + 1] = (uint32_t)time(NULL);
}

// Returns 1 if the specified ICAO address was seen in a DF format with proper
// checksum (not xored with address) no more than * MODE_S_ICAO_CACHE_TTL
// seconds ago. Otherwise returns 0.
int icao_addr_was_recently_seen(mode_s_t *self, uint32_t addr)
{
    uint32_t h = icao_cache_has_addr(addr);
    uint32_t a = self->icao_cache[h * 2];
    int32_t t = self->icao_cache[h * 2 + 1];

    return a && a == addr && time(NULL) - t <= MODE_S_ICAO_CACHE_TTL;
}

// If the message type has the checksum xored with the ICAO address, try to
// brute force it using a list of recently seen ICAO addresses.
//
// Do this in a brute-force fashion by xoring the predicted CRC with the
// address XOR checksum field in the message. This will recover the address: if
// we found it in our cache, we can assume the message is ok.
//
// This function expects mm->msgtype and mm->msgbits to be correctly populated
// by the caller.
//
// On success the correct ICAO address is stored in the mode_s_msg structure in
// the aa3, aa2, and aa1 fiedls.
//
// If the function successfully recovers a message with a correct checksum it
// returns 1. Otherwise 0 is returned.
int brute_force_ap(mode_s_t *self, unsigned char *msg, struct mode_s_msg *mm)
{
    unsigned char aux[MODE_S_LONG_MSG_BYTES];
    int msgtype = mm->msgtype;
    int msgbits = mm->msgbits;

    if (msgtype == 0 ||  // Short air surveillance
        msgtype == 4 ||  // Surveillance, altitude reply
        msgtype == 5 ||  // Surveillance, identity reply
        msgtype == 16 || // Long Air-Air survillance
        msgtype == 20 || // Comm-A, altitude request
        msgtype == 21 || // Comm-A, identity request
        msgtype >= 24)   // Comm-D ELM: the low 3 bits of "DF" are payload
    {
        uint32_t addr;
        uint32_t crc;
        int lastbyte = (msgbits / 8) - 1;

        // Work on a copy.
        memcpy(aux, msg, msgbits / 8);

        // Compute the CRC of the message and XOR it with the AP field so that
        // we recover the address, because:
        //
        // (ADDR xor CRC) xor CRC = ADDR.
        crc = mode_s_checksum(aux, msgbits);
        aux[lastbyte] ^= crc & 0xff;
        aux[lastbyte - 1] ^= (crc >> 8) & 0xff;
        aux[lastbyte - 2] ^= (crc >> 16) & 0xff;

        // If the obtained address exists in our cache we consider the message
        // valid.
        addr = aux[lastbyte] | (aux[lastbyte - 1] << 8) | (aux[lastbyte - 2] << 16);
        if (icao_addr_was_recently_seen(self, addr))
        {
            mm->aa1 = aux[lastbyte - 2];
            mm->aa2 = aux[lastbyte - 1];
            mm->aa3 = aux[lastbyte];
            return 1;
        }
    }
    return 0;
}

// TC5-8 movement field: 7 bits of ground speed in ranges of growing width.
// Returns the midpoint of the range, 0 for stopped/no data/reserved.
static double decode_movement(int mv)
{
    if (mv >= 125) return 0;
    if (mv == 124) return 180;                       // > 175 kt, pick a value
    if (mv >= 109) return 100 + (mv - 109 + 0.5) * 5;
    if (mv >= 94)  return 70 + (mv - 94 + 0.5) * 2;
    if (mv >= 39)  return 15 + (mv - 39 + 0.5);
    if (mv >= 13)  return 2 + (mv - 13 + 0.5) * 0.5;
    if (mv >= 9)   return 1 + (mv - 9 + 0.5) * 0.25;
    if (mv >= 2)   return 0.125 + (mv - 2 + 0.5) * 0.125;
    return 0;                                        // 1 stopped, 0 no data
}

// The 13-bit identity field, as carried in DF5/21 (message bits 20-32) and
// in the TC28 aircraft-status ME (bits 12-24), is Gillham-interleaved:
//
// C1-A1-C2-A2-C4-A4-ZERO-B1-D1-B2-D2-B4-D4
//
// Every group of three bits A, B, C, D is an octal digit. The result is the
// base-ten number that happens to spell those four octal digits (7700).
// For more info: http://en.wikipedia.org/wiki/Gillham_code
static int decode_id13_field(uint32_t f)
{
    int a = (((f >> 7) & 1) << 2) | (((f >> 9) & 1) << 1) | ((f >> 11) & 1);
    int b = (((f >> 1) & 1) << 2) | (((f >> 3) & 1) << 1) | ((f >> 5) & 1);
    int c = (((f >> 8) & 1) << 2) | (((f >> 10) & 1) << 1) | ((f >> 12) & 1);
    int d = (((f >> 0) & 1) << 2) | (((f >> 2) & 1) << 1) | ((f >> 4) & 1);
    return a * 1000 + b * 100 + c * 10 + d;
}

// Decode the 13 bit AC altitude field (in DF 20 and others). Returns the
// altitude, and set 'unit' to either MODE_S_UNIT_METERS or MDOES_UNIT_FEETS.
int decode_ac13_field(unsigned char *msg, int *unit)
{
    int m_bit = msg[3] & (1 << 6);
    int q_bit = msg[3] & (1 << 4);

    if (!m_bit)
    {
        *unit = MODE_S_UNIT_FEET;
        if (q_bit)
        {
            // N is the 11 bit integer resulting from the removal of bit Q and M
            int n = ((msg[2] & 31) << 6) |
                    ((msg[3] & 0x80) >> 2) |
                    ((msg[3] & 0x20) >> 1) |
                    (msg[3] & 15);
            // The final altitude is due to the resulting number multiplied by
            // 25, minus 1000.
            return n * 25 - 1000;
        }
        else
        {
            // TODO: Implement altitude where Q=0 and M=0
        }
    }
    else
    {
        *unit = MODE_S_UNIT_METERS;
        // TODO: Implement altitude when meter unit is selected.
    }
    return 0;
}

// Decode the 12 bit AC altitude field (in DF 17 and others). Returns the
// altitude or 0 if it can't be decoded.
int decode_ac12_field(unsigned char *msg, int *unit)
{
    int q_bit = msg[5] & 1;

    if (q_bit)
    {
        // N is the 11 bit integer resulting from the removal of bit Q
        *unit = MODE_S_UNIT_FEET;
        int n = ((msg[5] >> 1) << 4) | ((msg[6] & 0xF0) >> 4);
        // The final altitude is due to the resulting number multiplied by 25,
        // minus 1000.
        return n * 25 - 1000;
    }
    else
    {
        return 0;
    }
}

// Code 0 reads as a space: undefined, but some transponders pad with it
// and readsb keeps those callsigns. Every other '?' is a real hole.
static const char *ais_charset = " ABCDEFGHIJKLMNOPQRSTUVWXYZ????? ???????????????0123456789??????";

// Eight 6-bit characters from a 48-bit field starting at b (TC1-4 and
// BDS 2,0 share the layout). Returns 0 and an empty string on an
// undefined code -- a callsign with a hole in it is noise, not data.
static int decode_callsign(const unsigned char *b, char *out)
{
    out[0] = ais_charset[b[0] >> 2];
    out[1] = ais_charset[((b[0] & 3) << 4) | (b[1] >> 4)];
    out[2] = ais_charset[((b[1] & 15) << 2) | (b[2] >> 6)];
    out[3] = ais_charset[b[2] & 63];
    out[4] = ais_charset[b[3] >> 2];
    out[5] = ais_charset[((b[3] & 3) << 4) | (b[4] >> 4)];
    out[6] = ais_charset[((b[4] & 15) << 2) | (b[5] >> 6)];
    out[7] = ais_charset[b[5] & 63];
    out[8] = '\0';
    for (int i = 0; i < 8; i++)
        if (out[i] == '?')
        {
            out[0] = '\0';
            return 0;
        }
    return 1;
}

// Bit n (1-based, MSB first) of a 7-byte ME/MV/MB block, numbered as the
// ICAO register tables do.
static inline int fbit(const unsigned char *b, int n)
{
    return (b[(n - 1) >> 3] >> (7 - ((n - 1) & 7))) & 1;
}

static unsigned fbits(const unsigned char *b, int from, int to)
{
    unsigned v = 0;
    for (int i = from; i <= to; i++)
        v = (v << 1) | fbit(b, i);
    return v;
}

// BDS 3,0 (active resolution advisory) layout: 1-8 BDS code, 9-22 ARA,
// 23-26 RAC, 27 RAT, 28 MTE, 29-30 TTI, 31-56 threat identity. In the TC28
// broadcast the first byte is the TC/subtype instead. The register is read
// back empty most of the time and the code byte alone is weak evidence, so
// reject the shapes a real advisory cannot take.
static int acas_ra_plausible(mode_s_t *self, const unsigned char *b, int df, int tc28)
{
    if (!tc28 && b[0] != 0x30) return 0;
    if (!fbit(b, 9) && !fbit(b, 27) && !fbit(b, 28)) return 0;
    if (fbits(b, 9, 28) == 0) return 0;
    if ((fbit(b, 23) && fbit(b, 24)) || (fbit(b, 25) && fbit(b, 26))) return 0;
    if (df == 16) return fbits(b, 29, 56) == 0;      // reserved in the air-air reply
    if (fbit(b, 25) || fbit(b, 26)) return 0;        // turn complements: air-air only
    switch (fbits(b, 29, 30))
    {
    case 0:  return fbits(b, 31, 56) == 0;
    case 1:  return fbits(b, 55, 56) == 0 &&
                    icao_addr_was_recently_seen(self, fbits(b, 31, 54));
    case 2:  return tc28;   // range/bearing form: only trusted from the ES broadcast
    default: return 0;
    }
}

const char *mode_s_acas_ra_text(const unsigned char *b, char *out, size_t n)
{
    if (fbit(b, 27))
    {
        snprintf(out, n, "clear of conflict");
    }
    else if (fbit(b, 9))
    {
        // Single threat, or several resolved by one vertical advisory:
        // 10 corrective, 11 downward sense, 12 increased rate, 13 sense
        // reversal, 14 altitude crossing, 15 positive (vs. a rate limit).
        int corr = fbit(b, 10), down = fbit(b, 11), positive = fbit(b, 15);
        const char *sense = down ? "descend" : "climb";
        if (corr && positive)
            snprintf(out, n, "%s%s%s%s", fbit(b, 12) ? "increase " : "", sense,
                     fbit(b, 13) ? " now" : "", fbit(b, 14) ? " (crossing)" : "");
        else if (corr)
            snprintf(out, n, "level off");
        else if (positive)
            snprintf(out, n, "maintain v/s%s", fbit(b, 14) ? " (crossing)" : "");
        else
            snprintf(out, n, "monitor v/s");
    }
    else
    {
        // Multiple threats, each bit a separate constraint: 10 correct up,
        // 11 climb, 12 correct down, 13 descend, 14 crossing, 15 keep rate.
        snprintf(out, n, "multi-threat%s%s%s%s%s", fbit(b, 11) ? " climb" : "",
                 fbit(b, 13) ? " descend" : "",
                 fbit(b, 10) ? " correct up" : "", fbit(b, 12) ? " correct down" : "",
                 fbit(b, 14) ? " (crossing)" : "");
    }
    return out;
}

// ---------------------------------------------------------------------------
// Comm-B register inference. A DF20/21 does not say which register it
// carries, so each candidate is scored on whether its status bits and
// value ranges make sense (the way readsb and pyModeS do it) and the clear
// winner is decoded. Any field with its status bit clear must read zero;
// a value outside what an aircraft can do sinks the candidate outright.
// ---------------------------------------------------------------------------

static int bds_empty(const unsigned char *b)
{
    for (int i = 0; i < 7; i++)
        if (b[i]) return 0;
    return 56;
}

static int bds10(const unsigned char *b)
{
    return (b[0] == 0x10 && fbits(b, 10, 14) == 0) ? 56 : 0;
}

// Common-usage GICB capability report. Bits 25-56 are reserved; the rest
// is judged on which combinations of capabilities are seen in practice.
static int bds17(const unsigned char *b)
{
    if (fbits(b, 25, 56)) return 0;
    int score = fbit(b, 7) ? 1 : -2;                 // 2,0 is on nearly everything
    static const int unlikely[] = { 10, 11, 12, 13, 14, 20, 21, 22, 0 };   // waypoints, met reports
    for (int i = 0; unlikely[i]; i++)
        if (fbit(b, unlikely[i])) score -= 2;
    unsigned es = fbits(b, 1, 5);
    if (es == 0x1f)       score += 5 + fbit(b, 6);   // all ES registers: ADS-B out
    else if (es == 0 && !fbit(b, 6)) score += 1;     // none: Mode S only
    else                  score -= 12;               // partial ES support does not exist
    if (fbit(b, 16) && fbit(b, 24)) score += 2 + fbit(b, 9);   // 5,0 + 6,0, and 4,0 with them
    else if (!fbit(b, 16) && !fbit(b, 24) && !fbit(b, 9)) score += 1;
    else score -= 6;
    return score;
}

static int bds20(const unsigned char *b, struct mode_s_msg *mm, int store)
{
    char cs[9];
    if (b[0] != 0x20 || !decode_callsign(b + 1, cs)) return 0;
    if (store) memcpy(mm->flight, cs, sizeof(cs));
    return 8 + 6 * 8;
}

static int bds30(mode_s_t *self, const unsigned char *b, struct mode_s_msg *mm, int store)
{
    if (!acas_ra_plausible(self, b, mm->msgtype, 0)) return 0;
    if (store) mm->acas_ra_valid = 1;
    return 56;
}

// Selected vertical intention: MCP/FCU and FMS selected altitudes, the
// barometric setting, the vertical mode bits and the altitude source.
static int bds40(const unsigned char *b, struct mode_s_msg *mm, int store)
{
    int mcp_ok = fbit(b, 1), fms_ok = fbit(b, 14), qnh_ok = fbit(b, 27), mode_ok = fbit(b, 48), src_ok = fbit(b, 54);
    unsigned mcp = fbits(b, 2, 13), fms = fbits(b, 15, 26), qnh = fbits(b, 28, 39), mode = fbits(b, 49, 51), src = fbits(b, 55, 56);
    if (!mcp_ok && !fms_ok && !qnh_ok && !mode_ok && !src_ok) return 0;
    if (fbits(b, 40, 47) || fbits(b, 52, 53)) return 0;
    int score = 0;
    if (mcp_ok && mcp) { if (mcp * 16 < 1000 || mcp * 16 > 50000) return 0; score += 13; }
    else if (!mcp_ok && !mcp) score += 1;
    else return 0;
    if (fms_ok && fms) { if (fms * 16 < 1000 || fms * 16 > 50000) return 0; score += 13; }
    else if (!fms_ok && !fms) score += 1;
    else return 0;
    if (qnh_ok && qnh) { if (800 + qnh * 0.1 < 900 || 800 + qnh * 0.1 > 1100) return 0; score += 13; }
    else if (!qnh_ok && !qnh) score += 1;
    else return 0;
    if (mode_ok) score += 4; else if (!mode) score += 1; else return 0;
    if (src_ok) score += 3; else if (!src) score += 1; else return 0;
    if (mcp_ok && fms_ok && mcp != fms) score -= 4;
    // selected altitudes are almost always a multiple of 500 ft
    if (mcp_ok && (mcp * 16 % 500) >= 16 && (mcp * 16 % 500) <= 484) score -= 4;
    if (fms_ok && (fms * 16 % 500) >= 16 && (fms * 16 % 500) <= 484) score -= 4;
    if (store)
    {
        if (mcp_ok) { mm->nav_have |= MODE_S_NAV_HAVE_MCP; mm->nav_alt_mcp = mcp * 16; }
        if (fms_ok) { mm->nav_have |= MODE_S_NAV_HAVE_FMS; mm->nav_alt_fms = fms * 16; }
        if (qnh_ok) { mm->nav_have |= MODE_S_NAV_HAVE_QNH; mm->nav_qnh = 800 + qnh * 0.1f; }
        if (mode_ok)
        {
            mm->nav_have |= MODE_S_NAV_HAVE_MODES;
            mm->nav_modes = ((mode & 4) ? MODE_S_NAV_MODE_VNAV : 0) |
                            ((mode & 2) ? MODE_S_NAV_MODE_ALT_HOLD : 0) |
                            ((mode & 1) ? MODE_S_NAV_MODE_APPROACH : 0);
        }
    }
    return score;
}

// Track and turn report: roll, true track, ground speed, track rate, TAS.
static int bds50(const unsigned char *b, struct mode_s_msg *mm, int store)
{
    if (!fbit(b, 1) || !fbit(b, 12) || !fbit(b, 24) || !fbit(b, 46)) return 0;
    float roll = fbits(b, 3, 11) * 45.0f / 256 - (fbit(b, 2) ? 90 : 0);
    float track = fbits(b, 14, 23) * 90.0f / 512 + (fbit(b, 13) ? 180 : 0);
    unsigned gs = fbits(b, 25, 34) * 2, tas = fbits(b, 47, 56) * 2;
    int tr_ok = fbit(b, 35);
    float tr = fbits(b, 37, 45) * 8.0f / 256 - (fbit(b, 36) ? 16 : 0);
    if (roll < -40 || roll >= 40) return 0;
    if (!gs || gs < 50 || gs > 700) return 0;
    if (!tas || tas < 50 || tas > 700) return 0;
    int score = 11 + 12 + 11 + 11;
    if (tr_ok) { if (tr < -10 || tr > 10) return 0; score += 11; }
    else if (!fbits(b, 36, 45)) score += 1;
    else return 0;
    // a coordinated turn's rate follows from bank and TAS: g*tan(roll)/v
    if (tr_ok && fabs(68625 * tan(roll * M_PI / 180) / (tas * 20 * M_PI) - tr) > 2.0) score -= 6;
    if (store)
    {
        mm->roll_valid = 1;        mm->roll = roll;
        mm->heading_is_valid = 1;  mm->heading = (int)(track + 0.5) % 360;
        mm->gs_valid = 1;          mm->velocity = gs;
        mm->tas_valid = 1;         mm->tas = tas;
        if (tr_ok) { mm->track_rate_valid = 1; mm->track_rate = tr; }
    }
    return score;
}

// Heading and speed report: magnetic heading, IAS, Mach, barometric and
// inertial vertical rates.
static int bds60(const unsigned char *b, struct mode_s_msg *mm, int store)
{
    if (!fbit(b, 1) || !fbit(b, 13) || !fbit(b, 24) || (!fbit(b, 35) && !fbit(b, 46))) return 0;
    float hdg = fbits(b, 3, 12) * 90.0f / 512 + (fbit(b, 2) ? 180 : 0);
    unsigned ias = fbits(b, 14, 23);
    float mach = fbits(b, 25, 34) * 2.048f / 512;
    int br_ok = fbit(b, 35), ir_ok = fbit(b, 46);
    int br = fbits(b, 37, 45) * 32 - (fbit(b, 36) ? 16384 : 0);
    int ir = fbits(b, 48, 56) * 32 - (fbit(b, 47) ? 16384 : 0);
    if (!ias || ias < 50 || ias > 700) return 0;
    if (mach < 0.1f || mach > 0.9f) return 0;
    int score = 12 + 11 + 11;
    if (br_ok) { if (br < -6000 || br > 6000) return 0; score += 11; }
    else if (!fbits(b, 36, 45)) score += 1;
    else return 0;
    if (ir_ok) { if (ir < -6000 || ir > 6000) return 0; score += 11; }
    else if (!fbits(b, 47, 56)) score += 1;
    else return 0;
    if (br_ok && ir_ok && abs(br - ir) > 2000) score -= 12;
    if (store)
    {
        mm->mag_heading_valid = 1; mm->mag_heading = hdg;
        mm->ias_valid = 1;         mm->ias = ias;
        mm->mach_valid = 1;        mm->mach = mach;
        if (br_ok) { mm->baro_rate_valid = 1; mm->baro_rate = br; }
        if (ir_ok) { mm->geom_rate_valid = 1; mm->geom_rate = ir; }
    }
    return score;
}

static void decode_commb(mode_s_t *self, struct mode_s_msg *mm)
{
    const unsigned char *b = mm->msg + 4;
    struct { int bds; int score; } c[] = {
        { 0x00, bds_empty(b) },
        { 0x10, bds10(b) },
        { 0x17, bds17(b) },
        { 0x20, bds20(b, mm, 0) },
        { 0x30, bds30(self, b, mm, 0) },
        { 0x40, bds40(b, mm, 0) },
        { 0x50, bds50(b, mm, 0) },
        { 0x60, bds60(b, mm, 0) },
    };
    int best = 0, tie = 0, bds = 0;
    for (unsigned i = 0; i < sizeof(c) / sizeof(c[0]); i++)
    {
        if (c[i].score > best) { best = c[i].score; bds = c[i].bds; tie = 0; }
        else if (c[i].score == best && best) tie = 1;
    }
    if (!best || tie) return;
    mm->commb_bds = bds;
    switch (bds)
    {
    case 0x20: bds20(b, mm, 1); break;
    case 0x30: bds30(self, b, mm, 1); break;
    case 0x40: bds40(b, mm, 1); break;
    case 0x50: bds50(b, mm, 1); break;
    case 0x60: bds60(b, mm, 1); break;
    default: break;
    }
}

int mode_s_nic(int metype, int version, int nic_a, int nic_b, int nic_c)
{
    switch (metype)
    {
    case 5: case 9: case 20:  return 11;
    case 6: case 10: case 21: return 10;
    case 7:  return version == 2 ? (nic_a && !nic_c ? 9 : 8) : version == 1 ? (nic_a ? 9 : 8) : 8;
    case 8:  return version == 2 ? (nic_a && nic_c ? 7 : (nic_a || nic_c) ? 6 : 0) : 0;
    case 11: return version == 2 ? (nic_a && nic_b ? 9 : 8) : version == 1 ? (nic_a ? 9 : 8) : 8;
    case 12: return 7;
    case 13: return 6;
    case 14: return 5;
    case 15: return 4;
    case 16: return nic_a && nic_b ? 3 : 2;
    case 17: return 1;
    default: return 0;
    }
}

// TC29 target state and status. Subtype 0 is the version 1 layout
// (vertical/horizontal source + mode + target), subtype 1 the version 2 one
// (selected altitude, barometric setting, selected heading, mode bits).
// Both carry NACp / NICbaro / SIL.
static void decode_target_status(struct mode_s_msg *mm)
{
    const unsigned char *me = mm->msg + 4;
    mm->mesub = fbits(me, 6, 7);
    if (mm->mesub == 0 && !fbit(me, 11))
    {
        int vsrc = fbits(me, 8, 9);     // 1 MCP/FCU, 2 holding current altitude, 3 FMS
        int vmode = fbits(me, 14, 15);  // 1 acquiring, 2 maintaining
        int alt = -1000 + 100 * fbits(me, 16, 25);
        if (vsrc == 1) { mm->nav_have |= MODE_S_NAV_HAVE_MCP; mm->nav_alt_mcp = alt; }
        if (vsrc == 3) { mm->nav_have |= MODE_S_NAV_HAVE_FMS; mm->nav_alt_fms = alt; }
        if (vmode == 1 || vmode == 2)
        {
            mm->nav_have |= MODE_S_NAV_HAVE_MODES;
            mm->nav_modes |= vsrc == 3 ? MODE_S_NAV_MODE_VNAV
                           : (vmode == 2 && vsrc == 2) ? MODE_S_NAV_MODE_ALT_HOLD
                           : MODE_S_NAV_MODE_AUTOPILOT;
        }
        int hsrc = fbits(me, 26, 27);   // 1 MCP/FCU, 2 current track, 3 FMS
        if (hsrc)
        {
            mm->nav_have |= MODE_S_NAV_HAVE_HEADING;
            mm->nav_heading = fbits(me, 28, 36);
        }
        int hmode = fbits(me, 38, 39);
        if (hmode == 1 || hmode == 2)
        {
            mm->nav_have |= MODE_S_NAV_HAVE_MODES;
            mm->nav_modes |= hsrc == 3 ? MODE_S_NAV_MODE_LNAV : MODE_S_NAV_MODE_AUTOPILOT;
        }
        int tcas = fbits(me, 52, 53);   // 1 not operational, 2/3 operational
        if (tcas) mm->nav_have |= MODE_S_NAV_HAVE_MODES;
        if (tcas != 1) mm->nav_modes |= MODE_S_NAV_MODE_TCAS;
        mm->emergency_valid = 1;
        mm->emergency = fbits(me, 54, 56);
    }
    else if (mm->mesub == 1)
    {
        unsigned alt = fbits(me, 10, 20);
        if (alt)
        {
            if (fbit(me, 9)) { mm->nav_have |= MODE_S_NAV_HAVE_FMS; mm->nav_alt_fms = (alt - 1) * 32; }
            else             { mm->nav_have |= MODE_S_NAV_HAVE_MCP; mm->nav_alt_mcp = (alt - 1) * 32; }
        }
        unsigned qnh = fbits(me, 21, 29);
        if (qnh) { mm->nav_have |= MODE_S_NAV_HAVE_QNH; mm->nav_qnh = 800 + (qnh - 1) * 0.8f; }
        if (fbit(me, 30))
        {
            // two's complement -180..180, which reads the same as 0..360
            mm->nav_have |= MODE_S_NAV_HAVE_HEADING;
            mm->nav_heading = fbits(me, 31, 39) * 180.0f / 256;
        }
        if (fbit(me, 47))
        {
            mm->nav_have |= MODE_S_NAV_HAVE_MODES;
            mm->nav_modes = (fbit(me, 48) ? MODE_S_NAV_MODE_AUTOPILOT : 0) |
                            (fbit(me, 49) ? MODE_S_NAV_MODE_VNAV : 0) |
                            (fbit(me, 50) ? MODE_S_NAV_MODE_ALT_HOLD : 0) |
                            (fbit(me, 52) ? MODE_S_NAV_MODE_APPROACH : 0) |
                            (fbit(me, 53) ? MODE_S_NAV_MODE_TCAS : 0) |
                            (fbit(me, 54) ? MODE_S_NAV_MODE_LNAV : 0);
        }
    }
    else
        return;
    mm->nac_p_valid = 1;    mm->nac_p = fbits(me, 40, 43);
    mm->nic_baro_valid = 1; mm->nic_baro = fbit(me, 44);
    mm->sil_valid = 1;      mm->sil = fbits(me, 45, 46); mm->sil_type = 0;
}

// TC31 operational status, subtype 0 airborne / 1 surface. The layout of
// the capability and mode fields moved between versions 0, 1 and 2, so
// the version (bits 41-43, same place in all three) is read first.
static void decode_operational_status(struct mode_s_msg *mm)
{
    const unsigned char *me = mm->msg + 4;
    int surface = mm->mesub == 1;
    if (mm->mesub > 1) return;
    mm->opstatus_valid = 1;
    mm->version = fbits(me, 41, 43);
    switch (mm->version)
    {
    case 0:
        if (!surface && fbits(me, 9, 10) == 0) mm->cc_acas = !fbit(me, 12);
        break;
    case 1:
        if (fbits(me, 25, 26) == 0)
        {
            mm->om_acas_ra = fbit(me, 27);
            mm->om_ident   = fbit(me, 28);
        }
        if (fbits(me, 9, 10) == 0 && fbits(me, 13, 14) == 0)
        {
            if (!surface) mm->cc_acas = !fbit(me, 11);
            else { mm->nac_v_valid = 1; mm->nac_v = fbits(me, 17, 19); mm->nic_c = fbit(me, 20); }
        }
        mm->nic_a = fbit(me, 44);
        mm->nac_p_valid = 1; mm->nac_p = fbits(me, 45, 48);
        mm->sil_valid = 1;   mm->sil = fbits(me, 51, 52); mm->sil_type = 0;
        if (!surface) { mm->nic_baro_valid = 1; mm->nic_baro = fbit(me, 53); }
        break;
    case 2:
        if (fbits(me, 25, 26) == 0)
        {
            mm->om_acas_ra = fbit(me, 27);
            mm->om_ident   = fbit(me, 28);
            mm->sda_valid = 1; mm->sda = fbits(me, 31, 32);
        }
        if (fbits(me, 9, 10) == 0)
        {
            if (!surface) mm->cc_acas = fbit(me, 11);   // sense inverted from v0/v1
            else { mm->nac_v_valid = 1; mm->nac_v = fbits(me, 17, 19); mm->nic_c = fbit(me, 20); }
        }
        mm->nic_a = fbit(me, 44);
        mm->nac_p_valid = 1; mm->nac_p = fbits(me, 45, 48);
        mm->sil_valid = 1;   mm->sil = fbits(me, 51, 52); mm->sil_type = fbit(me, 55) ? 2 : 1;
        if (!surface)
        {
            mm->gva_valid = 1;      mm->gva = fbits(me, 49, 50);
            mm->nic_baro_valid = 1; mm->nic_baro = fbit(me, 53);
        }
        break;
    default:
        // versions 3-7 are unassigned; nothing else in the message is safe to read
        mm->opstatus_valid = 0;
        break;
    }
}

// Decode a raw Mode S message demodulated as a stream of bytes by
// mode_s_detect(), and split it into fields populating a mode_s_msg structure.
void mode_s_decode(mode_s_t *self, struct mode_s_msg *mm, unsigned char *msg)
{
    uint32_t crc2; // Computed CRC, used to verify the message CRC.

    /* Only the fields carried by this particular message type get assigned
     * below. Callers test them for non-zero to decide what's present, so a
     * reused stack mm must start clean -- otherwise altitude/velocity/heading
     * pick up whatever the task stack last held (0x20202020 from tui_draw()'s
     * space-filled radar panel, in practice) and read as valid data. */
    memset(mm, 0, sizeof(*mm));

    // Work on our local copy
    memcpy(mm->msg, msg, MODE_S_LONG_MSG_BYTES);
    msg = mm->msg;

    // Get the message type ASAP as other operations depend on this
    mm->msgtype = msg[0] >> 3; // Downlink Format

    // A DF field one bit away from 17 (1/16/19/21/25) may be a DF17 with the
    // error in its first byte -- the ES is the one format with a zero
    // expected CRC, so patching the DF and rechecking is a cheap, safe test
    // that the single-bit fixer below never gets to run for these types.
    mm->errorbit = -1; // No error
    if (self->fix_errors &&
        (mm->msgtype == 1 || mm->msgtype == 16 || mm->msgtype == 19 ||
         mm->msgtype == 21 || mm->msgtype == 25))
    {
        unsigned char orig = msg[0];
        msg[0] = (orig & 7) | (17 << 3);
        if (mode_s_checksum(msg, MODE_S_LONG_MSG_BITS) ==
            (((uint32_t)msg[11] << 16) | ((uint32_t)msg[12] << 8) | msg[13]))
        {
            mm->msgtype  = 17;
            mm->errorbit = __builtin_ctz((orig ^ msg[0]) & 0xf8) ^ 7;
        }
        else
            msg[0] = orig;
    }
    mm->msgbits = mode_s_msg_len_by_type(mm->msgtype);

    // CRC is always the last three bytes.
    mm->crc = ((uint32_t)msg[(mm->msgbits / 8) - 3] << 16) |
              ((uint32_t)msg[(mm->msgbits / 8) - 2] << 8) |
              (uint32_t)msg[(mm->msgbits / 8) - 1];
    crc2 = mode_s_checksum(msg, mm->msgbits);

    // Check CRC and fix single bit errors using the CRC when possible (DF 11 and 17).
    mm->crcok = (mm->crc == crc2);

    // DF11 answering a ground interrogation carries PI = CRC xor the
    // interrogator code (II 1-15 / SI 16-79) in the low 7 bits; the squitter
    // form has II 0. A residual that small is that code, not a bit error --
    // accepted when the address is one already seen, as readsb does.
    if (!mm->crcok && mm->msgtype == 11 && ((mm->crc ^ crc2) & 0xffff80) == 0)
    {
        uint32_t addr = ((uint32_t)msg[1] << 16) | ((uint32_t)msg[2] << 8) | msg[3];
        if (icao_addr_was_recently_seen(self, addr))
        {
            mm->iid   = (mm->crc ^ crc2) & 0x7f;
            mm->crcok = 1;
        }
    }
    else if (!mm->crcok && self->fix_errors &&
        (mm->msgtype == 11 || mm->msgtype == 17 || mm->msgtype == 18))
    {
        if ((mm->errorbit = fix_single_bit_errors(msg, mm->msgbits)) != -1)
        {
            mm->crc = mode_s_checksum(msg, mm->msgbits);
            mm->crcok = 1;
        }
        else if (self->aggressive && mm->msgtype == 17 &&
                 (mm->errorbit = fix_two_bits_errors(msg, mm->msgbits)) != -1)
        {
            mm->crc = mode_s_checksum(msg, mm->msgbits);
            mm->crcok = 1;
        }
    }

    // Note that most of the other computation happens *after* we fix the
    // single bit errors, otherwise we would need to recompute the fields
    // again.
    mm->ca = msg[0] & 7; // Responder capabilities.

    // ICAO address
    mm->aa1 = msg[1];
    mm->aa2 = msg[2];
    mm->aa3 = msg[3];

    // DF 17 type (assuming this is a DF17, otherwise not used)
    mm->metype = msg[4] >> 3; // Extended squitter message type.
    mm->mesub = msg[4] & 7;   // Extended squitter message subtype.

    // Fields for DF4,5,20,21
    mm->fs = msg[0] & 7;           // Flight status for DF4,5,20,21
    mm->dr = msg[1] >> 3 & 31;     // Request extraction of downlink request.
    mm->um = ((msg[1] & 7) << 3) | // Request extraction of downlink request.
             msg[2] >> 5;

    // Squawk from message bits 20-32; meaningful for DF5/21 only.
    mm->identity = decode_id13_field(((msg[2] & 0x1f) << 8) | msg[3]);
    mm->squawk_valid = mm->msgtype == 5 || mm->msgtype == 21;

    // Air/ground and alert/SPI status. FS (DF4/5/20/21): 0 airborne,
    // 1 ground, 2 alert airborne, 3 alert ground, 4 alert+SPI, 5 SPI --
    // the "airborne" of 0/2/4/5 is not trusted (readsb: many transponders
    // never report ground), only the explicit ground states are. VS
    // (DF0/16) bit 6 set = ground. CA (DF11/17): 4 ground, 5 airborne.
    if (mm->msgtype == 4 || mm->msgtype == 5 || mm->msgtype == 20 || mm->msgtype == 21)
    {
        if (mm->fs <= 5)
        {
            mm->alert_valid = mm->spi_valid = 1;
            mm->alert = mm->fs >= 2 && mm->fs <= 4;
            mm->spi   = mm->fs >= 4;
            if (mm->fs == 1 || mm->fs == 3)
                mm->airground = 1;
        }
    }
    else if (mm->msgtype == 0 || mm->msgtype == 16)
    {
        if (msg[0] & 0x04)
            mm->airground = 1;
    }
    else if (mm->msgtype == 11 || mm->msgtype == 17)
    {
        if (mm->ca == 4) mm->airground = 1;
        else if (mm->ca == 5) mm->airground = 2;
    }

    // DF 11 & 17: try to populate our ICAO addresses whitelist. DFs with an AP
    // field (xored addr and crc), try to decode it. DF18 shares 17's CRC
    // scheme but is kept out of the whitelist: a TIS-B/ADS-R source has no
    // Mode S transponder, so no AP-addressed reply will ever come from it.
    if (mm->msgtype != 11 && mm->msgtype != 17 && mm->msgtype != 18)
    {
        // Check if we can check the checksum for the Downlink Formats where
        // the checksum is xored with the aircraft ICAO address. We try to
        // brute force it using a list of recently seen aircraft addresses.
        if (brute_force_ap(self, msg, mm))
        {
            // We recovered the message, mark the checksum as valid.
            mm->crcok = 1;
        }
        else
        {
            mm->crcok = 0;
        }
    }
    else
    {
        // If this is DF 11 or DF 17 and the checksum was ok, we can add this
        // address to the list of recently seen addresses.
        if (mm->crcok && mm->errorbit == -1 && mm->msgtype != 18 && mm->iid == 0)
        {
            uint32_t addr = (mm->aa1 << 16) | (mm->aa2 << 8) | mm->aa3;
            add_recently_seen_icao_addr(self, addr);
        }
    }

    // Decode 13 bit altitude for DF0, DF4, DF16, DF20
    if (mm->msgtype == 0 || mm->msgtype == 4 ||
        mm->msgtype == 16 || mm->msgtype == 20)
    {
        mm->altitude = decode_ac13_field(msg, &mm->unit);
    }

    // ACAS RA in the DF16 MV field; the Comm-B register of a DF20/21 is
    // inferred from its content. A reply with DR/UM set is almost always
    // noise (nothing uses the multisite protocol), so it is not decoded.
    if (mm->crcok && mm->msgtype == 16)
        mm->acas_ra_valid = acas_ra_plausible(self, msg + 4, 16, 0);
    else if (mm->crcok && (mm->msgtype == 20 || mm->msgtype == 21) &&
             mm->dr == 0 && mm->um == 0 && mm->errorbit == -1)
        decode_commb(self, mm);

    // Decode extended squitter specific stuff.
    if (mm->msgtype == 17)
    {
        // Decode the extended squitter message.

        if (mm->metype >= 1 && mm->metype <= 4)
        {
            // Aircraft Identification and Category. TC4 is set A, TC1 set D.
            mm->aircraft_type = mm->metype - 1;
            mm->category = ((0x0e - mm->metype) << 4) | mm->mesub;
            decode_callsign(msg + 5, mm->flight);
        }
        else if (mm->metype == 0 ||
                 (mm->metype >= 9 && mm->metype <= 18) ||
                 (mm->metype >= 20 && mm->metype <= 22))
        {
            // Airborne position: TC9-18 barometric altitude, TC20-22
            // geometric (HAE), TC0 altitude only with no position.
            mm->altitude = decode_ac12_field(msg, &mm->unit);
            mm->alt_geom = mm->metype >= 20;
            mm->airground = 2;
            // Surveillance status: 0 none, 1 permanent alert (emergency
            // squawk), 2 temporary alert (squawk changed), 3 SPI. 1/2 win
            // over 3 in the encoding, so SPI is unknown while they are set.
            int ss = (msg[4] >> 1) & 3;
            mm->alert_valid = 1;
            mm->alert = ss == 1 || ss == 2;
            if (ss == 0 || ss == 3)
            {
                mm->spi_valid = 1;
                mm->spi = ss == 3;
            }
            // ME bit 8: NIC supplement B (version 2; IMF on a DF18)
            mm->nic_b_valid = 1;
            mm->nic_b = msg[4] & 1;
            if (mm->metype != 0)
            {
                mm->fflag = msg[6] & (1 << 2);
                mm->tflag = msg[6] & (1 << 3);
                mm->raw_latitude = ((msg[6] & 3) << 15) |
                                   (msg[7] << 7) |
                                   (msg[8] >> 1);
                mm->raw_longitude = ((msg[8] & 1) << 16) |
                                    (msg[9] << 8) |
                                    msg[10];
                /* A known transponder failure mode (readsb filters the same
                 * signature): TC15 with zero altitude, zero longitude and a
                 * latitude ending in 12 zero bits is not a position. */
                int ac12 = (msg[5] << 4) | (msg[6] >> 4);
                mm->cpr_valid = !(mm->metype == 15 && ac12 == 0 &&
                                  mm->raw_longitude == 0 &&
                                  (mm->raw_latitude & 0x0fff) == 0);
            }
        }
        else if (mm->metype >= 5 && mm->metype <= 8)
        {
            // Surface position: no altitude; movement and ground track
            // instead, and CPR in quarter-size cells (see cpr.h).
            int movement = ((msg[4] & 7) << 4) | (msg[5] >> 4);
            if (movement >= 1 && movement <= 124)
            {
                mm->velocity = (int)(decode_movement(movement) + 0.5);
                mm->gs_valid = 1;
            }
            if (msg[5] & 0x08)
            {
                mm->heading_is_valid = 1;
                mm->heading = (int)((((msg[5] & 7) << 4) | (msg[6] >> 4)) * 360.0 / 128 + 0.5) % 360;
            }
            mm->fflag = msg[6] & (1 << 2);
            mm->tflag = msg[6] & (1 << 3);
            mm->raw_latitude = ((msg[6] & 3) << 15) |
                               (msg[7] << 7) |
                               (msg[8] >> 1);
            mm->raw_longitude = ((msg[8] & 1) << 16) |
                                (msg[9] << 8) |
                                msg[10];
            mm->cpr_valid = 1;
            mm->cpr_surface = 1;
            mm->airground = 1;
        }
        else if (mm->metype == 19 && mm->mesub >= 1 && mm->mesub <= 4)
        {
            // Airborne Velocity Message
            if (mm->mesub == 1 || mm->mesub == 2)
            {
                // Ground speed as E/W and N/S components. The 10-bit fields
                // are biased by one (0 = no data); subtype 2 is the
                // supersonic encoding in 4 kt steps.
                int ew_raw = ((msg[5] & 3) << 8) | msg[6];
                int ns_raw = ((msg[7] & 0x7f) << 3) | ((msg[8] & 0xe0) >> 5);
                mm->ew_dir = (msg[5] & 4) >> 2;
                mm->ns_dir = (msg[7] & 0x80) >> 7;
                if (ew_raw && ns_raw)
                {
                    int scale = mm->mesub == 2 ? 4 : 1;
                    mm->ew_velocity = (ew_raw - 1) * scale;
                    mm->ns_velocity = (ns_raw - 1) * scale;
                    mm->velocity = (int)(sqrt((double)mm->ns_velocity * mm->ns_velocity +
                                              (double)mm->ew_velocity * mm->ew_velocity) + 0.5);
                    mm->gs_valid = 1;
                }
                if (mm->velocity)
                {
                    int ewv = mm->ew_velocity;
                    int nsv = mm->ns_velocity;
                    double heading;

                    if (mm->ew_dir)
                        ewv *= -1;
                    if (mm->ns_dir)
                        nsv *= -1;
                    heading = atan2(ewv, nsv);

                    // Convert to degrees.
                    mm->heading = heading * 360 / (M_PI * 2);
                    // We don't want negative values but a 0-360 scale.
                    if (mm->heading < 0)
                        mm->heading += 360;
                    /* dump1090 leaves this clear here and just reads heading
                     * unconditionally at display time; callers that gate on
                     * the flag (ours does) would otherwise never see a
                     * heading, since sub 1/2 is the common airborne case. */
                    mm->heading_is_valid = 1;
                }
            }
            else if (mm->mesub == 3 || mm->mesub == 4)
            {
                // Airspeed subtypes (GNSS velocity unavailable): magnetic or
                // true heading, then IAS or TAS -- 10 bits biased by one,
                // subtype 4 in 4 kt steps like the supersonic ground speed.
                mm->heading_is_valid = msg[5] & (1 << 2);
                mm->heading = (int)((((msg[5] & 3) << 8) | msg[6]) * 360.0 / 1024 + 0.5) % 360;
                int as_raw = ((msg[7] & 0x7f) << 3) | (msg[8] >> 5);
                if (as_raw)
                {
                    int kt = (as_raw - 1) * (mm->mesub == 4 ? 4 : 1);
                    if (msg[7] & 0x80) { mm->tas_valid = 1; mm->tas = kt; }
                    else               { mm->ias_valid = 1; mm->ias = kt; }
                }
            }
            // ME bits 11-13: NACv
            mm->nac_v_valid = 1;
            mm->nac_v = (msg[5] >> 3) & 7;
            // Vertical rate sits at the same bits in all four subtypes:
            // bit 36 source (1 = barometric), 37 sign, 38-46 in 64 ft/min
            // steps biased by one.
            mm->vert_rate_source = (msg[8] & 0x10) >> 4;
            mm->vert_rate_sign = (msg[8] & 0x8) >> 3;
            mm->vert_rate = ((msg[8] & 7) << 6) | ((msg[9] & 0xfc) >> 2);
            if (mm->vert_rate)
            {
                int fpm = (mm->vert_rate - 1) * (mm->vert_rate_sign ? -64 : 64);
                if (mm->vert_rate_source) { mm->baro_rate_valid = 1; mm->baro_rate = fpm; }
                else                      { mm->geom_rate_valid = 1; mm->geom_rate = fpm; }
            }
            // Bits 49-56: geometric minus barometric altitude, 25 ft steps
            // biased by one, bit 49 the sign.
            int delta = msg[10] & 0x7f;
            if (delta)
            {
                mm->geom_delta_valid = 1;
                mm->geom_delta = (delta - 1) * ((msg[10] & 0x80) ? -25 : 25);
            }
        }
        else if (mm->metype == 28 && mm->mesub == 1)
        {
            // Aircraft status, emergency/priority: ME bits 9-11 are the
            // state, 12-24 the squawk -- the only place a squawk is
            // broadcast without a ground interrogation.
            mm->emergency_valid = 1;
            mm->emergency = msg[5] >> 5;
            mm->identity = decode_id13_field(((msg[5] & 0x1f) << 8) | msg[6]);
            mm->squawk_valid = 1;
        }
        else if (mm->metype == 28 && mm->mesub == 2)
        {
            // ACAS RA broadcast: the ME carries the BDS 3,0 register.
            mm->acas_ra_valid = acas_ra_plausible(self, msg + 4, 17, 1);
        }
        else if (mm->metype == 23 && mm->mesub == 7)
        {
            // Test message, national use: a squawk in ME bits 9-21.
            int id13 = (msg[5] << 5) | (msg[6] >> 3);
            if (id13)
            {
                mm->identity = decode_id13_field(id13);
                mm->squawk_valid = 1;
            }
        }
        else if (mm->metype == 29)
        {
            decode_target_status(mm);
        }
        else if (mm->metype == 31)
        {
            decode_operational_status(mm);
        }
    }
    mm->phase_corrected = 0; // Set to 1 by the caller if needed.
}

// Turn I/Q samples pointed by `data` into the magnitude vector pointed by `mag`
void mode_s_compute_magnitude_vector(unsigned char *data, uint16_t *mag, uint32_t size)
{
    uint32_t j;

    // Compute the magnitude vector. It's just SQRT(I^2 + Q^2), but we rescale
    // to the 0-255 range to exploit the full resolution.
    for (j = 0; j < size; j += 2)
    {
        int i = data[j] - 127;
        int q = data[j + 1] - 127;

        if (i < 0)
            i = -i;
        if (q < 0)
            q = -q;
        mag[j / 2] = maglut[i * 129 + q];
    }
}

// Return -1 if the message is out of fase left-side
// Return  1 if the message is out of fase right-size
// Return  0 if the message is not particularly out of phase.
//
// Note: this function will access mag[-1], so the caller should make sure to
// call it only if we are not at the start of the current buffer.
int detect_out_of_phase(uint16_t *mag)
{
    if (mag[3] > mag[2] / 3)
        return 1;
    if (mag[10] > mag[9] / 3)
        return 1;
    if (mag[6] > mag[7] / 3)
        return -1;
    if (mag[-1] > mag[1] / 3)
        return -1;
    return 0;
}

// This function does not really correct the phase of the message, it just
// applies a transformation to the first sample representing a given bit:
//
// If the previous bit was one, we amplify it a bit.
// If the previous bit was zero, we decrease it a bit.
//
// This simple transformation makes the message a bit more likely to be
// correctly decoded for out of phase messages:
//
// When messages are out of phase there is more uncertainty in sequences of the
// same bit multiple times, since 11111 will be transmitted as continuously
// altering magnitude (high, low, high, low...)
//
// However because the message is out of phase some part of the high is mixed
// in the low part, so that it is hard to distinguish if it is a zero or a one.
//
// However when the message is out of phase passing from 0 to 1 or from 1 to 0
// happens in a very recognizable way, for instance in the 0 -> 1 transition,
// magnitude goes low, high, high, low, and one of of the two middle samples
// the high will be *very* high as part of the previous or next high signal
// will be mixed there.
//
// Applying our simple transformation we make more likely if the current bit is
// a zero, to detect another zero. Symmetrically if it is a one it will be more
// likely to detect a one because of the transformation. In this way similar
// levels will be interpreted more likely in the correct way.
void apply_phase_correction(uint16_t *mag)
{
    int j;

    mag += 16; // Skip preamble.
    for (j = 0; j < (MODE_S_LONG_MSG_BITS - 1) * 2; j += 2)
    {
        if (mag[j] > mag[j + 1])
        {
            // One
            mag[j + 2] = (mag[j + 2] * 5) / 4;
        }
        else
        {
            // Zero
            mag[j + 2] = (mag[j + 2] * 4) / 5;
        }
    }
}

// Detect a Mode S messages inside the magnitude buffer pointed by 'mag' and of
// size 'maglen' bytes. Every detected Mode S message is convert it into a
// stream of bits and passed to the function to display it.
void mode_s_detect(mode_s_t *self, uint16_t *mag, uint32_t maglen, uint64_t base_ts_us, mode_s_callback_t cb)
{
    unsigned char bits[MODE_S_LONG_MSG_BITS];
    unsigned char msg[MODE_S_LONG_MSG_BITS / 2];
    uint16_t aux[MODE_S_LONG_MSG_BITS * 2];
    uint32_t j;
    int use_correction = 0;

    // The Mode S preamble is made of impulses of 0.5 microseconds at the
    // following time offsets:
    //
    // 0   - 0.5 usec: first impulse.
    // 1.0 - 1.5 usec: second impulse.
    // 3.5 - 4   usec: third impulse.
    // 4.5 - 5   usec: last impulse.
    //
    // Since we are sampling at 2 Mhz every sample in our magnitude vector is
    // 0.5 usec, so the preamble will look like this, assuming there is an
    // impulse at offset 0 in the array:
    //
    // 0   -----------------
    // 1   -
    // 2   ------------------
    // 3   --
    // 4   -
    // 5   --
    // 6   -
    // 7   ------------------
    // 8   --
    // 9   -------------------
    for (j = 0; j < maglen - MODE_S_FULL_LEN * 2; j++)
    {
        int low, high, delta, i, errors;
        int good_message = 0;
        uint32_t msg_start = j; // sample offset of the preamble's first impulse

        if (use_correction)
            goto good_preamble; // We already checked it.

        // First check of relations between the first 10 samples representing a
        // valid preamble. We don't even investigate further if this simple
        // test is not passed.
        if (!(mag[j] > mag[j + 1] &&
              mag[j + 1] < mag[j + 2] &&
              mag[j + 2] > mag[j + 3] &&
              mag[j + 3] < mag[j] &&
              mag[j + 4] < mag[j] &&
              mag[j + 5] < mag[j] &&
              mag[j + 6] < mag[j] &&
              mag[j + 7] > mag[j + 8] &&
              mag[j + 8] < mag[j + 9] &&
              mag[j + 9] > mag[j + 6]))
        {
            continue;
        }

        // The samples between the two spikes must be < than the average of the
        // high spikes level. We don't test bits too near to the high levels as
        // signals can be out of phase so part of the energy can be in the near
        // samples.
        high = (mag[j] + mag[j + 2] + mag[j + 7] + mag[j + 9]) / 6;
        if (mag[j + 4] >= high ||
            mag[j + 5] >= high)
        {
            continue;
        }

        // Similarly samples in the range 11-14 must be low, as it is the space
        // between the preamble and real data. Again we don't test bits too
        // near to high levels, see above.
        if (mag[j + 11] >= high ||
            mag[j + 12] >= high ||
            mag[j + 13] >= high ||
            mag[j + 14] >= high)
        {
            continue;
        }

    good_preamble:
        // If the previous attempt with this message failed, retry using
        // magnitude correction.
        if (use_correction)
        {
            memcpy(aux, mag + j + MODE_S_PREAMBLE_US * 2, sizeof(aux));
            if (j && detect_out_of_phase(mag + j))
            {
                apply_phase_correction(mag + j);
            }
            // TODO ... apply other kind of corrections.
        }

        // Decode all the next 112 bits, regardless of the actual message size.
        // We'll check the actual message type later.
        errors = 0;
        for (i = 0; i < MODE_S_LONG_MSG_BITS * 2; i += 2)
        {
            low = mag[j + i + MODE_S_PREAMBLE_US * 2];
            high = mag[j + i + MODE_S_PREAMBLE_US * 2 + 1];
            delta = low - high;
            if (delta < 0)
                delta = -delta;

            if (i > 0 && delta < 256)
            {
                bits[i / 2] = bits[i / 2 - 1];
            }
            else if (low == high)
            {
                // Checking if two adiacent samples have the same magnitude is
                // an effective way to detect if it's just random noise that
                // was detected as a valid preamble.
                bits[i / 2] = 2; // error
                if (i < MODE_S_SHORT_MSG_BITS * 2)
                    errors++;
            }
            else if (low > high)
            {
                bits[i / 2] = 1;
            }
            else
            {
                // (low < high) for exclusion
                bits[i / 2] = 0;
            }
        }

        // Restore the original message if we used magnitude correction.
        if (use_correction)
            memcpy(mag + j + MODE_S_PREAMBLE_US * 2, aux, sizeof(aux));

        // Pack bits into bytes
        for (i = 0; i < MODE_S_LONG_MSG_BITS; i += 8)
        {
            msg[i / 8] =
                bits[i] << 7 |
                bits[i + 1] << 6 |
                bits[i + 2] << 5 |
                bits[i + 3] << 4 |
                bits[i + 4] << 3 |
                bits[i + 5] << 2 |
                bits[i + 6] << 1 |
                bits[i + 7];
        }

        int msgtype = msg[0] >> 3;
        int msglen = mode_s_msg_len_by_type(msgtype) / 8;

        // Last check, high and low bits are different enough in magnitude to
        // mark this as real message and not just noise?
        delta = 0;
        for (i = 0; i < msglen * 8 * 2; i += 2)
        {
            delta += abs(mag[j + i + MODE_S_PREAMBLE_US * 2] -
                         mag[j + i + MODE_S_PREAMBLE_US * 2 + 1]);
        }
        delta /= msglen * 4;

        // Filter for an average delta of three is small enough to let almost
        // every kind of message to pass, but high enough to filter some random
        // noise.
        if (delta < 10 * 255)
        {
            use_correction = 0;
            continue;
        }

        // If we reached this point, and error is zero, we are very likely with
        // a Mode S message in our hands, but it may still be broken and CRC
        // may not be correct. This is handled by the next layer.
        if (errors == 0 || (self->aggressive && errors < 3))
        {
            struct mode_s_msg mm;

            // Decode the received message
            mode_s_decode(self, &mm, msg);

            // 2 MSPS => 6 ticks/sample at the 12 MHz Beast clock; delta is
            // still the average bit-slicing magnitude from the noise filter
            // above, cheap to repurpose as a coarse relative signal level.
            mm.timestamp_12mhz = base_ts_us * 12ULL + (uint64_t)msg_start * 6ULL;
            mm.signal_level    = delta > 0xFF00 ? 255 : (delta >> 8);

            // Skip this message if we are sure it's fine.
            if (mm.crcok)
            {
                j += (MODE_S_PREAMBLE_US + mm.msgbits) * 2;
                good_message = 1;
                if (use_correction)
                    mm.phase_corrected = 1;
                self->stat_goodcrc++;
                if (mm.errorbit != -1)
                    self->stat_fixed++;
            }
            else if (use_correction)
            {
                // A failed candidate is always retried once with phase
                // correction, so only the retry's failure is final -- counting
                // the first pass as well would double every bad frame.
                self->stat_badcrc++;
            }

            // Pass data to the next layer
            if (self->check_crc == 0 || mm.crcok)
            {
                cb(self, &mm);
            }
        }

        // Retry with phase correction if possible.
        if (!good_message && !use_correction)
        {
            j--;
            use_correction = 1;
        }
        else
        {
            use_correction = 0;
        }
    }
}