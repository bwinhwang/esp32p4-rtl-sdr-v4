#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <sys/time.h>

#define MODE_S_ICAO_CACHE_LEN 1024 // Power of two required
#define MODE_S_LONG_MSG_BYTES (112 / 8)
#define MODE_S_UNIT_FEET 0
#define MODE_S_UNIT_METERS 1

// Program state
typedef struct
{
    // Internal state
    // Recently seen ICAO addresses cache: an (addr, timestamp) pair per slot,
    // indexed [h*2] / [h*2+1] with h < MODE_S_ICAO_CACHE_LEN. dump1090 sizes
    // it as a malloc() byte count, so the sizeof() belonged there, not in an
    // array bound -- keeping it made the array 4x the addressable range.
    uint32_t icao_cache[MODE_S_ICAO_CACHE_LEN * 2];

    // Configuration
    int fix_errors; // Single bit error correction if true
    int aggressive; // Aggressive detection algorithm
    int check_crc;  // Only display messages with good CRC

    // Demodulator statistics. Kept here because check_crc gates the callback on
    // a passing CRC -- a failed frame never reaches the caller, so there is no
    // other place from which a signal-quality figure can be derived.
    unsigned long stat_goodcrc; // CRC passed, error-corrected frames included
    unsigned long stat_badcrc;  // preamble looked real, CRC never passed
    unsigned long stat_fixed;   // subset of stat_goodcrc rescued by fix_errors
} mode_s_t;

// The struct we use to store information about a decoded message
struct mode_s_msg
{
    // Generic fields
    unsigned char msg[MODE_S_LONG_MSG_BYTES]; // Binary message
    int msgbits;                              // Number of bits in message
    int msgtype;                              // Downlink format #
    int crcok;                                // True if CRC was valid
    uint32_t crc;                             // Message CRC
    int errorbit;                             // Bit corrected. -1 if no bit corrected.
    int aa1, aa2, aa3;                        // ICAO Address bytes 1 2 and 3
    int phase_corrected;                      // True if phase correction was applied.

    // DF 11
    int ca; // Responder capabilities.

    // DF 17
    int metype; // Extended squitter message type.
    int mesub;  // Extended squitter message subtype.
    int heading_is_valid;
    int heading;
    int aircraft_type;
    int fflag;            // 1 = Odd, 0 = Even CPR message.
    int tflag;            // UTC synchronized?
    int raw_latitude;     // Non decoded latitude
    int raw_longitude;    // Non decoded longitude
    char flight[9];       // 8 chars flight number.
    int ew_dir;           // 0 = East, 1 = West.
    int ew_velocity;      // E/W velocity.
    int ns_dir;           // 0 = North, 1 = South.
    int ns_velocity;      // N/S velocity.
    int vert_rate_source; // Vertical rate source.
    int vert_rate_sign;   // Vertical rate sign.
    int vert_rate;        // Vertical rate.
    int velocity;         // Ground speed, kt: from EW/NS (TC19) or movement (TC5-8).
    int gs_valid;         // velocity carries a value, including a real 0
    int cpr_valid;        // raw_latitude/raw_longitude carry a position
    int cpr_surface;      // ...in the surface encoding (TC5-8, on the ground)
    int alt_geom;         // altitude is geometric (TC20-22), not barometric
    int emergency_valid;  // TC28 subtype 1 seen; `identity` is its squawk.
    int emergency;        // 0 none, 1 general, 2 lifeguard, 3 min fuel,
                          // 4 no comms, 5 unlawful interference, 6 downed.
    int category;         // TC1-4 emitter category, readsb's byte form:
                          // 0xA0..0xD7 (A0 = "no information"); 0 = not carried.
    int acas_ra_valid;    // msg+4 holds a plausible ACAS resolution advisory
                          // (BDS 3,0): DF16 MV, TC28 sub 2 ME or DF20/21 MB.

    // DF11: interrogator code recovered from the PI residual; 0 for a
    // squitter. Non-zero means the address was only accepted because it
    // was recently seen, so it must not seed the cache.
    int iid;

    // Comm-B (DF20/21 MB): the register is not named in the reply, so it
    // is inferred from the content -- 0x10 0x17 0x20 0x30 0x40 0x50 0x60,
    // 0 when nothing fits or two fit equally.
    int commb_bds;

    // Intent (TC29, BDS 4,0). The nav_have bits say which are set.
    int nav_have;
    int nav_alt_mcp;      // ft
    int nav_alt_fms;      // ft
    float nav_qnh;        // hPa
    float nav_heading;    // deg
    int nav_modes;        // MODE_S_NAV_MODE_* bits, meaningful with NAV_HAVE_MODES

    // Speeds and rates: TC19 (all subtypes), BDS 5,0, BDS 6,0. The
    // vertical rate carries its source with it; dump1090's raw vert_rate
    // fields above are still filled for TC19.
    int ias_valid, ias;             // kt
    int tas_valid, tas;             // kt
    int mach_valid; float mach;
    int roll_valid; float roll;                 // deg, right positive
    int track_rate_valid; float track_rate;     // deg/s, right positive
    int mag_heading_valid; float mag_heading;   // BDS 6,0
    int baro_rate_valid, baro_rate;             // ft/min
    int geom_rate_valid, geom_rate;             // ft/min; BDS 6,0 inertial counts as geometric
    int geom_delta_valid, geom_delta;           // ft, geometric minus barometric (TC19)

    // Accuracy and operational status (TC9-18 NIC-B, TC19 NACv, TC29, TC31).
    int nic_b_valid, nic_b;     // per message, TC9-18
    int nac_v_valid, nac_v;
    int nac_p_valid, nac_p;
    int sil_valid, sil, sil_type;   // sil_type: 0 unknown, 1 per hour, 2 per sample
    int nic_baro_valid, nic_baro;
    int gva_valid, gva;
    int sda_valid, sda;
    int opstatus_valid;         // the fields below are set
    int version;                // ADS-B version 0/1/2
    int nic_a, nic_c;
    int om_ident;               // IDENT switch active (another SPI source)
    int om_acas_ra;             // RA active
    int cc_acas;                // ACAS operational

    // Status shared by the surveillance replies and the ES position messages.
    // airground: 0 no information, 1 on the ground, 2 airborne. Only the
    // two certain values are reported; FS 0/2 and CA 6 say nothing (many
    // transponders send them regardless), so they stay 0.
    int airground;
    int alert_valid, alert; // squawk changed recently, or an emergency squawk
    int spi_valid, spi;     // pilot pressed IDENT

    // DF4, DF5, DF20, DF21
    int fs;       // Flight status for DF4,5,20,21
    int dr;       // Request extraction of downlink request.
    int um;       // Request extraction of downlink request.
    int identity; // 13 bits identity (Squawk).
    int squawk_valid; // identity is a squawk: DF5/21, TC28 sub 1, TC23 sub 7

    // Fields used by multiple message types.
    int altitude, unit;

    // Beast-format fields, filled in mode_s_detect() at detection time --
    // dump1090's original struct carried neither. timestamp_12mhz is a
    // receiver-local monotonic 12 MHz tick count derived from the caller's
    // buffer timestamp plus this message's sample offset; it is NOT GPS/PPS
    // disciplined, so it is fine for Beast-format feeder compatibility but
    // not for real cross-receiver MLAT. signal_level is 0-255, scaled from
    // the average bit-slicing delta already computed for the noise filter --
    // relative only, not a calibrated dBm figure.
    uint64_t timestamp_12mhz;
    int      signal_level;
};

#define MODE_S_NAV_HAVE_MCP     (1 << 0)
#define MODE_S_NAV_HAVE_FMS     (1 << 1)
#define MODE_S_NAV_HAVE_QNH     (1 << 2)
#define MODE_S_NAV_HAVE_HEADING (1 << 3)
#define MODE_S_NAV_HAVE_MODES   (1 << 4)

#define MODE_S_NAV_MODE_AUTOPILOT (1 << 0)
#define MODE_S_NAV_MODE_VNAV      (1 << 1)
#define MODE_S_NAV_MODE_ALT_HOLD  (1 << 2)
#define MODE_S_NAV_MODE_APPROACH  (1 << 3)
#define MODE_S_NAV_MODE_LNAV      (1 << 4)
#define MODE_S_NAV_MODE_TCAS      (1 << 5)

typedef void (*mode_s_callback_t)(mode_s_t *self, struct mode_s_msg *mm);

void mode_s_init(mode_s_t *self);
void mode_s_compute_magnitude_vector(unsigned char *data, uint16_t *mag, uint32_t size);
// base_ts_us is esp_timer_get_time()-scale microseconds at mag[0]; every
// detected message's timestamp_12mhz is derived from it plus that message's
// sample offset (2 MSPS => 6 ticks/sample at 12MHz).
void mode_s_detect(mode_s_t *self, uint16_t *mag, uint32_t maglen, uint64_t base_ts_us, mode_s_callback_t);
void mode_s_decode(mode_s_t *self, struct mode_s_msg *mm, unsigned char *msg);
// Human-readable advisory for a BDS 3,0 block that passed the acas_ra_valid
// test ("climb", "clear of conflict", ...). Returns out.
const char *mode_s_acas_ra_text(const unsigned char *mb, char *out, size_t n);
// NIC (navigation integrity category) of a position message: the type code
// sets it, the supplements (A and C from the aircraft's TC31, B from this
// message) refine it under version 1/2 rules.
int mode_s_nic(int metype, int version, int nic_a, int nic_b, int nic_c);
void runme();