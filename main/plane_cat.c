/*
 * Aircraft category from the ICAO address and the ADS-B callsign.
 *
 * Both tables are heuristics, not registries: the ICAO ranges cover only the
 * blocks that are well documented as military, and the callsign rules key off
 * naming conventions rather than any authoritative list. A wrong answer here
 * is cosmetic -- nothing in the decode path branches on it.
 */

#include <ctype.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "plane_cat.h"

static const char *const MIL_PREFIXES[] = {
    /* US */
    "RCH", "SAM", "PAT", "SHADO", "KNIFE", "GRIM", "SPAR",
    "CNV", "VV", "NAVY", "MC", "ARMY", "CG",
    /* UK / NATO */
    "RRR", "NATO", "MAGMA",
    /* Canada */
    "CFC", "HUSKY",
    /* France / Germany */
    "COTAM", "GAF",
    NULL
};

static const struct { uint32_t lo, hi; } MIL_RANGES[] = {
    { 0xADF7C8, 0xAFFFFF },   /* US    */
    { 0x43C000, 0x43FFFF },   /* UK    */
    { 0x3F8000, 0x3FFFFF },   /* DE    */
    { 0x3B7000, 0x3B7FFF },   /* FR    */
    { 0xC87F00, 0xC87FFF },   /* CA    */
    { 0, 0 }
};

static bool starts_with_ci(const char *s, const char *pfx)
{
    while (*pfx) {
        if (toupper((unsigned char)*s) != toupper((unsigned char)*pfx)) return false;
        s++; pfx++;
    }
    return true;
}

/* The digit is not decoration: military callsigns are prefix-then-number, while
 * registration marks continue with letters, and several prefixes collide with
 * a registration otherwise -- "CG" swallows every Canadian C-Gxxx, "GAF" every
 * British G-AFxx. Requiring a digit separates "CG1502" from "CGXYZ". */
static bool callsign_is_mil(const char *cs)
{
    for (int i = 0; MIL_PREFIXES[i]; i++) {
        size_t n = strlen(MIL_PREFIXES[i]);
        if (starts_with_ci(cs, MIL_PREFIXES[i]) && isdigit((unsigned char)cs[n]))
            return true;
    }
    return false;
}

static bool icao_is_mil(uint32_t icao)
{
    for (int i = 0; MIL_RANGES[i].hi; i++)
        if (icao >= MIL_RANGES[i].lo && icao <= MIL_RANGES[i].hi) return true;
    return false;
}

/* Airline flight numbers: three-letter ICAO operator code then a digit. */
static bool callsign_is_commercial(const char *cs)
{
    return isalpha((unsigned char)cs[0]) && isalpha((unsigned char)cs[1]) &&
           isalpha((unsigned char)cs[2]) && isdigit((unsigned char)cs[3]);
}

/* Registration marks flown as the callsign. Every test short-circuits on the
 * NUL, so a one- or two-character callsign never reads past the terminator. */
static bool callsign_is_ga(const char *cs)
{
    if (cs[0] == 'N' && isdigit((unsigned char)cs[1])) return true;
    if (cs[0] == 'E' && cs[1] == 'I' && isalpha((unsigned char)cs[2])) return true;
    /* CF- is Canadian military, already caught above by CFC. */
    if ((cs[0] == 'G' || cs[0] == 'D' || cs[0] == 'F' ||
         (cs[0] == 'C' && cs[1] != 'F')) &&
        isalpha((unsigned char)cs[1]) && isalpha((unsigned char)cs[2])) return true;
    return false;
}

plane_cat_t plane_classify(uint32_t icao, const char *callsign)
{
    const char *cs = callsign ? callsign : "";

    if (icao_is_mil(icao) || callsign_is_mil(cs)) return PLANE_MILITARY;
    /* Commercial must be tested first: "DLH123" and "CCA1234" satisfy the GA
     * rule as well, and only the digit in the fourth slot separates an airline
     * flight number from a five-letter registration like "DEFGH". */
    if (callsign_is_commercial(cs))               return PLANE_COMMERCIAL;
    if (callsign_is_ga(cs))                       return PLANE_GA;
    return PLANE_UNKNOWN;
}

const char *plane_cat_label(plane_cat_t c)
{
    switch (c) {
    case PLANE_MILITARY:   return "MIL";
    case PLANE_COMMERCIAL: return "COM";
    case PLANE_GA:         return "GA";
    default:               return "UNK";
    }
}
