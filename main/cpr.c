#include "cpr.h"
#include <math.h>

/* The CPR indices j and m are routinely negative and C's % and fmod() keep
 * the dividend's sign; the algorithm wants [0, b). */
static int mod_int(int a, int b)
{
    int r = a % b;
    return r < 0 ? r + b : r;
}

static double mod_dbl(double a, double b)
{
    double r = fmod(a, b);
    return r < 0.0 ? r + b : r;
}

int cpr_nl(double lat)
{
    if (lat < 0) lat = -lat;
    if (lat < 10.47047130) return 59;
    if (lat < 14.82817437) return 58;
    if (lat < 18.18626357) return 57;
    if (lat < 21.02939493) return 56;
    if (lat < 23.54504487) return 55;
    if (lat < 25.82924707) return 54;
    if (lat < 27.93898710) return 53;
    if (lat < 29.91135686) return 52;
    if (lat < 31.77209708) return 51;
    if (lat < 33.53993436) return 50;
    if (lat < 35.22899598) return 49;
    if (lat < 36.85025108) return 48;
    if (lat < 38.41241892) return 47;
    if (lat < 39.92256684) return 46;
    if (lat < 41.38651832) return 45;
    if (lat < 42.80914012) return 44;
    if (lat < 44.19454951) return 43;
    if (lat < 45.54626723) return 42;
    if (lat < 46.86733252) return 41;
    if (lat < 48.16039128) return 40;
    if (lat < 49.42776439) return 39;
    if (lat < 50.67150166) return 38;
    if (lat < 51.89342469) return 37;
    if (lat < 53.09516153) return 36;
    if (lat < 54.27817472) return 35;
    if (lat < 55.44378444) return 34;
    if (lat < 56.59318756) return 33;
    if (lat < 57.72747354) return 32;
    if (lat < 58.84763776) return 31;
    if (lat < 59.95459277) return 30;
    if (lat < 61.04917774) return 29;
    if (lat < 62.13216659) return 28;
    if (lat < 63.20427479) return 27;
    if (lat < 64.26616523) return 26;
    if (lat < 65.31845310) return 25;
    if (lat < 66.36171008) return 24;
    if (lat < 67.39646774) return 23;
    if (lat < 68.42322022) return 22;
    if (lat < 69.44242631) return 21;
    if (lat < 70.45451075) return 20;
    if (lat < 71.45986473) return 19;
    if (lat < 72.45884545) return 18;
    if (lat < 73.45177442) return 17;
    if (lat < 74.43893416) return 16;
    if (lat < 75.42056257) return 15;
    if (lat < 76.39684391) return 14;
    if (lat < 77.36789461) return 13;
    if (lat < 78.33374083) return 12;
    if (lat < 79.29428225) return 11;
    if (lat < 80.24923213) return 10;
    if (lat < 81.19801349) return 9;
    if (lat < 82.13956981) return 8;
    if (lat < 83.07199445) return 7;
    if (lat < 83.99173563) return 6;
    if (lat < 84.89166191) return 5;
    if (lat < 85.75541621) return 4;
    if (lat < 86.53536998) return 3;
    if (lat < 87.00000000) return 2;
    return 1;
}

/* Longitude zones for a frame of this parity: the odd encoding uses one fewer. */
static int n_zones(double lat, bool odd)
{
    int n = cpr_nl(lat) - (odd ? 1 : 0);
    return n < 1 ? 1 : n;
}

static double dlon(double lat, bool odd, bool surface)
{
    return (surface ? 90.0 : 360.0) / n_zones(lat, odd);
}

/* The longitude index m and the result for the frame of the chosen parity.
 * Shared by the airborne and surface decoders; only the latitude step and the
 * quadrant handling differ between them. */
static double decode_lon(double lat, int even_lon, int odd_lon, bool odd_latest, bool surface)
{
    int nl = cpr_nl(lat);
    int m  = (int)floor(((double)even_lon * (nl - 1) - (double)odd_lon * nl) / 131072.0 + 0.5);
    int ni = n_zones(lat, odd_latest);
    int raw = odd_latest ? odd_lon : even_lon;
    return dlon(lat, odd_latest, surface) * (mod_int(m, ni) + raw / 131072.0);
}

int cpr_decode_airborne(int even_lat, int even_lon, int odd_lat, int odd_lon,
                        bool odd_latest, double *lat, double *lon)
{
    const double dlat0 = 360.0 / 60.0, dlat1 = 360.0 / 59.0;

    int    j     = (int)floor((59.0 * even_lat - 60.0 * odd_lat) / 131072.0 + 0.5);
    double rlat0 = dlat0 * (mod_int(j, 60) + even_lat / 131072.0);
    double rlat1 = dlat1 * (mod_int(j, 59) + odd_lat / 131072.0);
    if (rlat0 >= 270.0) rlat0 -= 360.0;
    if (rlat1 >= 270.0) rlat1 -= 360.0;
    if (rlat0 < -90.0 || rlat0 > 90.0 || rlat1 < -90.0 || rlat1 > 90.0) return -2;

    /* Both frames must sit in the same longitude-zone band or m is meaningless. */
    if (cpr_nl(rlat0) != cpr_nl(rlat1)) return -1;

    double rlat = odd_latest ? rlat1 : rlat0;
    double rlon = decode_lon(rlat, even_lon, odd_lon, odd_latest, false);
    rlon -= floor((rlon + 180.0) / 360.0) * 360.0;

    *lat = rlat;
    *lon = rlon;
    return 0;
}

/* Surface latitude comes out in 0..90; the true value is either that or 90
 * less. Take whichever is nearer the reference; the poles and the equator
 * all encode as zero and need the reference to tell them apart. */
static double surface_lat_quadrant(double rlat, double reflat)
{
    if (rlat == 0.0) {
        if (reflat < -45.0) return -90.0;
        if (reflat >  45.0) return  90.0;
        return 0.0;
    }
    return (rlat - reflat) > 45.0 ? rlat - 90.0 : rlat;
}

int cpr_decode_surface(double reflat, double reflon,
                       int even_lat, int even_lon, int odd_lat, int odd_lon,
                       bool odd_latest, double *lat, double *lon)
{
    const double dlat0 = 90.0 / 60.0, dlat1 = 90.0 / 59.0;

    int    j     = (int)floor((59.0 * even_lat - 60.0 * odd_lat) / 131072.0 + 0.5);
    double rlat0 = surface_lat_quadrant(dlat0 * (mod_int(j, 60) + even_lat / 131072.0), reflat);
    double rlat1 = surface_lat_quadrant(dlat1 * (mod_int(j, 59) + odd_lat / 131072.0), reflat);
    if (rlat0 < -90.0 || rlat0 > 90.0 || rlat1 < -90.0 || rlat1 > 90.0) return -2;
    if (cpr_nl(rlat0) != cpr_nl(rlat1)) return -1;

    double rlat = odd_latest ? rlat1 : rlat0;
    double rlon = decode_lon(rlat, even_lon, odd_lon, odd_latest, true);

    /* All four longitude quadrants are possible: shift by whole quarters
     * towards the reference, then wrap into -180..180. */
    rlon += floor((reflon - rlon + 45.0) / 90.0) * 90.0;
    rlon -= floor((rlon + 180.0) / 360.0) * 360.0;

    *lat = rlat;
    *lon = rlon;
    return 0;
}

int cpr_decode_relative(double reflat, double reflon, int cpr_lat, int cpr_lon,
                        bool odd, bool surface, double *lat, double *lon)
{
    double flat  = cpr_lat / 131072.0;
    double flon  = cpr_lon / 131072.0;
    double dlat  = (surface ? 90.0 : 360.0) / (odd ? 59.0 : 60.0);

    int    j    = (int)(floor(reflat / dlat) + floor(0.5 + mod_dbl(reflat, dlat) / dlat - flat));
    double rlat = dlat * (j + flat);
    if (rlat >= 270.0) rlat -= 360.0;
    if (rlat < -90.0 || rlat > 90.0) return -1;
    if (fabs(rlat - reflat) > dlat / 2.0) return -1;

    double dl   = dlon(rlat, odd, surface);
    int    m    = (int)(floor(reflon / dl) + floor(0.5 + mod_dbl(reflon, dl) / dl - flon));
    double rlon = dl * (m + flon);
    if (rlon > 180.0) rlon -= 360.0;
    if (fabs(rlon - reflon) > dl / 2.0) return -1;

    *lat = rlat;
    *lon = rlon;
    return 0;
}
