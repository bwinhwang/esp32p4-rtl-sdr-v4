#pragma once

#include <stdbool.h>

/* ═════════════════════════════════════════════════════════════════════════════
 * Compact Position Reporting -- the 17+17-bit lat/lon fields of the airborne
 * (TC9-18, 20-22) and surface (TC5-8) position messages. Pure math, no state:
 * the caller keeps the even/odd frames and picks the reference.
 *
 * Every decoder returns 0 with lat/lon set, -1 for "not decodable from this
 * input, try again with the next frame" (frames straddle a latitude zone, or
 * the reference is more than half a cell away) and -2 for data that cannot be
 * a real position.
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Number of longitude zones at this latitude, 1..59. */
int cpr_nl(double lat);

/* Global decode from an even/odd pair; `odd_latest` says which of the two the
 * result is for (the newer frame). Airborne cells are 360/60 degrees of
 * latitude, so the pair alone fixes the position anywhere on Earth. */
int cpr_decode_airborne(int even_lat, int even_lon, int odd_lat, int odd_lon,
                        bool odd_latest, double *lat, double *lon);

/* Surface cells are a quarter the size and the message carries no quadrant, so
 * the pair fixes the position only to within 90 degrees of latitude and
 * longitude: `reflat`/`reflon` pick the quadrant nearest to them. Any point
 * within ~45 degrees of the truth -- the antenna, or the aircraft's own last
 * fix -- is a good enough reference. */
int cpr_decode_surface(double reflat, double reflon,
                       int even_lat, int even_lon, int odd_lat, int odd_lon,
                       bool odd_latest, double *lat, double *lon);

/* Single-frame decode against a reference that is known to be within half a
 * cell (180 NM airborne, 45 NM surface) of the truth: the aircraft's own last
 * fix, while it is fresh. The result is the cell nearest the reference, so a
 * reference outside that bound silently yields a position one cell off -- the
 * caller must bound the reference's age and distance itself. */
int cpr_decode_relative(double reflat, double reflon, int cpr_lat, int cpr_lon,
                        bool odd, bool surface, double *lat, double *lon);
