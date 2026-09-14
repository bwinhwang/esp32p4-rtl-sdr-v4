/*
 * See map.h.
 */

#include <math.h>
#include "map.h"
#include "map_data.h"

#define MAX_CROSS 128

bool map_matches(float lat, float lon)
{
    return fabsf(lat - MAP_LAT) < 0.01f && fabsf(lon - MAP_LON) < 0.01f;
}

void map_extent(float *half_ew_km, float *half_ns_km)
{
    *half_ew_km = MAP_HALF_EW_KM;
    *half_ns_km = MAP_HALF_NS_KM;
}

/* Even-odd scanline through the row's centre: every polygon edge it crosses
 * is a coast, and the cells between the 1st and 2nd, 3rd and 4th... are land.
 * The rings were clipped to a box in the generator, so a ring that leaves the
 * box comes back along its edge and the parity stays right. */
void map_land(int rows, int cols, float cy, float cx, float km_per_row, float km_per_col, map_cell_fn land)
{
    const float sx = MAP_UNIT_KM / km_per_col;       /* data units -> cells */

    for (int y = 0; y < rows; y++) {
        float ny = (cy - (float)y) * km_per_row / MAP_UNIT_KM;
        float xs[MAX_CROSS];
        int   n = 0;

        for (int p = 0; p < MAP_NPOLYS && n < MAX_CROSS; p++) {
            const map_pt_t *pt  = &MAP_PTS[MAP_POLY_START[p]];
            int             cnt = MAP_POLY_START[p + 1] - MAP_POLY_START[p];
            for (int i = 0, j = cnt - 1; i < cnt && n < MAX_CROSS; j = i++) {
                float ay = pt[i].y, by = pt[j].y;
                if ((ay <= ny) == (by <= ny)) continue;
                float x = pt[i].x + (pt[j].x - pt[i].x) * (ny - ay) / (by - ay);
                int k = n++;
                while (k > 0 && xs[k - 1] > x) { xs[k] = xs[k - 1]; k--; }
                xs[k] = x;
            }
        }

        for (int i = 0; i + 1 < n; i += 2) {
            int xa = (int)ceilf(xs[i]     * sx + cx);
            int xb = (int)ceilf(xs[i + 1] * sx + cx);
            if (xa < 0)    xa = 0;
            if (xb > cols) xb = cols;
            for (int x = xa; x < xb; x++) land(y, x);
        }
    }
}

void map_airports(int rows, int cols, float cy, float cx, float km_per_row, float km_per_col, map_apt_fn airport)
{
    for (int i = 0; i < MAP_NAPTS; i++) {
        const map_apt_t *a = &MAP_APTS[i];
        int px = (int)lroundf(cx + a->x * MAP_UNIT_KM / km_per_col);
        int py = (int)lroundf(cy - a->y * MAP_UNIT_KM / km_per_row);
        if (py >= 0 && py < rows && px >= 0 && px < cols) airport(py, px, a->code);
    }
}
