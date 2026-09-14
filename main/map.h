/*
 * The map under the display: land and airports baked into map_data.h by
 * tools/mkmap.py, sampled onto a character grid centred on the antenna.
 * Pure C so it also builds on the host for a look at the output.
 */
#pragma once

#include <stdbool.h>

typedef void (*map_cell_fn)(int y, int x);
typedef void (*map_apt_fn)(int y, int x, const char *code);

/* False when the data was baked for another antenna position. */
bool map_matches(float lat, float lon);

/* Half-sizes of the baked box, km east-west and north-south of the antenna. */
void map_extent(float *half_ew_km, float *half_ns_km);

/* Cell (cy, cx) is the antenna; a cell is km_per_col wide and km_per_row
 * tall. land() is called for every cell whose centre is on land; airports()
 * for every airport in view, nearest first, with the cell its marker goes in.
 * map_land takes the antenna in fractional cells so a caller sampling a finer
 * grid (2x2 per character) can keep it on the character's centre. */
void map_land(int rows, int cols, float cy, float cx, float km_per_row, float km_per_col, map_cell_fn land);
void map_airports(int rows, int cols, float cy, float cx, float km_per_row, float km_per_col, map_apt_fn airport);
