#ifndef _GPS_SATELLITE_H
#define _GPS_SATELLITE_H

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

static const uint8_t antenna_pat_elevation_num = 19; // every 5deg

// Source: https://en.wikipedia.org/wiki/GPS_satellite_blocks, 2024/07/25
typedef enum
{
    // Retired now
    GPS_BLOCK_I,
    GPS_BLOCK_II,
    GPS_BLOCK_IIA,
    // Operational
    GPS_BLOCK_IIR,
    GPS_BLOCK_IIRM,
    GPS_BLOCK_IIF,
    GPS_BLOCK_III,
    // Future
    GPS_BLOCK_IIIF,
    GPS_BLOCK_MAX
} GPS_BLOCK;

typedef struct gpssatellite
{
    uint32_t PRN;  // 1 - 32
    char* block;
    GPS_BLOCK block_id;
    int8_t antenna_gain[antenna_pat_elevation_num];  // dB
} gps_satellite;

uint8_t InitGPSSatellite(gps_satellite* gps_sat, const uint32_t PRN);
uint8_t GetAntennaGain(const gps_satellite* gps_sat, const uint8_t elevation_deg, int8_t* gain);

#endif // _GPS_SATELLITE_H
