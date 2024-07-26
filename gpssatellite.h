#ifndef _GPS_SATELLITE_H
#define _GPS_SATELLITE_H

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define ANTENNA_ELE_RESOLUTION_DEG 5
#define ANTENNA_PAT_ELE_NUM (90 / ANTENNA_ELE_RESOLUTION_DEG + 1)

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
    const char* block;
    GPS_BLOCK block_id;
    int8_t antenna_gain[ANTENNA_PAT_ELE_NUM];  // dB
} gps_satellite;

uint8_t InitGPSSatellite(gps_satellite* gps_sat, const uint32_t PRN);
uint8_t GetAntennaGain(const gps_satellite* gps_sat, const uint8_t elevation_deg, int8_t* gain);
uint8_t SetAntennaPattern(gps_satellite* gps_sat, const uint8_t pattern_id);

#endif // _GPS_SATELLITE_H
