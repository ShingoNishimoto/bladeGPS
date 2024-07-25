#include "gpssatellite.h"

#include <time.h>
#include <math.h>
#ifdef _WIN32
// To avoid conflict between time.h and pthread.h on Windows
#define HAVE_STRUCT_TIMESPEC
#endif

static const uint8_t max_prn = 32;
static const uint8_t max_antenna_pattern = 3;
static const uint8_t antenna_elevation_resolution_deg = 5;

static const char* block_list[GPS_BLOCK_MAX] = {
    "I",
    "II",
    "IIA",
    "IIR",
    "IIR-M",
    "IIF",
    "III",
    "IIIF"
    };

// NOTE: The gain from 0 - 15 deg is common from the Block IIR data.
static const int8_t antenna_gains[GPS_BLOCK_MAX][max_antenna_pattern][antenna_pat_elevation_num] = {
    // Off-Boresight angle [deg]
    //  0,  5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90
    {},
    {},
    // IIA
    //  0,  5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90
    { {13, 15, 16, 14, },  // pattern 1 default, highest peak
      {13, 15, 16, 14, },  // pattern 2
      {13, 15, 16, 14, },  // pattern 3 lowest peak
    },
    // IIR
    //  0,  5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90
    { {13, 15, 16, 14,  4,  0,  4,  3, -2,-10, -1,  2,  0, -6,-15,-15,-15,-15,-15},  // pattern 1 default, highest peak
      {13, 15, 16, 14,  4,  0,  4,  0, -3, -3, -5,-11,-17,-17,-17,-17,-17,-17,-17},  // pattern 2
      {13, 15, 16, 14,  4,-13, -7, -7, -5, -6, -5, -4, -3, -2, -3, -7,-12,-15,-15},  // pattern 3 lowest peak
    },
    // IIR-M
    //  0,  5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90
    { {13, 15, 16, 14,  8,  0,  3,  0, -5,-10,-12,-22,-17,-17,-17,-17,-17,-17,-17},  // pattern 1 default, highest peak
      {13, 15, 16, 14,  8, -5,-15, -7, -3, -5, -4, -3, -8, -8, -6,-15,-20,-14,-14},  // pattern 2
      {13, 15, 16, 14,  8, -4, -8, -7, -5,-13,-18,-13,-13,-13,-19,-19,-19,-19,-19},  // pattern 3 lowest peak
    },
    // IIF
    { {13, 15, 16, 14, },  // pattern 1 default, highest peak
      {13, 15, 16, 14, },  // pattern 2
      {13, 15, 16, 14, },  // pattern 3 lowest peak
    },
};

uint8_t InitGPSSatellite(gps_satellite* gps_sat, const uint32_t PRN)
{
    gps_sat->PRN = PRN;
    // info from https://www.navcen.uscg.gov/?Do=constellationStatus
    switch (PRN)
        {
        case 1:
            gps_sat->block_id = GPS_BLOCK_IIF;
            break;
        case 2:
            gps_sat->block_id = GPS_BLOCK_IIR;
            break;
        case 3:
            gps_sat->block_id = GPS_BLOCK_IIF;
            break;
        case 4:
            gps_sat->block_id = GPS_BLOCK_III;
            break;
        case 5:
            gps_sat->block_id = GPS_BLOCK_IIRM;
            break;
        case 6:
            gps_sat->block_id = GPS_BLOCK_IIF;
            break;
        case 7:
            gps_sat->block_id = GPS_BLOCK_IIRM;
            break;
        case 8:
            gps_sat->block_id = GPS_BLOCK_IIF;
            break;
        case 9:
            gps_sat->block_id = GPS_BLOCK_IIF;
            break;
        case 10:
            gps_sat->block_id = GPS_BLOCK_IIF;
            break;
        case 11:
            gps_sat->block_id = GPS_BLOCK_III;
            break;
        case 12:
            gps_sat->block_id = GPS_BLOCK_IIRM;
            break;
        case 13:
            gps_sat->block_id = GPS_BLOCK_IIR;
            break;
        case 14:
            gps_sat->block_id = GPS_BLOCK_III;
            break;
        case 15:
            gps_sat->block_id = GPS_BLOCK_IIRM;
            break;
        case 16:
            gps_sat->block_id = GPS_BLOCK_IIR;
            break;
        case 17:
            gps_sat->block_id = GPS_BLOCK_IIRM;
            break;
        case 18:
            gps_sat->block_id = GPS_BLOCK_III;
            break;
        case 19:
            gps_sat->block_id = GPS_BLOCK_IIR;
            break;
        case 20:
            gps_sat->block_id = GPS_BLOCK_IIR;
            break;
        case 21:
            gps_sat->block_id = GPS_BLOCK_IIR;
            break;
        case 22:
            gps_sat->block_id = GPS_BLOCK_IIR;
            break;
        case 23:
            gps_sat->block_id = GPS_BLOCK_III;
            break;
        case 24:
            gps_sat->block_id = GPS_BLOCK_IIF;
            break;
        case 25:
            gps_sat->block_id = GPS_BLOCK_IIF;
            break;
        case 26:
            gps_sat->block_id = GPS_BLOCK_IIF;
            break;
        case 27:
            gps_sat->block_id = GPS_BLOCK_IIF;
            break;
        case 28:
            gps_sat->block_id = GPS_BLOCK_IIR;
            break;
        case 29:
            gps_sat->block_id = GPS_BLOCK_IIRM;
            break;
        case 30:
            gps_sat->block_id = GPS_BLOCK_IIF;
            break;
        case 31:
            gps_sat->block_id = GPS_BLOCK_IIRM;
            break;
        case 32:
            gps_sat->block_id = GPS_BLOCK_IIF;
            break;
        default:
            return 1;
        }

    gps_sat->block = block_list[gps_sat->block_id];
    for (uint8_t i = 0; i < antenna_pat_elevation_num; i++)
    {
        gps_sat->antenna_gain[i] = antenna_gains[gps_sat->block_id][0][i];
    }
    return 0;
}

// TODO: consider the azimuth distribution
uint8_t GetAntennaGain(const gps_satellite* gps_sat, const uint8_t elevation_deg, int8_t* gain)
{
    if (elevation_deg > 90)
    {
        printf("Invalid elevation angle %u [deg]\n", elevation_deg);
        return 1;
    }

    const uint8_t zenith_deg = 90 - elevation_deg;
    const uint8_t nearest_id = round((antenna_pat_elevation_num - 1) * zenith_deg / 90.0);
    const uint8_t nearest_zenith = nearest_id * antenna_elevation_resolution_deg;
    uint8_t other_id;
    if (zenith_deg > nearest_zenith)
    {
        other_id = nearest_id + 1;
    }
    else
    {
        other_id = nearest_id - 1;
    }

    *gain = gps_sat->antenna_gain[nearest_id]
          + (gps_sat->antenna_gain[other_id] - gps_sat->antenna_gain[nearest_id])
          * (zenith_deg - nearest_zenith) / antenna_elevation_resolution_deg;
    return 0;
}
