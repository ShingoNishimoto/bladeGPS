#include "gpssatellite.h"

#include <time.h>
#include <math.h>
#include <csv.h>
#include <libgen.h>
#ifdef _WIN32
// To avoid conflict between time.h and pthread.h on Windows
#define HAVE_STRUCT_TIMESPEC
#endif

#define MAX_ANTENNA_PATTERN 3

// static const char* block_list[GPS_BLOCK_MAX] = {
//     "I",
//     "II",
//     "IIA",
//     "IIR",
//     "IIR-M",
//     "IIF",
//     "III",
//     "IIIF"
//     };

// NOTE: The gain from 0 - 15 deg is common from the Block IIR data.
// static const int8_t antenna_gains[GPS_BLOCK_MAX][MAX_ANTENNA_PATTERN][ANTENNA_PAT_ELE_NUM] = {
//     // Off-Boresight angle [deg]
//     //  0,  5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90
//     // I
//     {},
//     // II
//     {},
//     // IIA
//     //  0,  5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90
//     { {13, 15, 16, 14,  8,-13, -2, -3,-12,-18, -2,  0,  1, -2, -7,-17,-20,-20,-20},  // pattern 1 default, highest peak
//       {13, 15, 16, 14,  8, -8, -2, -3, -8, -8, -5, -4, -5, -8,-17,-22,-22,-22,-22},  // pattern 2
//       {13, 15, 16, 14,  8, -3, -8,-10,-10,-10,-10,-15,-17,-17,-17,-22,-22,-22,-22},  // pattern 3 lowest peak
//     },
//     // IIR
//     //  0,  5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90
//     { {13, 15, 16, 14,  4,  0,  4,  3, -2,-10, -1,  2,  0, -6,-15,-15,-15,-15,-15},  // pattern 1 default, highest peak
//       {13, 15, 16, 14,  4,  0,  4,  0, -3, -3, -5,-11,-17,-17,-17,-17,-17,-17,-17},  // pattern 2
//       {13, 15, 16, 14,  4,-13, -7, -7, -5, -6, -5, -4, -3, -2, -3, -7,-12,-15,-15},  // pattern 3 lowest peak
//     },
//     // IIR-M
//     //  0,  5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90
//     { {13, 15, 16, 14,  8,  0,  3,  0, -5,-10,-12,-22,-17,-17,-17,-17,-17,-17,-17},  // pattern 1 default, highest peak
//       {13, 15, 16, 14,  8, -5,-15, -7, -3, -5, -4, -3, -8, -8, -6,-15,-20,-14,-14},  // pattern 2
//       {13, 15, 16, 14,  8, -4, -8, -7, -5,-13,-18,-13,-13,-13,-19,-19,-19,-19,-19},  // pattern 3 lowest peak
//     },
//     // IIF
//     //  0,  5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90
//     { {13, 15, 16, 14, -7, -1, -4,-11, -5, -3, -1,  0, -2, -7,-18,-18,-18,-18,-18},  // pattern 1 default, highest peak
//       {13, 15, 16, 14,-13, -5, -4,-11, -7, -5, -4, -7,-11,-19,-19,-19,-19,-19,-19},  // pattern 2
//       {13, 15, 16, 14, -3,-19,-22,-15,-12,-12, -9,-12,-17,-22,-22,-22,-22,-22,-22},  // pattern 3 lowest peak
//     },
//     // III NOTE: there is no information of III, so use that of IIRM because manufacturer is the same (Lockheed Martin)
//     //  0,  5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90
//     { {13, 15, 16, 14,  8,  0,  3,  0, -5,-10,-12,-22,-17,-17,-17,-17,-17,-17,-17},  // pattern 1 default, highest peak
//       {13, 15, 16, 14,  8, -5,-15, -7, -3, -5, -4, -3, -8, -8, -6,-15,-20,-14,-14},  // pattern 2
//       {13, 15, 16, 14,  8, -4, -8, -7, -5,-13,-18,-13,-13,-13,-19,-19,-19,-19,-19},  // pattern 3 lowest peak
//     },
//     // IIIF
//     {}
// };

typedef enum {
    CSV_INT,
    CSV_FLOAT,
    CSV_STRING
} CSVType;

typedef struct {
    CSVType type;
    union {
        int i;
        float f;
        char *s;
    } value;
} CSVCell;

typedef struct {
    CSVCell **data;
    int row;
    int col;
    int max_rows;
    int max_cols;
} CSVState;


void free_cell(CSVCell *cell) {
    if (cell->type == CSV_STRING) {
        free(cell->value.s);
    }
}


void field_cb(void *s, size_t len, void *data_) {
    CSVState *state = (CSVState *)data_;

    if (state->row >= state->max_rows) {
        state->max_rows *= 2;
        state->data = realloc(state->data, state->max_rows * sizeof(CSVCell *));
        for (int i = state->row; i < state->max_rows; i++) {
            state->data[i] = NULL;
        }
    }

    if (state->col >= state->max_cols) {
        state->max_cols *= 2;
        for (int i = 0; i <= state->row; i++) {
            if (state->data[i]) {
                state->data[i] = realloc(state->data[i], state->max_cols * sizeof(CSVCell));
                if (!state->data[i]) {
                    perror("realloc cols");
                    exit(1);
                }
            }
        }
    }

    char buf[256];
    snprintf(buf, sizeof(buf), "%.*s", (int)len, (char *)s);

    // Allocate next row if needed
    if (!state->data[state->row]) {
        state->data[state->row] = calloc(state->max_cols, sizeof(CSVCell));
    }

    CSVCell *cell = &state->data[state->row][state->col++];

    // Type inference (simple): check for '.' for float, digit-only for int, else string
    if (strchr(buf, '.')) {
        cell->type = CSV_FLOAT;
        cell->value.f = strtof(buf, NULL);
    } else if (strspn(buf, "0123456789-") == strlen(buf)) {
        cell->type = CSV_INT;
        cell->value.i = atoi(buf);
    } else {
        cell->type = CSV_STRING;
        cell->value.s = strdup(buf);
    }
}


void row_cb(int c, void *data_) {
    CSVState *state = (CSVState *)data_;
    state->row++;
    state->col = 0;
}


static FILE *open_csv_relative_to_source(const char *relative_path) {
    char *file_copy = strdup(SOURCE_FILE);
    if (!file_copy) {
        perror("strdup");
        return NULL;
    }

    char *dir = dirname(file_copy);
    char full_path[1024];
    snprintf(full_path, sizeof(full_path), "%s/%s", dir, relative_path);
    free(file_copy);

    FILE *fp = fopen(full_path, "rb");
    if (!fp) {
        perror("fopen");
        fprintf(stderr, "Failed to open: %s\n", full_path);
    }
    return fp;
}

static int parse_csv(FILE *fp, size_t max_rows, size_t max_cols, CSVState *state) {
    struct csv_parser parser;
    if (csv_init(&parser, CSV_STRICT) != 0) {
        fprintf(stderr, "csv_init failed\n");
        return 1;
    }

    *state = (CSVState){ .row = 0, .col = 0, .max_rows = max_rows, .max_cols = max_cols };
    state->data = calloc(max_rows, sizeof(CSVCell *));
    if (!state->data) {
        perror("calloc data");
        return 1;
    }
    state->data[0] = calloc(max_cols, sizeof(CSVCell));
    if (!state->data[0]) {
        perror("calloc row 0");
        return 1;
    }

    char buf[1024];
    size_t bytes_read;
    while ((bytes_read = fread(buf, 1, sizeof(buf), fp)) > 0) {
        if (csv_parse(&parser, buf, bytes_read, field_cb, row_cb, state) != bytes_read) {
            fprintf(stderr, "Parse error: %s\n", csv_strerror(csv_error(&parser)));
            return 1;
        }
    }

    if (csv_fini(&parser, field_cb, row_cb, state) != 0) {
        fprintf(stderr, "csv_fini error: %s\n", csv_strerror(csv_error(&parser)));
    }

    csv_free(&parser);
    return 0;
}

static void free_csv_state(CSVState *state) {
    for (int i = 0; i < state->max_rows; i++) {
        if (state->data[i]) {
            for (int j = 0; j < state->max_cols; j++) {
                free_cell(&state->data[i][j]);
            }
            free(state->data[i]);
        }
    }
    free(state->data);
}


uint8_t ReadGpsTable(gps_table* gps_tables) {
    FILE *fp = open_csv_relative_to_source("gnss/gps_table.csv");
    if (!fp) return 1;

    CSVState state;
    if (parse_csv(fp, MAX_SAT, 5, &state)) {
        fclose(fp);
        return 1;
    }
    fclose(fp);

    for (int i = 1; i < state.row; i++) {
        gps_tables[i - 1].PRN = state.data[i][0].value.i;
        gps_tables[i - 1].SVN = state.data[i][1].value.i;
        gps_tables[i - 1].block_name = strdup(state.data[i][2].value.s);
        gps_tables[i - 1].antenna_file_name = strdup(state.data[i][4].value.s);
    }

    free_csv_state(&state);
    return 0;
}

uint8_t ReadGpsAntennaGain(gps_satellite* gps_sat) {
    char rel_path[512];
    snprintf(rel_path, sizeof(rel_path), "gnss/antenna/%s.txt", gps_sat->sat_info.antenna_file_name);
    FILE *fp = open_csv_relative_to_source(rel_path);
    if (!fp) return 1;

    CSVState state;
    if (parse_csv(fp, ANT_ELE_NUM + 1, ANT_AZI_NUM + 1, &state)) {
        fclose(fp);
        return 1;
    }
    fclose(fp);

    for (int i = 1; i < state.max_rows; i++) {
        for (int j = 1; j < state.max_cols; j++) {
            gps_sat->antenna_gain[i - 1][j - 1] = state.data[i][j].value.f;
        }
    }

    free_csv_state(&state);
    return 0;
}



uint8_t InitGPSSatellite(gps_satellite* gps_sats)
{
    gps_table gps_tables[MAX_SAT];
    ReadGpsTable(gps_tables);
    for (int i = 0; i < MAX_SAT; i++)
        {
            gps_sats[i].sat_info = gps_tables[i];
            ReadGpsAntennaGain(&gps_sats[i]);
        }

    return 0;
}

// TODO: consider the azimuth distribution
uint8_t GetAntennaGain(const gps_satellite* gps_sat, const uint8_t elevation_deg, const uint16_t azimuth_deg, int8_t* gain)
{
    if (elevation_deg > 90 || azimuth_deg >= 360)
    {
        printf("Invalid attitude. Ele: %u, Azi: %u [deg]\n", elevation_deg, azimuth_deg);
        return 1;
    }

    const uint8_t zenith_deg = 90 - elevation_deg;
    *gain = gps_sat->antenna_gain[zenith_deg][azimuth_deg];
    // const uint8_t nearest_id = round((ANTENNA_PAT_ELE_NUM - 1) * zenith_deg / 90.0);
    // const uint8_t nearest_zenith = nearest_id * ANTENNA_ELE_RESOLUTION_DEG;
    // uint8_t other_id;
    // if (zenith_deg > nearest_zenith)
    // {
    //     other_id = nearest_id + 1;
    // }
    // else
    // {
    //     other_id = nearest_id - 1;
    // }

    // *gain = gps_sat->antenna_gain[nearest_id]
    //       + (gps_sat->antenna_gain[other_id] - gps_sat->antenna_gain[nearest_id])
    //       * (zenith_deg - nearest_zenith) / ANTENNA_ELE_RESOLUTION_DEG;
    return 0;
}


// For test.
// int	main(int	argc,	char	const	*argv[]){
//     // gps_table gps_tables[MAX_SAT];
//     // ReadGpsTable(gps_tables);
//     InitGPSSatellite();
//     return 0;
// }
