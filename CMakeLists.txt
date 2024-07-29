add_library(bladegps
    SHARED
        bladegps.c
        gpssim.c
        gpssatellite.c
        getopt.c
)

target_include_directories(bladegps
    PUBLIC
        ${CMAKE_SOURCE_DIR}/bladeRF/host/libraries/libbladeRF/include
        ${CMAKE_SOURCE_DIR}/bladeRF/host/build/output
)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -Wall -I ${CMAKE_SOURCE_DIR}/bladeRF/host/libraries/libbladeRF/include -shared")
set(CMAKE_EXE_LINKER_FLAGS "${PLATFORM_LDFLAGS} -lm -lpthread -L ${CMAKE_SOURCE_DIR}/bladeRF/host/build/output -lbladeRF -fPIC")