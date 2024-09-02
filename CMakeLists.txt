add_library(bladegps
    SHARED
        bladegps.c
        gpssim.c
        gpssatellite.c
        getopt.c
)

# Setting for SPICE
set(HOME_DIR $ENV{HOME})
set(SPICE_LIB_DIR "${HOME_DIR}/work/cspice/lib/")
set(SPICE_INCLUDE_DIR "${HOME_DIR}/work/cspice/include/")

target_include_directories(bladegps
    PUBLIC
        ${CMAKE_SOURCE_DIR}/bladeRF/host/libraries/libbladeRF/include
        ${CMAKE_SOURCE_DIR}/bladeRF/host/build/output
        # ../algorithms/libs/environment
        ${SPICE_INCLUDE_DIR}
)

target_link_libraries(bladegps
    PRIVATE
        # core_system_parameters
        # algorithms_libs_rtklib
        algorithms_libs_environment
        ${SPICE_LIB_DIR}cspice.a
)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -Wall -I ${CMAKE_SOURCE_DIR}/bladeRF/host/libraries/libbladeRF/include -shared")
set(CMAKE_EXE_LINKER_FLAGS "${PLATFORM_LDFLAGS} -lm -lpthread -L ${CMAKE_SOURCE_DIR}/bladeRF/host/build/output -lbladeRF -fPIC")