# CANlib (the CAN message definitions shared by main boards and expansion boards), built for
# DUET_MCU, which the consuming project's toolchain file sets along with the core's code generation
# flags.
#
#   DUET_MCU             SAME70, SAME5x, SAMC21 or STM32H5
#   DUET_PART_DEFINE     the macro naming the MCU part, e.g. __SAME70Q20B__
#   DUET_RTL_DUMPS       write each source's RTL (-fdump-rtl-expand), for call graph and
#                        stack analysis; ccache cannot cache these compiles
#   DUET_RTOS            build against Duet::FreeRTOS, which must already exist
#   CANLIB_OPTIMISATION  Release optimisation for this library, if not the build type's
#
# Target: Duet::CANlib. Needs Duet::RRFLibraries and Duet::CoreN2G.

cmake_minimum_required(VERSION 3.25)
project(CANlib LANGUAGES CXX)

set(_options)
if(DUET_MCU STREQUAL "STM32H5")
    set(_options -mno-unaligned-access)
elseif(NOT DUET_MCU MATCHES "^(SAME70|SAME5x|SAMC21)$")
    message(FATAL_ERROR "CANlib: unsupported DUET_MCU '${DUET_MCU}'")
endif()

file(GLOB_RECURSE _srcs CONFIGURE_DEPENDS src/*.cpp)

add_library(CANlib STATIC ${_srcs})
add_library(Duet::CANlib ALIAS CANlib)

set_target_properties(CANlib PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)

target_include_directories(CANlib PUBLIC src)
target_compile_definitions(CANlib PRIVATE ${DUET_PART_DEFINE})

target_link_libraries(CANlib PUBLIC Duet::RRFLibraries Duet::CoreN2G)
if(DUET_RTOS)
    target_link_libraries(CANlib PRIVATE Duet::FreeRTOS)
endif()

target_compile_options(CANlib PRIVATE
    $<$<CONFIG:Release>:${CANLIB_OPTIMISATION}>
    ${_options}
    -fno-math-errno
    -fsingle-precision-constant
    -fno-threadsafe-statics
    -fno-rtti
    -fno-exceptions
    -Wall
    -Wundef
    -Wdouble-promotion
    -Werror=return-type
    -Wnoexcept
    -Wshadow
    -Wsign-promo
    -Werror
    -fstack-usage
    $<$<BOOL:${DUET_RTL_DUMPS}>:-fdump-rtl-expand>
    # __FILE__ relative to the library root, so the image does not depend on the checkout location
    -fmacro-prefix-map=${CMAKE_CURRENT_SOURCE_DIR}/=)
