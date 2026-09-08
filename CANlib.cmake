# CANlib as a reusable CMake component. See lib/FreeRTOS/FreeRTOS.cmake for the pattern.
#
#   canlib_add_library(TARGET <name> MCU <SAME70|SAME51|SAMC21|HOST> ARCH <interface target> [RTOS])
#
# MCU HOST builds for an ordinary hosted target (Linux) rather than a Duet MCU, exactly as the same
# option does in RRFLibraries.cmake. It exists for the SBC-side code, which needs the message
# definitions - CanMessageFormats, CanId, RemoteInputHandle - so that both ends of the CAN link
# describe a message with the same declaration instead of a hand-copied one. Differences from an
# MCU build:
#   - the sources that need a CAN peripheral or an RTOS are excluded; a hosted consumer only ever
#     builds messages, it does not drive a bus.
#   - the freestanding-only compile option -nostdlib is dropped, as is -Werror: this is upstream
#     source built for a target it was not written for, so a new warning here must not break a
#     downstream project's build. -fsingle-precision-constant stays, because it changes results
#     rather than just diagnostics.
#   - the objects are position-independent, because they get linked into a shared library.

set(CANLIB_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(CANLIB_LIBRARY_FLAGS
    "RTOS"
)
set(CANLIB_LIBRARY_ARGS
    "COREN2G_INTERFACE"          # interface target for CoreN2G, if CAN is enabled
    "FREERTOS_INTERFACE"        # interface target for FreeRTOS, if RTOS is enabled
    "RRFLIBRARIES_INTERFACE"    # interface target for RRFLibraries, if RTOS is enabled
)

# Options the consumer must impose on this library's own translation units rather than only on its
# usage requirements - a forced include being the motivating case, as in RRFLibraries.cmake.
set(CANLIB_LIBRARY_MULTI_ARGS
    "EXTRA_COMPILE_OPTIONS"
)

include("${CMAKE_CURRENT_LIST_DIR}/../DuetArch.cmake")

# Create interface target for CANlib, which is linked into other libraries (e.g. CoreN2G)
function(canlib_add_interface OUT_TARGET)
    cmake_parse_arguments(PARSE_ARGV 1 ARG "${CANLIB_LIBRARY_FLAGS}" "${DEFAULT_INTERFACE_ARGS}" "")
    if(ARG_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "canlib_add_interface: unknown arguments: ${ARG_UNPARSED_ARGUMENTS}")
    endif()

    get_enabled_features(_enabled_features ${CANLIB_LIBRARY_FLAGS})
    make_library_name(_target "CANlib" INTERFACE ${ARG_MCU} ${_enabled_features})
    set(${OUT_TARGET} "${_target}" PARENT_SCOPE)
    if(TARGET ${_target})
        return()  # already built for this MCU and feature set
    endif()

    add_library(${_target} INTERFACE)
    target_include_directories(${_target} INTERFACE "${CANLIB_DIR}/src")
endfunction()

# Create static library target for CANlib, which is linked into the executable
function(canlib_add_library OUT_TARGET)
    cmake_parse_arguments(PARSE_ARGV 1 ARG "${CANLIB_LIBRARY_FLAGS}" "${DEFAULT_LIBRARY_ARGS};${CANLIB_LIBRARY_ARGS}" "${CANLIB_LIBRARY_MULTI_ARGS}")
    if(ARG_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "canlib_add_library: unknown arguments: ${ARG_UNPARSED_ARGUMENTS}")
    endif()

    get_enabled_features(_enabled_features ${CANLIB_LIBRARY_FLAGS})
    make_library_name(_target "CANlib" STATIC ${ARG_MCU} ${_enabled_features})
    set(${OUT_TARGET} "${_target}" PARENT_SCOPE)
    if(TARGET ${_target})
        return()  # already built for this MCU and feature set
    endif()

    set(_src "${CANLIB_DIR}/src")
    set(_lib "${CANLIB_DIR}/..")

    if(ARG_MCU STREQUAL "SAME70")
        set(_freertos_port "portable/GCC/ARM_CM7/r0p1")

        # MCU specific compile options
        set(_mcu_nonrtos_compile_options
            "-Wfloat-conversion"
            "-Wsuggest-override"
            "-fstack-usage"
        )
    elseif(ARG_MCU STREQUAL "SAME51")
        set(_freertos_port "portable/GCC/ARM_CM7/r0p1")

        # MCU specific compile options
        set(_mcu_compile_options "-fstack-usage")
        set(_mcu_rtos_compile_options "-fdump-rtl-expand")
        set(_mcu_nonrtos_compile_options
            "-Wfloat-conversion"
            "-Wsuggest-override"
        )
    elseif(ARG_MCU STREQUAL "SAME4E")
        set(_freertos_port "portable/GCC/ARM_CM4F")

        # MCU specific compile options
        set(_mcu_compile_options
            "-Wfloat-conversion"
            "-Wsuggest-override"
            "-fstack-usage"
            "-Os"  # Use Os optimisation
        )
    elseif(ARG_MCU STREQUAL "SAMC21")
        set(_freertos_port "portable/GCC/ARM_CM0")

        # MCU specific compile options
        set(_mcu_compile_options
            "-Wfloat-conversion"
            "-Wsuggest-override"
            "-fstack-usage"
        )
    elseif(ARG_MCU STREQUAL "RP2040")
        set(_freertos_port "portable/GCC/ARM_CM0")

        # MCU specific compile options
        set(_mcu_compile_options
            "-Wfloat-conversion"
            "-Wsuggest-override"
            "-fstack-usage"
        )
    elseif(ARG_MCU STREQUAL "HOST")
        # CanSettings drives a CAN peripheral and CanMessageBuffer's allocator is built on the RTOS
        # free list; a hosted consumer builds messages and does not need either
        set(_excludes "CanSettings" "CanMessageBuffer")
    else()
        message(FATAL_ERROR "canlib_add_library: unsupported MCU '${ARG_MCU}'")
    endif()

    file(GLOB_RECURSE _srcs CONFIGURE_DEPENDS "${_src}/*.cpp")
    foreach(_ex IN LISTS _excludes)
        list(FILTER _srcs EXCLUDE REGEX "${_ex}")
    endforeach()
    add_library(${_target} STATIC ${_srcs})

    target_link_libraries(${_target} PUBLIC I_${_target}) # link own interface target
    target_link_libraries(${_target} PRIVATE
        ${ARG_COREN2G_INTERFACE}
        ${ARG_RRFLIBRARIES_INTERFACE}
    ) # link library dependencies

    if(ARG_RTOS)
        target_link_libraries(${_target} PRIVATE ${ARG_FREERTOS_INTERFACE})
        # target_include_directories(${_target} PRIVATE
        #     "${_lib}/FreeRTOS/src/include"
        #     "${_lib}/FreeRTOS/src/${_freertos_port}"
        # )
        # target_compile_definitions(${_target} PRIVATE RTOS)
        target_compile_options(${_target} PRIVATE ${_mcu_rtos_compile_options})
    else()
        target_compile_options(${_target} PRIVATE ${_mcu_nonrtos_compile_options})
    endif()

    # Options that only make sense freestanding, or that turn an upstream warning into a downstream
    # build failure. A HOST build is this source compiled for a target it was not written for, so it
    # opts out of both groups.
    #
    # -fsingle-precision-constant is deliberately NOT in this group, for the same reason as in
    # RRFLibraries.cmake: it decides whether a literal like 0.01 is float or double. Dropping it
    # would change what HeaterModel computes relative to the firmware, and fill the build with
    # -Wdouble-promotion warnings saying so.
    set(_freestanding_options
        -nostdlib
        -Werror)
    if(ARG_MCU STREQUAL "HOST")
        set(_freestanding_options)
        set_target_properties(${_target} PROPERTIES
            POSITION_INDEPENDENT_CODE ON
            CXX_STANDARD 20
            CXX_STANDARD_REQUIRED ON)
    endif()

    target_compile_options(${_target} PRIVATE
        -ffunction-sections
        -fdata-sections
        -fno-threadsafe-statics
        -fno-rtti # not for SAME51_nonRTOS
        -fno-exceptions
        -Wundef
        -Wdouble-promotion
        -Werror=return-type
        -Wall
        -Wnoexcept
        -Wshadow
        -Wsign-promo
        -fsingle-precision-constant
        ${_freestanding_options}
        $<$<NOT:$<CONFIG:Debug>>:-O2>
        $<$<CONFIG:Debug>:-Og;-g3>
        ${_mcu_compile_options} # might override default optimisation flags above
        ${ARG_EXTRA_COMPILE_OPTIONS}
    )
    target_link_libraries(${_target} PRIVATE ${ARG_ARCH})
endfunction()
