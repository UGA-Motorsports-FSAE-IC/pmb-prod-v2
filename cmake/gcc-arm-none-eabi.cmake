set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# Some default GCC settings
# arm-none-eabi- must be part of path environment
set(TOOLCHAIN_PREFIX                arm-none-eabi-)

set(CMAKE_C_COMPILER                ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER              ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_LINKER                    ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_OBJCOPY                   ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_SIZE                      ${TOOLCHAIN_PREFIX}size)

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags.
#
# Hoisted into cache variables so the STM32H7 port is a preset change, not a
# toolchain-file edit.  For example, in CMakePresets.json:
#
#   "cacheVariables": {
#     "PMB_CPU": "cortex-m7",
#     "PMB_FPU": "fpv5-d16",
#     "PMB_LINKER_SCRIPT": "${sourceDir}/STM32H7xx_FLASH.ld"
#   }
set(PMB_CPU "cortex-m0plus" CACHE STRING "Target CPU for -mcpu")
set(PMB_FPU "" CACHE STRING "Target FPU for -mfpu; empty means soft float")
set(PMB_LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/STM32C092XX_FLASH.ld"
    CACHE FILEPATH "Linker script")

set(TARGET_FLAGS "-mcpu=${PMB_CPU} ")
if(PMB_FPU)
    set(TARGET_FLAGS "${TARGET_FLAGS} -mfpu=${PMB_FPU} -mfloat-abi=hard ")
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections -fstack-usage")

# The cyclomatic-complexity parameter must be defined for the Cyclomatic complexity feature in STM32CubeIDE to work.
# However, most GCC toolchains do not support this option, which causes a compilation error; for this reason, the feature is disabled by default.
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fcyclomatic-complexity")

# Debug is the build that runs on the car during a tuning session, so it uses
# -Og rather than -O0: at 48 MHz a 1 kHz control loop plus a 250-byte sprintf
# has no headroom to waste, and -Og keeps the code debuggable.
#
# Release uses -O2 rather than -Os because the control loop is the hot path, and
# keeps -g because DWARF costs nothing in flash and pmbtune.py resolves the
# pmb_params / pmb_log addresses out of the ELF symbol table.
set(CMAKE_C_FLAGS_DEBUG "-Og -g3")
set(CMAKE_C_FLAGS_RELEASE "-O2 -g")
set(CMAKE_CXX_FLAGS_DEBUG "-Og -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${PMB_LINKER_SCRIPT}\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")
