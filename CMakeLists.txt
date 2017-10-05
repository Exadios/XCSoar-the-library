project(XCSoar-the-library)
cmake_minimum_required(VERSION 3.0)

enable_testing()

set_property(GLOBAL PROPERTY JOB_POOLS compile_pool=4)
set_property(GLOBAL PROPERTY JOB_POOL_COMPILE compile_pool)

set(warnings "-Wno-error=unused-parameter -Wall -Wextra -Wwrite-strings -Wcast-qual -Wpointer-arith -Wsign-compare -Wundef -Wmissing-declarations -Wredundant-decls -Wmissing-noreturn -Wno-unused-parameter -Wno-missing-field-initializers  -Wcast-align -Werror")

#if (NOT CONFIGURED_ONCE)
    set(CMAKE_CXX_FLAGS "${warnings} -fno-exceptions -fno-rtti -std=gnu++0x -fno-threadsafe-statics -fmerge-all-constants -fvisibility=hidden -fpic -funwind-tables -finput-charset=utf-8"
        CACHE STRING "Flags used by the compiler during all build types." FORCE)
    set(CMAKE_C_FLAGS   "${warnings}"
        CACHE STRING "Flags used by the compiler during all build types." FORCE)
#endif()

add_definitions(-DRADIANS # Do everything in radians.
                -DUSE_WGS84 # And on WGS84.
               )
add_subdirectory(./build)

set(CONFIGURED_ONCE TRUE CACHE INTERNAL
    "A flag showing that CMake has configured at least once.")


