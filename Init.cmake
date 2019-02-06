find_program(CMAKE_C_COMPILER NAMES gcc cc)
find_program(CMAKE_CXX_COMPILER NAMES g++ c++)
set(CMAKE_CXX_FLAGS "-Wno-error=unused-parameter -Wall -Wextra -Wwrite-strings -Wcast-qual -Wpointer-arith -Wsign-compare -Wundef -Wmissing-declarations -Wredundant-decls -Wmissing-noreturn -Wno-unused-parameter -Wno-missing-field-initializers  -Wcast-align -Werror -fno-exceptions -fno-rtti -std=gnu++14 -fno-threadsafe-statics -fmerge-all-constants -fvisibility=hidden -fpic -funwind-tables -finput-charset=utf-8" CACHE STRING "Flags used by the compiler during all build types" FORCE)
