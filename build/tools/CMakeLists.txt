set(XCSOAR ${CMAKE_HOME_DIRECTORY})
set(TOOLS_DIR ${XCSOAR}/tools)
set(XCSOAR_SRC ${XCSOAR}/src)
set(TOOLS_BIN ${XCSoar-the-library_BINARY_DIR}/tools)
set(OUTPUT_INCLUDE ${XCSoar-the-library_BINARY_DIR}/include)

include_directories(${XCSOAR_SRC})

add_executable(generatesinetables ${TOOLS_DIR}/GenerateSineTables.cpp)

