set(XCSOAR ${CMAKE_HOME_DIRECTORY})
set(XCSOAR_SRC ${XCSOAR}/src)
set(OUTPUT_INCLUDE ${XCSoar-the-library_BINARY_DIR}/include)
add_subdirectory(linux-debug)
add_subdirectory(linux-release)

add_custom_target(xcsoar-linux
                  DEPENDS xcsoar-linux-debug
                          xcsoar-linux-release)
add_custom_target(xcsoar         DEPENDS xcsoar-linux)
add_custom_target(xcsoar-debug   DEPENDS xcsoar-linux-debug)
add_custom_target(xcsoar-release DEPENDS xcsoar-linux-release)
