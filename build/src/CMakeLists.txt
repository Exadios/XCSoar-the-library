set(XCSOAR ${CMAKE_HOME_DIRECTORY})
set(XCSOAR_SRC ${XCSOAR}/src)
set(OUTPUT_INCLUDE ${XCSoar-the-library_BINARY_DIR}/include)
add_subdirectory(linux-32-debug)
add_subdirectory(linux-32-release)

add_custom_target(xcsoar-linux-32
                  DEPENDS xcsoar-linux-32-debug
                          xcsoar-linux-32-release)
add_custom_target(xcsoar-linux   DEPENDS xcsoar-linux-32)
add_custom_target(xcsoar         DEPENDS xcsoar-linux)
add_custom_target(xcsoar-debug   DEPENDS xcsoar-linux-32-debug)
add_custom_target(xcsoar-release DEPENDS xcsoar-linux-32-release)
