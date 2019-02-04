set(XCSOAR ${CMAKE_HOME_DIRECTORY})
set(XCSOAR_SRC ${XCSOAR}/src)
set(OUTPUT_INCLUDE ${XCSoar-the-library_BINARY_DIR}/include)
add_subdirectory(linux-native-debug)
add_subdirectory(linux-native-release)

add_custom_target(linux-native
                  DEPENDS linux-native-debug
                          linux-native-release)
