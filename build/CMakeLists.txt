add_subdirectory(tools)
add_subdirectory(resource)
add_subdirectory(generate)
add_subdirectory(src)

add_custom_target(debug DEPENDS xcsoar-debug)
add_custom_target(release DEPENDS xcsoar-release)
add_custom_target(linux DEPENDS xcsoar-linux)

add_custom_target(doc DEPENDS xcsoar_doc)

set(OUTPUT_DIR ${XCSoar-the-library_BINARY_DIR})
set(XCSOAR_SRC ${XCSoar-the-library_SOURCE_DIR}/src)
add_custom_command(OUTPUT ${OUTPUT_DIR}/doc/xcsoar
                   COMMAND mkdir -p ${OUTPUT_DIR/doc/xcsoar})
add_custom_target(xcsoar_doc
                  COMMAND doxygen doc/Doxyfile.xcsoar
                  DEPENDS ${OUTPUT_DIR/doc/xcsoar}
                  WORKING_DIRECTORY ${XCSoar-the-library_SOURCE_DIR})
