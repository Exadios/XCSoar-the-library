set(TOOLS_BIN ${XCSoar-the-library_BINARY_DIR}/build/tools)
set(OUTPUT_INCLUDE ${XCSoar-the-library_BINARY_DIR}/include)

add_custom_command(OUTPUT ${OUTPUT_INCLUDE}/MathTables.h
                   COMMAND [ -d ${OUTPUT_INCLUDE} ] || mkdir ${OUTPUT_INCLUDE} 
                   COMMAND ${TOOLS_BIN}/generatesinetables > ${OUTPUT_INCLUDE}/MathTables.h
                   DEPENDS generatesinetables
                   VERBATIM)
set_source_files_properties(${TOOLS_BIN}/generatesinetables
                            PROPERTIES GENERATE TRUE)
add_custom_target(generate ALL
                  DEPENDS ${OUTPUT_INCLUDE}/MathTables.h
                 )
add_dependencies(generate generatesinetables)
