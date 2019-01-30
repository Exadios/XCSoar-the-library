set(TOOLS_BIN ${XCSoar-the-library_BINARY_DIR}/build/tools)
set(OUTPUT_INCLUDE ${XCSoar-the-library_BINARY_DIR}/include)

add_custom_target(generate ALL
                  COMMAND [ -d ${OUTPUT_INCLUDE} ] || mkdir ${OUTPUT_INCLUDE}
                  DEPENDS ${OUTPUT_INCLUDE}/InputEvents_altair.cpp
                          ${OUTPUT_INCLUDE}/InputEvents_default.cpp
                          ${OUTPUT_INCLUDE}/MathTables.h
                          ${OUTPUT_INCLUDE}/InputEvents_Text2Event.cpp
                          ${OUTPUT_INCLUDE}/InputEvents_Text2GCE.cpp
                          ${OUTPUT_INCLUDE}/InputEvents_Text2GCE.cpp
                          ${OUTPUT_INCLUDE}/InputEvents_Text2NE.cpp
                          ${OUTPUT_INCLUDE}/InputEvents_Char2GCE.cpp
                          ${OUTPUT_INCLUDE}/InputEvents_Char2NE.cpp
                          ${OUTPUT_INCLUDE}/Status_defaults.cpp
                          ${OUTPUT_INCLUDE}/resource.h
                          ${OUTPUT_INCLUDE}/resource_data.h
                          ${OUTPUT_BASE}/resources.c
                          ${OUTPUT_SRC}/${BOOST_BASE_NAME}
                  VERBATIM)

add_dependencies(generate tools)

add_custom_command(OUTPUT ${OUTPUT_INCLUDE}/MathTables.h
                   COMMAND [ -d ${OUTPUT_INCLUDE} ] || mkdir ${OUTPUT_INCLUDE} 
                   COMMAND ${TOOLS_BIN}/generatesinetables > ${OUTPUT_INCLUDE}/MathTables.h || rm ${OUTPUT_INCLUDE}/MathTables.h
                   DEPENDS generatesinetables
                   VERBATIM)
set_source_files_properties(${TOOLS_BIN}/generatesinetables
                            PROPERTIES GENERATE TRUE)

add_custom_command(OUTPUT ${OUTPUT_INCLUDE}/InputEvents_altair.cpp
                   COMMAND [ -d ${OUTPUT_INCLUDE} ] || mkdir ${OUTPUT_INCLUDE}
                   COMMAND perl ${TOOLS_DIR}/xci2cpp.pl ${XCSOAR}/Data/Input/default.xci ${XCSOAR}/Data/Input/altair.xci > ${OUTPUT_INCLUDE}/InputEvents_altair.cpp
                   DEPENDS ${TOOLS_DIR}/xci2cpp.pl)

add_custom_command(OUTPUT ${OUTPUT_INCLUDE}/InputEvents_default.cpp
                   COMMAND [ -d ${OUTPUT_INCLUDE} ] || mkdir ${OUTPUT_INCLUDE}
                   COMMAND perl ${TOOLS_DIR}/xci2cpp.pl ${XCSOAR}/Data/Input/default.xci > ${OUTPUT_INCLUDE}/InputEvents_default.cpp
                   DEPENDS ${TOOLS_DIR}/xci2cpp.pl)

add_custom_command(OUTPUT ${OUTPUT_INCLUDE}/InputEvents_Text2Event.cpp
                   COMMAND [ -d ${OUTPUT_INCLUDE} ] || mkdir ${OUTPUT_INCLUDE}
                   COMMAND perl ${TOOLS_DIR}/Text2Event.pl ${XCSOAR_SRC}/Input/InputEvents.hpp > ${OUTPUT_INCLUDE}/InputEvents_Text2Event.cpp
                   DEPENDS ${TOOLS_DIR}/Text2Event.pl)

add_custom_command(OUTPUT ${OUTPUT_INCLUDE}/InputEvents_Text2GCE.cpp
                   COMMAND [ -d ${OUTPUT_INCLUDE} ] || mkdir ${OUTPUT_INCLUDE}
                   COMMAND perl ${TOOLS_DIR}/Text2GCE.pl ${XCSOAR_SRC}/Input/InputQueue.hpp > ${OUTPUT_INCLUDE}/InputEvents_Text2GCE.cpp
                   DEPENDS ${TOOLS_DIR}/Text2GCE.pl)

add_custom_command(OUTPUT ${OUTPUT_INCLUDE}/InputEvents_Text2NE.cpp
                   COMMAND [ -d ${OUTPUT_INCLUDE} ] || mkdir ${OUTPUT_INCLUDE}
                   COMMAND perl ${TOOLS_DIR}/Text2NE.pl ${XCSOAR_SRC}/Input/InputQueue.hpp > ${OUTPUT_INCLUDE}/InputEvents_Text2NE.cpp
                   DEPENDS  ${TOOLS_DIR}/Text2NE.pl)

add_custom_command(OUTPUT ${OUTPUT_INCLUDE}/InputEvents_Char2GCE.cpp
                   COMMAND [ -d ${OUTPUT_INCLUDE} ] || mkdir ${OUTPUT_INCLUDE}
                   COMMAND perl ${TOOLS_DIR}/Char2GCE.pl < ${XCSOAR_SRC}/Input/InputQueue.hpp > ${OUTPUT_INCLUDE}/InputEvents_Char2GCE.cpp.tmp
                   COMMAND mv ${OUTPUT_INCLUDE}/InputEvents_Char2GCE.cpp.tmp ${OUTPUT_INCLUDE}/InputEvents_Char2GCE.cpp
                   DEPENDS ${TOOLS_DIR}/Char2GCE.pl)

add_custom_command(OUTPUT ${OUTPUT_INCLUDE}/InputEvents_Char2NE.cpp
                   COMMAND [ -d ${OUTPUT_INCLUDE} ] || mkdir ${OUTPUT_INCLUDE}
                   COMMAND perl ${TOOLS_DIR}/Char2NE.pl < ${XCSOAR_SRC}/Input/InputQueue.hpp > ${OUTPUT_INCLUDE}/InputEvents_Char2NE.cpp.tmp
                   COMMAND mv ${OUTPUT_INCLUDE}/InputEvents_Char2NE.cpp.tmp ${OUTPUT_INCLUDE}/InputEvents_Char2NE.cpp
                   DEPENDS ${TOOLS_DIR}/Char2NE.pl)

add_custom_command(OUTPUT ${OUTPUT_INCLUDE}/Status_defaults.cpp
                   COMMAND [ -d ${OUTPUT_INCLUDE} ] || mkdir ${OUTPUT_INCLUDE}
                   COMMAND perl ${TOOLS_DIR}/xcs2cpp.pl ${XCSOAR}/Data/Status/default.xcs > ${OUTPUT_INCLUDE}/Status_defaults.cpp
                   DEPENDS ${TOOLS_DIR}/xcs2cpp.pl)

add_custom_command(OUTPUT ${OUTPUT_INCLUDE}/XCSoar.rc
                   COMMAND [ -d ${OUTPUT_INCLUDE} ] || mkdir ${OUTPUT_INCLUDE}
                   COMMAND cat "${XCSOAR}/Data/XCSoar.rc" | gcc -E -I${OUTPUT_BASE}/include -o  ${OUTPUT_INCLUDE}/XCSoar.rc -
                   DEPENDS ${OUTPUT_INCLUDE}/resource.h)

add_custom_command(OUTPUT ${OUTPUT_INCLUDE}/resource.h
                   COMMAND [ -d ${OUTPUT_INCLUDE} ] || mkdir ${OUTPUT_INCLUDE}
                   COMMAND cat ${XCSOAR_SRC}/Resources.hpp | ${TOOLS_DIR}/GenerateOldResources > ${OUTPUT_INCLUDE}/resource.h.tmp
                   COMMAND mv ${OUTPUT_INCLUDE}/resource.h.tmp ${OUTPUT_INCLUDE}/resource.h
                   )

if(NOT SUPPRESS_RESOURCES_GENERATION)
add_custom_command(OUTPUT ${OUTPUT_INCLUDE}/resource_data.h
                   COMMAND ${TOOLS_DIR}/GenerateResources.pl < ${OUTPUT_INCLUDE}/XCSoar.rc > ${OUTPUT_INCLUDE}/resource_data.h.tmp
                   COMMAND mv ${OUTPUT_INCLUDE}/resource_data.h.tmp ${OUTPUT_INCLUDE}/resource_data.h
                   DEPENDS ${RESOURCE_FILES}
                  )

add_custom_command(OUTPUT ${OUTPUT_BASE}/resources.c
                   COMMAND perl -I${TOOLS_DIR} ${TOOLS_DIR}/LinkResources.pl ${OUTPUT_INCLUDE}/XCSoar.rc ${OUTPUT_INCLUDE}/resources.c
                   DEPENDS ${OUTPUT_INCLUDE}/XCSoar.rc
                   DEPENDS ${OUTPUT_INCLUDE}/resource.h
                   DEPENDS ${TOOLS_DIR}/LinkResources.pl
                   DEPENDS ${TOOLS_DIR}/BinToC.pm)
endif(NOT SUPPRESS_RESOURCES_GENERATION)

set(DOWNLOAD ${XCSoar_BIN_DIR}/download)
set(BOOST_TARBALL "boost_1_65_1.tar.bz2")
set(BOOST_URL https://dl.bintray.com/boostorg/release/1.65.1/source/${BOOST_TARBALL})
set(BOOST_ALTERNATIVE_URL https://fossies.org/linux/misc/${BOOST_TARBALL})
set(BOOST_MD5 9807a5d16566c57fd74fb522764e0b134a8bbe6b6e8967b83afefd30dcd3be81)
string(REPLACE ".tar.bz2" "" BOOST_BASE_NAME ${BOOST_TARBALL})
add_custom_command(OUTPUT ${DOWNLOAD}/${BOOST_TARBALL}
                   COMMAND [ -d ${DOWNLOAD} ] || mkdir -p ${DOWNLOAD}
                   COMMAND ${XCSOAR}/build/download.py ${BOOST_URL} ${BOOST_ALTERNATIVE_URL} ${BOOST_MD5} ${DOWNLOAD})
add_custom_command(OUTPUT ${OUTPUT_SRC}/${BOOST_BASE_NAME}}
                   COMMAND [ -d ${OUTPUT_SRC}/${BOOST_BASE_NAME} ] || mkdir -p $${OUTPUT_SRC}/${BOOST_BASE_NAME}
                   COMMAND tar xjfC ${DOWNLOAD}/${BOOST_TARBALL} ${OUTPUT_SRC}
                   DEPENDS ${DOWNLOAD}/${BOOST_TARBALL})

set(BOOST_CPPFLAGS -isystem ${OUTPUT_SRC}/${BOOST_BASE_NAME} -DBOOST_NO_IOSTREAM -DBOOST_MATH_NO_LEXICAL_CAST -DBOOST_UBLAS_NO_STD_CERR -DBOOST_ERROR_CODE_HEADER_ONLY -DBOOST_SYSTEM_NO_DEPRECATED -DBOOST_NO_STD_LOCALE -DBOOST_LEXICAL_CAST_ASSUME_C_LOCALE 
    PARENT_SCOPE)
