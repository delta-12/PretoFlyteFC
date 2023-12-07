# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(PretoFlyteFC)

# Find cppcheck in tools/ or /opt/cppcheck/build/bin/
set(CPPCHECK_NAME cppcheck)
set(CPPCHECK_SEARCH_PATH
    ${CMAKE_SOURCE_DIR}/tools/cppcheck/build/bin
    /opt/cppcheck/build/bin/
)
find_program(${CPPCHECK_NAME}_BIN
    NAMES ${CPPCHECK_NAME}
    HINTS ${CPPCHECK_SEARCH_PATH}
)
if(${CPPCHECK_NAME}_BIN)
    message(STATUS "Found ${CPPCHECK_NAME} at: ${${CPPCHECK_NAME}_BIN}")
else()
    message(FATAL_ERROR "${CPPCHECK_NAME} not found.")
endif()

# Add source files and files to include
message(STATUS "Adding main to cppcheck")
set(CPPCHECK_SOURCES ${CMAKE_SOURCE_DIR}/main)
set(CPPCHECK_INCLUDE ${CMAKE_SOURCE_DIR}/main/include)
set(CPPCHECK_COMPONENTS_DIR ${CMAKE_SOURCE_DIR}/components)
message(STATUS "Adding components to cppcheck:")
file(GLOB CPPCHECK_COMPONENT_SUBDIRS RELATIVE ${CPPCHECK_COMPONENTS_DIR} ${CPPCHECK_COMPONENTS_DIR}/*)
foreach(CPPCHECK_COMPONENT_SUBDIR ${CPPCHECK_COMPONENT_SUBDIRS})
    message("     ${CPPCHECK_COMPONENT_SUBDIR}")
    if(IS_DIRECTORY ${CPPCHECK_COMPONENTS_DIR}/${CPPCHECK_COMPONENT_SUBDIR})
        list(APPEND CPPCHECK_SOURCES ${CPPCHECK_COMPONENTS_DIR}/${CPPCHECK_COMPONENT_SUBDIR})
        list(APPEND CPPCHECK_INCLUDE ${CPPCHECK_COMPONENTS_DIR}/${CPPCHECK_COMPONENT_SUBDIR}/include)
    endif()
endforeach()

# Run cppcheck
add_custom_target(
        cppcheck
        ${${CPPCHECK_NAME}_BIN}
        --enable=all
        --suppress-xml=../suppressions.xml
        --error-exitcode=2
        --xml
        --output-file=cppcheck_report.xml
        ${CPPCHECK_SOURCES}
)