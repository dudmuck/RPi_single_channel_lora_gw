
cmake_minimum_required(VERSION 2.6)
project (single_channel)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2")

list(APPEND app_scgw_srcs
    app_scgw.cpp
)

add_subdirectory(i2c_lora_master)

add_executable(app_scgw ${app_scgw_srcs})

add_library(scgw STATIC
    scgw/scgw.cpp
    scgw/lorawan.cpp
    scgw/gladman_cmac.cpp
    scgw/gladman_aes.cpp
)
target_link_libraries(scgw i2c_lora)

include_directories(i2c_lora_master scgw)

target_link_libraries(app_scgw scgw json-c)

####################################################
FIND_PACKAGE(SWIG REQUIRED)
INCLUDE(${SWIG_USE_FILE})

FIND_PACKAGE(PythonLibs)
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

SET(CMAKE_SWIG_FLAGS "")

SET_SOURCE_FILES_PROPERTIES(scgw/scgw.i PROPERTIES CPLUSPLUS ON)
SET_SOURCE_FILES_PROPERTIES(scgw/scgw.i PROPERTIES SWIG_FLAGS "-includeall")
SWIG_ADD_MODULE(scgw python scgw/scgw.i)
SWIG_LINK_LIBRARIES(scgw ${PYTHON_LIBRARIES} scgw i2c_lora)

# add a target to generate API documentation with Doxygen
find_package(Doxygen)
option(BUILD_DOCUMENTATION "Create and install the HTML based API documentation (requires Doxygen)" ${DOXYGEN_FOUND})

if(BUILD_DOCUMENTATION)
    if(NOT DOXYGEN_FOUND)
        message(FATAL_ERROR "Doxygen is needed to build the documentation.")
    endif()

    set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
    set(doxyfile ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    configure_file(${doxyfile_in} ${doxyfile} @ONLY)

    add_custom_target(doc
        COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM)

    install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html DESTINATION share/doc)
endif()

