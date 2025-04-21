# Copyright © 2020 Andy Maloney <asmaloney@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required( VERSION 3.20 )

project( easyexif
    LANGUAGES
        CXX
)

add_library( easyexif SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/include/easyexif/exif.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/easyexif/exports.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/easyexif/exif.cpp
)

# generate exports header file
include(GenerateExportHeader)
generate_export_header(easyexif EXPORT_FILE_NAME  ${CMAKE_CURRENT_SOURCE_DIR}/include/easyexif/exports.h)

# include directories
target_include_directories( easyexif PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories( easyexif PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
set_target_properties( easyexif PROPERTIES FOLDER "")

# prevent custom compiler extensions to ensure cross-compilation
target_compile_features( easyexif PUBLIC cxx_std_17)
set_target_properties( easyexif PROPERTIES CXX_EXTENSIONS OFF )

# install command
install(TARGETS easyexif LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# DEMO usage project
add_executable( easyexif_demo EXCLUDE_FROM_ALL )

target_compile_features( easyexif_demo
    PRIVATE
        cxx_std_17
)

target_sources( easyexif_demo
    PRIVATE
        demo.cpp
)

target_link_libraries( easyexif_demo
    PRIVATE
        easyexif
)
