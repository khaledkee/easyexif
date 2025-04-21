# Copyright © 2020 Andy Maloney <asmaloney@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required( VERSION 3.20 )

project( easyexif
    LANGUAGES
        CXX
)

# Require out-of-source builds
file( TO_CMAKE_PATH "${PROJECT_BINARY_DIR}/CMakeLists.txt" LOC_PATH )

if ( EXISTS "${LOC_PATH}" )
    message( FATAL_ERROR "You cannot build in the source directory. Please use a build subdirectory." )
endif()

add_library( easyexif )

target_compile_features( easyexif
    PRIVATE
        cxx_std_17
)

target_sources( easyexif
    PRIVATE
        exif.cpp
        exif.h
)

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
