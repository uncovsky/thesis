cmake_minimum_required( VERSION 3.12 )

project( mo-brtdp )
 
set( CMAKE_CXX_STANDARD 17 )

# default compiler set to gcc
set( CMAKE_CXX_COMPILER "/usr/bin/g++" )

set( CXX_OPTIONS -Wall -Wextra -Werror )
set( CXX_DEBUG_OPTIONS -g )
set( CXX_RELEASE_OPTIONS -O2 )

# set build folder as default destination for generated binaries
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR} )

add_subdirectory( src )
add_subdirectory( tests )


