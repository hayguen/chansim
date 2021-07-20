cmake_minimum_required(VERSION 2.8)
project(CHANSIM)

option(DISABLE_LINK_WITH_M "Disables linking with m library to build with clangCL from MSVC" OFF)


set(CMAKE_C_STANDARD 99)
set(CMAKE_C_EXTENSIONS ON)

########################################################################
# select the release build type by default to get optimization flags
########################################################################
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Release")
   message(STATUS "Build type not specified: defaulting to release.")
endif(NOT CMAKE_BUILD_TYPE)

if ( CMAKE_C_COMPILER_ID MATCHES "MSVC" )
  # using Visual Studio C++
  message(STATUS "INFO: detected MSVC: will not link math lib m")
  set(MATHLIB "")

  # add_definitions("/D_CRT_SECURE_NO_WARNINGS")
  # set(MSVC_DISABLED_WARNINGS_LIST "C4996")
else()
  if(DISABLE_LINK_WITH_M)
  else()
    message(STATUS "INFO: detected NO MSVC: ${CMAKE_C_COMPILER_ID}: will link math lib m")
    set(MATHLIB "m")
  endif()
endif()


set(CHANSIM_SRCS
  src/delay.c
  src/fade.c
  src/filter.c
  src/main.c
  src/noise.c
  src/rms.c
)

set(CHANSIM_HDRS
  src/chansim.h
  src/cplx.h
  src/filter.h
  src/noise.h
  src/rms.h
)

add_executable(chansim  ${CHANSIM_SRCS} ${CHANSIM_HDRS})
target_compile_definitions(chansim PRIVATE _GNU_SOURCE)
if (WIN32 OR MINGW)
  message(WARNING "Soundcard is not supported on Windows or MINGW")
else()
  target_compile_definitions(chansim PRIVATE USE_SOUND)
endif()
target_link_libraries(chansim  ${MATHLIB})

# if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
target_compile_options(chansim PRIVATE
  $<$<C_COMPILER_ID:GNU>:-Wall -Wextra -pedantic>
)
