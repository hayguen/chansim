cmake_minimum_required(VERSION 2.8)
project(CHANSIM)

# cmake support - just for build

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_EXTENSIONS ON)

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
  src/filter.h
  src/noise.h
  src/rms.h
)

add_executable(chansim  ${CHANSIM_SRCS} ${CHANSIM_HDRS})
target_compile_definitions(chansim PRIVATE _GNU_SOURCE)
target_link_libraries(chansim  m)

# if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
target_compile_options(chansim PRIVATE
  $<$<C_COMPILER_ID:GNU>:-Wall -Wextra -pedantic>
)
