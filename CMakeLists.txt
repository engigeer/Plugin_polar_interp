add_library(polar_interp INTERFACE)

target_sources(polar_interp INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}/polar_interp.c
)

target_include_directories(polar_interp INTERFACE ${CMAKE_CURRENT_LIST_DIR})
