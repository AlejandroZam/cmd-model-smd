cmake_minimum_required(VERSION 3.15)
project(cmd-model-smd VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Use osk from a parent CMD project (via define_dependency) if already defined,
# otherwise require an installed osk package for standalone builds.
if(NOT TARGET osk)
    find_package(osk 1.0.0 REQUIRED)
endif()

add_library(smd STATIC spring_mass_damper.cpp)
target_include_directories(smd PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(smd PUBLIC osk)

install(TARGETS smd EXPORT smd-targets ARCHIVE DESTINATION lib)
install(FILES spring_mass_damper.h  DESTINATION include)
install(FILES default_params.yaml   DESTINATION share/cmd-model-smd)
