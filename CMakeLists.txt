cmake_minimum_required(VERSION 3.18)
project(chrono_drive CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Chrono comes from the build tree of the pinned submodule (see README.md).
find_package(Chrono COMPONENTS Vehicle VSG CONFIG REQUIRED
             PATHS ${CMAKE_SOURCE_DIR}/build-chrono/cmake)

add_executable(chrono_drive src/main.cpp)           # the app
add_executable(suspension_sweep src/sweep.cpp)      # kinematic check of the generated vehicle

foreach(target chrono_drive suspension_sweep)
  target_link_libraries(${target} PRIVATE ${CHRONO_TARGETS})
  # Absolute data paths, so the binary runs from any working directory.
  # (CHRONO_VEHICLE_DATA_DIR is already defined by the Chrono::Chrono_vehicle target.)
  target_compile_definitions(${target} PRIVATE
    "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
    "CHRONO_DRIVE_DATA_DIR=\"${CMAKE_SOURCE_DIR}/data/\"")
endforeach()
