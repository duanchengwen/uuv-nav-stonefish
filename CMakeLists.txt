cmake_minimum_required(VERSION 3.16)
project(uuv_charge_nav LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Stonefish REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(Threads REQUIRED)

set(UUV_NAV_SOURCES
    src/main.cpp
    src/NavSimulationManager.cpp
    src/Autopilot.cpp
    src/DataRecorder.cpp
    src/EpisodeConfig.cpp
)

add_executable(uuv_nav ${UUV_NAV_SOURCES})
target_link_libraries(uuv_nav PRIVATE Stonefish::Stonefish ${OpenCV_LIBS} Threads::Threads)
target_compile_options(uuv_nav PRIVATE -O3 -Wall)
target_compile_definitions(uuv_nav PRIVATE DATA_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}/data/")
