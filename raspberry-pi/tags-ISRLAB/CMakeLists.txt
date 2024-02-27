
cmake_minimum_required(VERSION 3.25)

set(TARGET main)

project(tags-ISRLAB)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(eclipse-paho-mqtt-c REQUIRED)
find_package(PahoMqttCpp REQUIRED)
find_package(Boost 1.84.0 REQUIRED COMPONENTS json) 



add_library(libcamera-base SHARED IMPORTED)
add_library(libcamera SHARED IMPORTED)
add_library(libevent SHARED IMPORTED)
add_library(libevent-core SHARED IMPORTED)

set_target_properties(libcamera-base PROPERTIES IMPORTED_LOCATION /usr/lib/aarch64-linux-gnu/libcamera-base.so)
set_target_properties(libcamera PROPERTIES IMPORTED_LOCATION /usr/lib/aarch64-linux-gnu/libcamera.so)
set_target_properties(libevent PROPERTIES IMPORTED_LOCATION /usr/lib/aarch64-linux-gnu/libevent_pthreads.so)
set_target_properties(libevent-core PROPERTIES IMPORTED_LOCATION /usr/lib/aarch64-linux-gnu/libevent_core.so)


find_library(APRILTAGS apriltags ${PROJECT_SOURCE_DIR}/build/lib)



file(GLOB SRC "src/*.h" "src/*.cpp")
MESSAGE("-- Build type: ${CMAKE_BUILD_TYPE}")

add_executable(${TARGET} ${SRC})
target_include_directories( ${TARGET} PRIVATE ${PROJECT_SOURCE_DIR}/build/include/)
target_include_directories( ${TARGET} PRIVATE /usr/include/libcamera)


target_link_libraries(${TARGET} 
    ${OpenCV_LIBS} 
    Eigen3::Eigen
    ${APRILTAGS}
    PahoMqttCpp::paho-mqttpp3
    ${Boost_LIBRARIES}
    libcamera-base
    libcamera
    libevent-core
    libevent
)



