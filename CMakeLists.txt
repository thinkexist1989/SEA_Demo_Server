cmake_minimum_required(VERSION 3.10)
project(SEA_Demo_Server)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(cppzmq REQUIRED)
find_package(protobuf REQUIRED)
find_package(spdlog REQUIRED)
find_package(tinyxml2 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_subdirectory(third-party/sml-1.1.11)

set(CMAKE_PREFIX_PATH /opt/rocos/ecm)
find_package(rocos_ecm REQUIRED)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS protos/sea.proto)

include_directories(${CMAKE_SOURCE_DIR}/src)

add_executable(server
        src/main.cpp
        src/sea_control.cpp
        src/interpolate.cc
        src/robotMath.cc
        ${PROTO_SRCS} ${PROTO_HDRS})
target_include_directories(server PUBLIC ${Protobuf_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(server PUBLIC cppzmq protobuf::libprotobuf sml::sml spdlog::spdlog rocos::ecat_config tinyxml2::tinyxml2 yaml-cpp::yaml-cpp)

file(COPY config/sea_config.yml
     DESTINATION ${CMAKE_BINARY_DIR})