cmake_minimum_required(VERSION 3.10)
project(SEA_Demo_Server)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(cppzmq REQUIRED)
find_package(protobuf REQUIRED)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS protos/sea.proto)

add_executable(server main.cpp ${PROTO_SRCS} ${PROTO_HDRS})
target_include_directories(server PUBLIC ${Protobuf_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(server PUBLIC cppzmq protobuf::libprotobuf)