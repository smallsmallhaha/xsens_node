cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS -std=c++11)

file(GLOB SRC_LIST "*.cpp" "*.c")

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/../include
	${CMAKE_CURRENT_SOURCE_DIR}/../include/xcommunication
	)

add_library(xcommunication STATIC ${SRC_LIST})
