cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS -std=c++11)

file(GLOB SRC_LIST "*.cpp" "*.c")
list(REMOVE_ITEM SRC_LIST ${CMAKE_CURRENT_SOURCE_DIR}/xstypesdynlib.c)

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/../include
	${CMAKE_CURRENT_SOURCE_DIR}/../include/xsens
	)

add_library(xstypes STATIC ${SRC_LIST})
