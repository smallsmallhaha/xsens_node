cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_CXX_FLAGS -std=c++11)

file(GLOB SRC_LIST "*.cpp" "*.c")

add_subdirectory(xcommunication)
add_subdirectory(xstypes)

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/include
	# include
	${catkin_INCLUDE_DIRS}
	)

add_executable(xsens_node ${SRC_LIST})
target_link_libraries(xsens_node xcommunication xstypes ${catkin_LIBRARIES})

install(TARGETS xsens_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
