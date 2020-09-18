project(map_creator)

cmake_minimum_required(VERSION 3.10)

find_package(cartographer REQUIRED)

include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
option(BUILD_GRPC "build features that require Cartographer gRPC support" false)
google_initialize_cartographer_project()

add_compile_options(-w)

file(GLOB MAP_TEST_SRC "*.cpp")
add_compile_options(-g)

add_executable(map_test ${MAP_TEST_SRC})

target_link_libraries(map_test ${catkin_LIBRARIES} ${PROTOBUF_LIBRARY} cartographer)
add_dependencies(map_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} cartographer)
get_target_property(INCLUDE_CARTO cartographer INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(map_test PUBLIC ${INCLUDE_CARTO})
