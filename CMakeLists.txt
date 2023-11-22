

cmake_minimum_required(VERSION 3.10)
project(raisim_examples LANGUAGES CXX)

find_package(raisim CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories (${EIGEN3_INCLUDE_DIRS})
set(file_name example.cc)

add_executable(program ${file_name})
target_link_libraries(program PUBLIC raisim::raisim pthread)
target_include_directories(program PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)


