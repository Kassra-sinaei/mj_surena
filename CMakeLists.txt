cmake_minimum_required(VERSION 3.10)
project(MJ_Surena)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
add_compile_options(-std=c++11)
LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

# find_package(GLEW REQUIRED)
find_package(mujoco REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

find_package(GLEW REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)

include_directories(
    ${glfw3_INCLUDE_DIR}
    include/surena_sim
    ${mujoco_INCLUDE_DIR} 
)

file(GLOB SOURCES "src/*.cpp")

# add_executable(main src/simulation.cpp ${SOURCES})
# target_link_libraries(main ${mujoco_LIBRARIES} ${glfw3_LIBRARIES} ${libglew_LIBRARIES} ${OPENGL_LIBRARIES})
# install(TARGETS main)

add_executable(main src/simulation.cpp ${SOURCES})
target_include_directories(main PRIVATE ${GLFW_INCLUDE_DIRS} ${mujoco_LIBRARIES} ${glfw3_LIBRARIES} ${libglew_LIBRARIES} ${OPENGL_LIBRARIES})
target_link_libraries(main PRIVATE GLEW::GLEW ${GLFW_LIBRARIES} ${mujoco_LIBRARIES} ${glfw3_LIBRARIES} ${libglew_LIBRARIES} ${OPENGL_LIBRARIES})