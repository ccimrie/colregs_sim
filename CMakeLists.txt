cmake_minimum_required(VERSION 3.10)

project(seams25)

## Setting up libraries
find_package(PythonLibs REQUIRED)

include_directories(${PYTHON_INCLUDE_DIRS})
find_package(yaml-cpp REQUIRED)

## Setting up Box2D library
if(NOT BOX2D_ROOT_DIR)
    message(FATAL_ERROR "Environment variable BOX2D_ROOT_DIR is not defined")
endif()

add_library(box2dlib SHARED IMPORTED GLOBAL)
set_target_properties(box2dlib PROPERTIES IMPORTED_LOCATION ${BOX2D_ROOT_DIR}/build/src/libbox2d.a )


set (SOURCE_FILES
    src/main.cpp
    src/agent.cpp
    src/normal_agent.cpp
    src/mass_agent.cpp
    src/csv_functions.cpp
    src/vision_unit.cpp
)
add_executable(test ${SOURCE_FILES})
target_include_directories(test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${BOX2D_ROOT_DIR}/include/box2d ${BOX2D_ROOT_DIR}/src)

target_include_directories(test PUBLIC ${BOX2D_ROOT_DIR}/include ${BOX2D_ROOT_DIR}/src)
target_link_libraries(test PUBLIC ${PYTHON_LIBRARIES} ${Python3_LIBRARIES} yaml-cpp box2dlib)

set_target_properties(test PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

target_compile_features(test PRIVATE cxx_std_17)