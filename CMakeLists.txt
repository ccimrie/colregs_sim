cmake_minimum_required(VERSION 4.0)

project(MultiAgentSim)

## Setting up libraries
# message("INCLUDE PATH:  ${YAML_CPP_INCLUDE_DIR}")
# # #find_library(libyaml-cpp yaml-cpp)
# message("CMAKE prefix is: "  "${CMAKE_PREFIX_PATH}")
# message("yaml-cpp path is: " "${YAML_CPP_ROOT_DIR}")
# list(APPEND CMAKE_PREFIX_PATH "${YAML_CPP_ROOT_DIR}")
# message("${CMAKE_PREFIX_PATH}")
# message("Current list dir is:  ${CMAKE_CURRENT_LIST_DIR}")
# message("INCLUDE PATH:  ${YAML_CPP_INCLUDE_DIR}")
# set(YAML_CPP_INCLUDE_DIR "${YAML_CPP_ROOT_DIR}/include" CACHE PATH "yaml-cpp include dir" FORCE)
find_package(yaml-cpp CONFIG REQUIRED HINTS ${YAML_CPP_ROOT_DIR})

## Setting up Box2D library
#list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/my_packages")
add_library(box2dlib SHARED IMPORTED GLOBAL)
set_target_properties(box2dlib PROPERTIES IMPORTED_LOCATION ${BOX2D_ROOT_DIR}/build/src/libbox2d.a)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(${EIGEN_INCLUDE_DIR})
find_package (osqp REQUIRED NO_MODULE)
include_directories(${OSQP_INCLUDE_DIR})

set(SOURCE_FILES
    src/main.cpp
    src/agent.cpp
    src/normal_agent.cpp
    src/mass_agent.cpp
    src/csv_functions.cpp
    src/vision_unit.cpp
    # src/nlmpc.cpp
)
add_executable(test ${SOURCE_FILES})

target_include_directories(test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(test PUBLIC ${BOX2D_ROOT_DIR}/include/box2d ${BOX2D_ROOT_DIR}/src ${BOX2D_ROOT_DIR}/include)
message("MPC lib path:  ${MPC_ROOT_DIR}")
target_include_directories(test PUBLIC ${MPC_ROOT_DIR}/include)


target_link_libraries(test PUBLIC 
    ${PYTHON_LIBRARIES} 
    ${Python3_LIBRARIES} 
    yaml-cpp::yaml-cpp 
    box2dlib 
    Eigen3::Eigen 
    osqp::osqp
    nlopt
)

set_target_properties(test PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS NO
)

target_compile_features(test PRIVATE cxx_std_20)


# message("Found yaml-cpp!")

#add_library(yamlcpplib SHARED IMPORTED GLOBAL)
#set_target_properties(yamlcpplib PROPERTIES IMPORTED_LOCATION ${YAML_CPP_ROOT_DIR}/build/libyaml-cpp.so)

#### Using github/dependecies approach ####
# include(FetchContent)

# FetchContent_Declare(
#   yaml-cpp
#   GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
#   GIT_TAG master # Can be a tag (yaml-cpp-x.x.x), a commit hash, or a branch name (master)
# )
# FetchContent_MakeAvailable(yaml-cpp)

# #target_link_libraries(YOUR_LIBRARY PUBLIC yaml-cpp::yaml-cpp) # The library or executable that require yaml-cpp library

# find_package(Python3 REQUIRED)
# include_directories(${PYTHON_INCLUDE_DIRS})
# message("Pointing to ${BOX2D_ROOT_DIR}/include/box2d and ${BOX2D_ROOT_DIR}/src")
# target_include_directories(test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${BOX2D_ROOT_DIR}/include/box2d ${BOX2D_ROOT_DIR}/src)

# #target_include_directories(test PUBLIC ${BOX2D_ROOT_DIR}/include ${BOX2D_ROOT_DIR}/src)
# target_link_libraries(test PUBLIC ${PYTHON_LIBRARIES} ${Python3_LIBRARIES} yamlcpplib box2dlib)
