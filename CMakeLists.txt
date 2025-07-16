#add_subdirectory(src)

set (SOURCE_FILES
    src/main.cpp
    src/agent.cpp
    src/normal_agent.cpp
    src/mass_agent.cpp
    src/genetic_algorithm.cpp
)

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

find_package(yaml-cpp REQUIRED)

add_executable(mass_colregs ${SOURCE_FILES})
target_include_directories(mass_colregs PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} include)
target_link_libraries(mass_colregs PUBLIC box2d ${PYTHON_LIBRARIES} yaml-cpp)
set_target_properties(mass_colregs PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

target_compile_features(mass_colregs PRIVATE cxx_std_17)