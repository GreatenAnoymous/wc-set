cmake_minimum_required(VERSION 3.16)
project(mapf_python)

find_package(Boost COMPONENTS graph REQUIRED)
include_directories( ${Boost_INCLUDE_DIRS} )


# find_package(Python 3.6 COMPONENTS Interpreter Development REQUIRED)
find_package(Python3 3.6 EXACT REQUIRED COMPONENTS Interpreter Development)
find_package(PythonInterp 3.6 EXACT REQUIRED)
find_package(pybind11 REQUIRED)
set(CMAKE_CXX_STANDARD 20 CACHE STRING "C++ version selection")  # or 11, 14, 17, 20

# set(CMAKE_BUILD_TYPE Debug)
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF()
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0  -g2 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3")

include_directories("./cpp/grid_pathfinding")
include_directories("./cpp/mapf")
include_directories("./cpp/MSVC")

# file(GLOB A_files A/*.cpp)

ADD_LIBRARY(GRAPHLIB SHARED 
            "./cpp/grid_pathfinding/graph.cpp"
            "./cpp/grid_pathfinding/node.cpp"
            "./cpp/grid_pathfinding/pos.cpp")
          
            
file(GLOB LACAM_FILES "./cpp/lacam/*.cpp")
file(GLOB MAPF_FILES 
    "./cpp/mapf/paths.cpp"
    "./cpp/mapf/solver.cpp"
    "./cpp/mapf/plan.cpp"
    "./cpp/mapf/problem.cpp"
    "./cpp/mapf/hca.cpp"
    "./cpp/mapf/lib_cbs.cpp"
    "./cpp/mapf/ecbs.cpp"
    "./cpp/mapf/maxflow.cpp"
    "./cpp/mapf/pbs.cpp"
    "./cpp/mapf/pibt.cpp"
    "./cpp/mapf/pibt_complete.cpp"
    "./cpp/unlabeled/lib_ga.cpp"
    "./cpp/unlabeled/tswap.cpp"
    "./cpp/mapf/push_and_swap.cpp"
    "./cpp/unlabeled/goal_allocator.cpp"
)            
set(LIBMAPFCPP ${MAPF_FILES} ${LACAM_FILES})
ADD_LIBRARY(MAPFLIB SHARED ${LIBMAPFCPP} )


set(ORTOOLS_LIB "/usr/local/lib/libortools.so")
set(GUROBI_INC /opt/gurobi912/linux64/include/)
# set(GUROBI_LIB "/opt/gurobi912/linux64/lib/libgurobi_c++.a" "/opt/gurobi912/linux64/lib/libgurobi91.so")

# pybind11_add_module(Grids "./cpp/grid_pathfinding/wrapper.cpp" 
#                         "./cpp/grid_pathfinding/graph.cpp"
#                         "./cpp/grid_pathfinding/node.cpp"
#                         "./cpp/grid_pathfinding/pos.cpp")
pybind11_add_module(Grids  "./cpp/grid_pathfinding/wrapper.cpp")
target_link_libraries(Grids PRIVATE GRAPHLIB)

pybind11_add_module(MAPF "./cpp/mapf/wrapper.cpp") 
target_link_libraries(MAPF PRIVATE MAPFLIB GRAPHLIB ${ORTOOLS_LIB})                     


pybind11_add_module(MSVC "./cpp/MSVC/msvc.cpp" "./cpp/MSVC/wrapper.cpp" "./cpp/MSVC/unpp.cpp")
target_link_libraries(MSVC PRIVATE GRAPHLIB MAPFLIB ${Boost_LIBRARIES})




# add_executable(main "./cpp/mapf/test.cpp")
# target_link_libraries(main PRIVATE GRAPHLIB MAPFLIB ${ORTOOLS_LIB})

add_executable(main "./cpp/MSVC/msvc.cpp" "./cpp/MSVC/test.cpp")
target_link_libraries(main PRIVATE GRAPHLIB)

