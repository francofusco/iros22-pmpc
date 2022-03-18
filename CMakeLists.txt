project(pmpc-iros)
cmake_minimum_required(VERSION 3.19.4)
add_definitions(--std=c++14)

# Pre-compiled pmpc library for the IROS release
add_library(pmpc_lib INTERFACE)
target_include_directories(pmpc_lib INTERFACE include)
target_link_libraries(pmpc_lib INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/lib/libpmpc_iros.a")

# Eigen is the chosen algebra library
find_package(Eigen3 REQUIRED)
target_include_directories(pmpc_lib INTERFACE ${EIGEN3_INCLUDE_DIRS})

# Needed by Eigen
find_package(OpenMP)
if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
else(OPENMP_FOUND)
  message (ERROR "OpenMP not found")
endif()

# Used to plot results
find_package(Python3 REQUIRED COMPONENTS Development)
target_include_directories(pmpc_lib INTERFACE ${Python3_INCLUDE_DIRS} matplotlibcpp)
target_link_libraries(pmpc_lib INTERFACE Python3::Python)

# This is a quick'n'dirty way of including qpOASES
set(qpOASES_BASE_DIR "$ENV{HOME}/programs/qpOASES")
set(qpOASES_INCLUDE_DIRS "${qpOASES_BASE_DIR}/include")
set(qpOASES_LIBRARIES "${qpOASES_BASE_DIR}/build/libs/libqpOASES.a")
if(EXISTS ${qpOASES_INCLUDE_DIRS} AND EXISTS ${qpOASES_LIBRARIES})
  set(qpOASES_FOUND TRUE)
  message(STATUS "qpOASES found in ${qpOASES_BASE_DIR}")
  target_include_directories(pmpc_lib INTERFACE ${qpOASES_INCLUDE_DIRS})
  target_link_libraries(pmpc_lib INTERFACE ${qpOASES_LIBRARIES})
  target_compile_definitions(pmpc_lib INTERFACE -DPMPC_WITH_QPOASES)
else()
  message(ERROR "qpOASES not found in ${qpOASES_BASE_DIR}")
endif()

target_compile_options(pmpc_lib INTERFACE -O3)
target_compile_definitions(pmpc_lib INTERFACE -DEIGEN_NO_DEBUG)

# Test #1: triple integrator (linear model)
add_executable(triple_integrator src/example_triple_integrator.cpp)
target_link_libraries(triple_integrator pmpc_lib)

# Test #2: cart-pole (nonlinear model)
add_executable(crane_pendulum src/example_crane_pendulum.cpp)
target_link_libraries(crane_pendulum pmpc_lib)

# Test #3: cart-pole with swingup (nonlinear model)
add_executable(crane_pendulum_swingup src/example_crane_pendulum.cpp)
target_link_libraries(crane_pendulum_swingup pmpc_lib)
target_compile_definitions(crane_pendulum_swingup PRIVATE -DSWINGUP)
