cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(BASIC_MONOCULAR_SLAM)

# Configure compile mode: Debug, Release, etc.
# set(CMAKE_BUILD_TYPE Release)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable C++17 standard
# Check if the compiler supports C++17
# DBoW3 uses some features which have been expired since C++17
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
if(COMPILER_SUPPORTS_CXX17)
   message(STATUS "Using flag -std=c++17.")
   set(CMAKE_CXX_STANDARD 17)
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif()

# Configure compile output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/lib)

# Add cmake module path 
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(manif REQUIRED) 
find_package(Sophus REQUIRED)
find_package(CHOLMOD REQUIRED)
find_package(G2O REQUIRED)
find_package(DBoW3 REQUIRED)
find_package(GLOG REQUIRED)
find_package(GFLAGS REQUIRED)
find_package(Pangolin REQUIRED)
# find_package(PCL 1.11 REQUIRED COMPONENTS common io visualization)
# if (PCL_FOUND)
#     link_directories(${PCL_LIBRARY_DIRS})
#     add_definitions(${PCL_DEFINITIONS})
# endif()

# Add additional include path for header files
include_directories(BEFORE
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/mono_slam
    ${OpenCV_INCLUDE_DIRS}
    # ${manif_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
    # ${PCL_INCLUDE_DIRS}
    /usr/local/include/g2o/solvers/slam2d_linear/
    ${CHOLMOD_INCLUDE_DIR}
    ${G2O_INCLUDE_DIR}
    ${DBoW3_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS} 
    ${GFLAGS_INCLUDE_DIRS}
) # path relative to this CMakeLists.txt

# Set link libraries
list(APPEND LINK_LIBRARIES
    ${OpenCV_LIBRARIES}
    ${ARMADILLO_LIBRARIES}
    Sophus::Sophus
    ${PCL_LIBRARIES}
    ${DBoW3_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${CHOLMOD_LIBRARIES}
    ${G2O_CORE_LIBRARY}
    ${G2O_STUFF_LIBRARY}
    ${G2O_SOLVER_CHOLMOD}
    ${G2O_TYPES_SBA} 
    cholmod 
    # ${G2O_SOLVER_CSPARSE}
    # csparse
    ${GLOG_LIBRARIES}
    ${GFLAGS_LIBRARIES}
)

# Set source files
list(APPEND SOURCE_FILES
    src/system.cc 
    src/tracking.cc 
    src/initialization.cc 
    src/local_mapping.cc 
    src/map.cc 
    src/frame.cc 
    src/map_point.cc 
    src/camera.cc
    src/matcher.cc
    src/geometry_solver.cc 
    src/geometry_solver/kneip_p3p.cc
    src/g2o_optimizer.cc 
    src/config.cc 
    src/dataset.cc 
    src/viewer.cc 
)

add_library(my_slam SHARED ${SOURCE_FILES})
# target_compile_options(my_slam PRIVATE -O0)
target_compile_options(my_slam PRIVATE -O0)
target_link_libraries(my_slam ${LINK_LIBRARIES})

add_executable(mono_kitti app/mono_kitti.cc)
# target_compile_options(mono_kitti PRIVATE -O0)
target_compile_options(my_slam PRIVATE -O0)
target_link_libraries(mono_kitti my_slam ${LINK_LIBRARIES})

# add_executable(my_test src/my_test.cc)
# target_compile_options(my_test PRIVATE -O3 -march=native)
# target_link_libraries(my_test ${LINK_LIBRARIES})



