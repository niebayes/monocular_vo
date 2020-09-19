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

# OpenCV 
find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
    message("Found OpenCV")
else()
    message("Not Found OpenCV")
endif()

# Eigen3
find_package(Eigen3 REQUIRED)
if (Eigen3_FOUND)
    message("Found Eigen3")
else()
    message("Not Found Eigen3")
endif()

# g2o 
find_package(G2O REQUIRED)
if (G2O_FOUND)
    message("Found G2O")
else()
    message("Not Found G2O")
endif()

# DBoW3 
find_package(DBoW3 REQUIRED)
if (DBoW3_FOUND)
    message("Found DBoW3")
else()
    message("Not Found DBoW3")
endif()
# set(DBoW3_LIBRARIES "/usr/local/lib/libDBoW3.dylib")

# glog
find_package(Glog REQUIRED)
if (Glog_FOUND)
    message("Found glog")
else()
    message("Not Found glog")
endif()

# gflags
find_package(GFLAGS REQUIRED)
if (GFLAGS_FOUND)
    message("Found gflags")
else() 
    message("Not Found gflags")
endif()

# pcl
find_package(PCL 1.11 REQUIRED COMPONENTS common io visualization)
if (PCL_FOUND)
    message("Found PCL")
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
else() 
    message("Not Found PCL")
endif()


# Add additional include path for header files
include_directories(BEFORE
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/utils
    ${PROJECT_SOURCE_DIR}/src
    ${ARMADILLO_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIR}
    ${DBoW3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
) # path relative to this CMakeLists.txt

add_subdirectory(src)



