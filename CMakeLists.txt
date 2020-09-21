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
   set(CMAKE_CXX_STANDARD 14)
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
find_package(manif REQUIRED) 
find_package(G2O REQUIRED)
find_package(DBoW3 REQUIRED)
find_package(Glog REQUIRED)
find_package(GFLAGS REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL 1.11 REQUIRED COMPONENTS common io visualization)
if (PCL_FOUND)
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
else() 
    message("Not Found PCL")
endif()

# Add additional include path for header files
include_directories(BEFORE
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${manif_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIR}
    ${DBoW3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${Glog_INCLUDE_DIRS} 
    ${GFLAGS_INCLUDE_DIRS}
) # path relative to this CMakeLists.txt

# Set link libraries
list(APPEND LINK_LIBS
    ${OpenCV_LIBRARIES}
    ${ARMADILLO_LIBRARIES}
    ${PCL_LIBRARIES}
    ${DBoW3_LIBRARIES}
    ${G2O_CORE_LIBRARY}
    ${G2O_STUFF_LIBRARY}
    ${Glog_LIBRARIES}
    ${GFLAGS_LIBRARIES}
)

add_subdirectory(src)
# add_subdirectory(app)



