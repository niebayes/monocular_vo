project(SRC)

# Set source files
list(APPEND SOURCE_FILES
    system.cc 
    front_end_tracking.cc 
    front_end_map_initialization.cc 
    back_end_local_mapping.cc 
    map.cc 
    frame.cc 
    map_point.cc 
    camera.cc
    matcher.cc
    geometry_solver.cc 
    g2o_optimizer.cc 
    keyframe_database.cc
    config.cc 
    dataset.cc 
    viewer.cc 
)

add_library(MY_SLAM_LIB SHARED ${SOURCE_FILES})
target_compile_options(MY_SLAM_LIB PRIVATE -O3 -march=native)
target_link_libraries(MY_SLAM_LIB ${LINK_LIBRARIES})

# add_executable(my_test my_test.cc)
# target_compile_options(my_test PRIVATE -O0 -march=native)
# target_link_libraries(my_test ${LINK_LIBRARIES})

# add_executable(test1 test1.cc)
# target_compile_options(test1 PRIVATE -O0 -march=native)
# target_link_libraries(test1 ${LINK_LIBRARIES})