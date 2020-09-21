project(src)

# Set source files
list(APPEND SOURCE_FILES
    slam_system.cc 
    front_end_tracking.cc 
    front_end_map_initialization.cc 
    back_end_local_mapping.cc 
    map.cc 
    frame.cc 
    feature.cc 
    camera.cc
    map_point.cc 
    orb.cc
    orb/detector.cc 
    orb/matcher.cc 
    geometry_solver.cc 
    # src/geometry_solver/
    g2o_optimizer.cc 
    g2o_optimizer/global_bundle_adjustment.cc
    g2o_optimizer/pose_graph_optimization.cc
    g2o_optimizer/local_bundle_adjustment.cc
    config.cc 
    dataset.cc 
    viewer.cc 
)

# add_library(my_slam SHARED ${SOURCE_FILES})
# target_compile_options(my_slam PRIVATE -O3 -march=native)
# target_link_libraries(my_slam ${LINK_LIBS})

add_executable(test _test.cc)
target_compile_options(test PRIVATE -O0 -march=native)
target_link_libraries(test)