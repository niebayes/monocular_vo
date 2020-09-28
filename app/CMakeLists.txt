project(APP)

add_executable(mono_kitti mono_kitti.cc)
target_compile_options(mono_kitti PRIVATE -O3 -march=native)
target_link_libraries(mono_kitti MY_SLAM_LIB ${LINK_LIBRARIES})

# add_executable(mono_malaga mono_malaga.cc)
# target_compile_options(mono_malaga PRIVATE -O3 -march=native)
# target_link_libraries(mono_malaga ${LINK_LIBRARIES})

# add_executable(mono_parking mono_parking.cc)
# target_compile_options(mono_parking PRIVATE -O3 -march=native)
# target_link_libraries(mono_parking ${LINK_LIBRARIES})