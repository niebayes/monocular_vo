project(SOURCE)

add_executable(main main.cc)
target_compile_options(main PRIVATE -O3 -march=native)
target_link_libraries(main 
  ${OpenCV_LIBRARIES}
  ${GlOG_LIBRARIES}
  ${ARMADILLO_LIBRARIES}
  ${PCL_LIBRARIES}
  ${DBoW3_LIBRARIES}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
)