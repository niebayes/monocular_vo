

## Compilation environment
1. OS: 
  - **Ubuntu** (tested version: 18.04 LTS and 20.04 LTS).
  - **macOS**  (tested version: Catalina 10.15.3).
2. Tools: 
  - **CMake** (tested version: 3.10 and 3.18).
  - **g++**   (tested version: 7.5.0 for Ubuntu).
  - **clang** (tested version: 11.0.0 for macOS).
3. Dependencies:
  - **C++17**                                       , GNU version.
  - **Eigen3**    (tested version: 3.3.8)           , for most matrix computations.
  - **Armadillo** (tested version: 9.900.3)         , for loading data into matrix.
  - **Sophus**                                      , for representing pose as SE3.
  - **g2o**       (tested version: 1.14)            , for constructiong and solving BA.
  - **OpenCV**    (tested version: 4.3.0 and 4.4.0) , for drawing figures and image I/O.
  - **PCL**       (tested version: 1.8.1 and 1.11.1), for visualizing camera pose trajectory and map points cloud.
  - **DBoW3**                                       , for converting feature descriptors into bag of words representation and constructing vocabulary tree.
  - **boost**                                       , for formatting string.
  - **Glog**                                        , for general logging.
  - **Gflags**                                      , for parsing command line arguments.
  - **cholmod**                                     , (optional) cholmod solver module dependeny of g2o.

## How to compile.
1. `cd <path_to>/monocular_vo`
2. `mkdir build && cd build` 
3. `cmake -j -DCMAKE_BUILD_TYPE=Release ..` 
4. `make -j`
The compiled binary files will be in the `bin` directory of the root workspace folder. If the memory is run out, you can explicitly specifying the number of parallel jobs by passing `-j<num_par_jobs>` flag to `make` commands rather leaving it to exhaust the memory.

### Issues related to build option
If you've encountered any issue relating memory access, e.g. `double free or corruption (out)`, it's recommended to first check the build options of each libs you've built for this project. It's well pointed out that the vectorization technique Eigen employed raises these issues. 

The solutions may vary, but the list below might be helpful whatsoever: 
1. Consistent build option. Since **Eigen** is extensively used in this project, care has to be taken when building libs. If the build option for one lib is with `BUILD_WITH_MARCH_NATIVE` on while another with that off and they both require Eigen as dependency, conflict is raised and the solution is to rebuild both of'em with the consistent option.
2. Disable Eigen's vectorization technique. By declaring `EIGEN_DONT_VECTORIZE` and / or `EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT` and rebuild corresponding libs, the issues may be resolved, though at the cost of worse performance.
3. To preserve the performance gain introduced by vectorization technique and preclude the issues, you may find it helpful to upgrade **Eigen** to 3.3.9 or higher which completely resolve these issues and free users of declaring `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` each time you'd have to do that before. Since they're not released officially yet, you have to direct to the repository hosted in **GitLab** and mannually select a git revision. Note if you do so, many libs have to be re-compiled.

This project is not tested thoroughly (actually, few of functions were tested).