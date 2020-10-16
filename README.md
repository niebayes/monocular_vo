UZH Robot Perception Course Project
---
## Outline 
This project is the course project of UZH (University of Zurich) Robot Perception Course 2019 Fall. The instructions of the course project can be found at [vo_project_statement.pdf](./vo_project_statement.pdf)

### Course overview
> For a robot to be autonomous, it has to perceive and understand the world around it. This course introduces you to the key computer vision algorithms used in mobile robotics, such as image formation, filtering, feature extraction, multiple view geometry, dense reconstruction, tracking, image retrieval, event-based vision, visual-inertial odometry, Simultaneous Localization And Mapping (SLAM), and some basics of deep learning.

## How does this Visual Odometry work
1. Initialization. 
	1. Select two consecutive frames at the interval of 3 frames to provide enough baseline (e.g. frame 0 and frame 3 may be selected). 
	2. Compute the relative pose encoded in two view geometry with *normalized eight-point algorithm*.  
	3. Triangulate initial map points during checking for good relative pose.  
	4.  A global bundle adjustment is applied to refine both the structure and motion. 
	5. If enough inliers are survived for the global bundle adjustment, the initial map is obtained by selecting both of the frames as keyframes while fixing the reference frame as the datum frame, and collecting all good triangulated map points. If not otherwise, repeat till the criterion is satisfied.
2. **Continuous tracking**  
	1. Initialize the pose of current frame by assuming the *Markov property* and *constant velocity model*.  
	2. Search for 3D-2D matches by projecting the visible map points in initial map onto current frame. 
	3. Apply pose graph optimization to estimate the pose and remove outliers which induce too large reprojection error. 
	4. In order to make the tracking more robust, more 3D-2D matches are searched by projecting all map points observed by locally covisible keyframes onto current frame. Only the unmatched features in current frame are matched against those map points. Again a pose graph optimization is employed to refine the pose estimate. 
	5. If enough number of inliers survived from the pose graph optimization, the tracking is viewed as good and as lost otherwise.
3. **Local mapping**  
	1. Construct covisibility graph with keyframes as nodes and number of covisible map points as weight of edges. The covisibility graph is continuously augmented as a new keyframe is inserted into map. 
	2. Triangulate new map points by projecing all map points observed by covisible keyframes onto current keyframe. Again only the unmatched features in current keyframe are matched against these map points.  
	3. Apply local bundle adjustment to refine the local configuration of keyframes and map points. 
	4. Remove redundant keyframes in covisibility graph if most of the map points they observed are as well observed by others keyframes. This removal makes the map more compact.

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

## How to compile
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

## Future works 
1. Evenly samples keypoints on image and adaptively change threshold to collect enough well distributed features. 
2. Design a more deliberated keyframe selection paradigm.
3. Add a loop closing module. 
4. Use `Valgrind` or other memory profiler to detect and resolve memory leak. 

## References 
1. [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2)
2. [SVO](https://github.com/uzh-rpg/rpg_svo)
3. [Monocular Visual Odometry](https://github.com/felixchenfy/Monocular-Visual-Odometry)
