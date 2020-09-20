# Initialization
1. Why: Mono-SLAM needs two frames to bootstrap the tracking because there's no direct ways for Mono-SLAM to get initial map (which contains initial map points) without which the SLAM system cannot proceed.
2. Focus: create the initial map.
3. By the way: track the current frame based on the last frame to estimate the relative pose T_curr_last and fix the position of the last frame as the world frame.

## Logic
1. Since the initialization needs two frames and the tracker grabs image frame by frame, an **initializer** should be implemented to hold the last frame as the initial frame which will be used as the reference frame when requesting initialization. Hence two data members should be contained: *init_frame* and *curr_frame*.
2. To initialize, generally there're two methods: recover the pose from planar / nearly planer scene using homography or from non-planar scene using fundamental. For the sake of simplicity and generalizability, fundamental is selected. Of course, the computation of the fundamental matrix should be independent of the system, thus a **Geometry** class should be implemented and its methods should be *static* (i.e. it's simply a function wrapper).
3. To compute the fundamental matrix, at least eight point correspondences should be obtained among the matched keypoints. Hence, *RANSAC* scheme should be applied to reject outliers. To mark the remained inliers survived from RANSAC, an *inlier_mask* should be maintained. This mask can help the triangulation during creating the initial map.
4. At this point, we have inlier matched 2D keypoints and camera poses for the first two frames computed from above as well as the camera intrinsics, so it's time to back-project the 2D keypoints to get the 3D scene points (aka. triangulation).
5. After triangulation, we're ready to create an initial map. Hence, a **map** class should be implemented in advance. Since the **frame** class is just a wrapper of the original image object, the **map** class only contains **keyframe** objects and the triangulated map points (hence a **map_point** class should also be implemented in advance). This could limit the growth of the system and speed up image quering when requested. 
6. To create the initial map, the main workload is on the linking among the 2D keypoints (i.e. observations), 3D map points, keyframes, map object and the initialization of these objects.
7. We now have the "structrue and motion", it's recommended to apply a global bundle adjustment to optimize them jointly. Hence, an **optimizer** class should should be implemented and its methods should be *static* (i.e. it's simply a function wrapper).
8. To conclude, the **initializer** should be implemented as below: 
   1. Data members: 
      1. *init_frame*. 
      2. *curr_frame*. 
      3. *tracker* to communicate with the tracker (e.g. get and set the *init_frame* and *curr_frame*, communicate with the **map** object, communicate with **local_mapper** object, set other stuff etc.). 
   2. Methods: 
      1. constructor to hold the *init_frame*. 
      2. *initialize* main method calling methods below to compute initial pose, triangulate initial map points, and create initial map. 
      3. *GetFundamentalNormalized8Point* to compute the fundamental matrix and obtain the *inlier_mask* of RANSAC. 
      4. *GetStructureAndMotion* to compute the relative pose and triangulate the 3D scene points. 
      5. *CreateInitialMap* to link keyframes, map points, and map. The global BA and linking of other stuff between initializer and tracker are also performed in this function.
9. Since the initialization may fail, care has to be taken when resetting the initialzer (i.e. delete the old initializer object and call the destructors of objects it contained, and then redirect the initializer member of tracker to nullptr, and finally create a new initializer properly).

## Algorithms 
1. *GetFundamentalNormalized8Point*: 
   1. [in] px1, px2, homogeneous 2D image points. 
   2. [out] 