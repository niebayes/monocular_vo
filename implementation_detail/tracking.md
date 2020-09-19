# Tracking 
1. Focus: process image stream frame by frame, track the current frame based on the last frame (i.e. estimate the pose T_curr_last (in fact, Tcw is returned with the initial frame fixed as the world frame)). 
2. Others: decide when to insert a new keyframe.

## Logic 
1. If not initialized (i.e. the tracking state is *NOT_INITIALIZED*), do initialization. To initialize, at least two frames are needed. Hence, tracker needs to maintain and incrementally update the *last_frame* and *curr_frame*. Thus a **frame** class needs to be implemented which wraps the original image object and stores corresponding characteristics.
2. To do the initialization, it's better to have an **initializer** to handle all the stuff involved because the initialization shall be independent to the main tracking process. It takes as input *last_frame* to construct and grab *curr_frame* to form an initialzation system. So the Mono-SLAM system needs at least two consecutive "good" frames to bootstrap the tracking. For good, we mean that the matched keypoints between the two frames shall exceed some preset threshold, and the quality of the map shall not be too bad (i.e. number inlier matches survived from the optimization should be higher than another preset threshold). The outputs are the initial map, relative pose between the first two frames (with the *last_frame* fixed as the world frame), and other stuff.
3. If the initalization is okay, the tracking state is turned to *GOOD* and the main tracking process is invoked. Otherwise, the **current** initializer is deleted whilst a new initialzer will be created when it comes to the next frame. 
4. There're generally two different ways to invoke the main tracking process:
   1. If the current frame is very after the initialization or relocalization, *TrackFromReferenceKeyframe* is applied to handle the unstability of the tracking system due to initialization and relocalization. 
   2. If the tracking system is stable (i.e. the time interval from the last initialization or relocalization is large enough), *TrackWithConstantMotionModel* is applied. Compared with above, this tracking scheme is a bit of light-weight as it does not involves the keyframe and BoW.  
5. If the tracking is lost, *Relocalization* is applied trying to track the current frame from one of candidate keyframes in the *keyframe_database* accumulated thus far. 
6. If the tracking is not recoverable, reset the tracking system as well as other stuff (i.e. reset the whole SLAM system).  
7. If the tracking is okay anyway, *TrackLocalMap* is applied. Since there're inceasingly more map points created by the **local_mapper**, if the data association between these unmatched map points and the keypoints in the current frame could be established, the tracking would be more stable. And with more 3D-2D correspondences, the pose graph optimization would obtain more accurate pose. To track the local map, two data members should be implemented: *local_keyframes* and *local_map_points*. Some methods related to these members should also be implemented. 
8. If the tracking is still okay, a *const_velocity_model* is computed to used in *TrackWithConstantMotionModel*. Otherwise, the velocity model is empty to indicate that *TrackWithConstantMotionModel* is not applicable. 
9. In the end of each tracking, it's time to decide whether or not insert a new keyframe. Hence, some methods related to this should be implemented. The new created keyframe will be passed into the **local_mapper** and this is one of the few relation between **tracker** and **local_mapper** since we want the tracking process as independent as possible.  
10. Done all these stuff, we should make the tracking ready for the next frame. E.g. we should update the *last_frame* and redirect the outlier map points (the correspondences between the these map points and certain set of keypoints are rejected during the optimization) to the nullptr. Since we don't want the next frame to track these map points (without which the tracking will be more robust and this is the core of the tracking process).
11. (Optionally) update the trajectory infos. Hence two data members *reference_keyframes* and *relative_frame_poses* should be implemented.

## Data members 
1. Handles: 
   1. *system* to communicate with the system, e.g. to reset the system.
   2. *map* to communicate with map, e.g. add new keyframes and new map points. 
   3. *local_mapper* to communicate with the local mapper, e.g. add new keyframes and map points the list of keyframes and list of map points respectively.
   4. *initializer* to initialize. 
   5. *keyframe_database* for fast relocalization.
3. Tracking related: 
   1. *state* state of the tracking system. 
      1. *NOT_INITIALIZED* not initialized, i.e. the very first of tracking or very after the resetting. 
      2. *GOOD*. 
      3. *LOST*.
   2. *last_frame*. 
   3. *init_frame*. 
   4. *last_keyframe_id* to identify the time interval from the last insertion of keyframe. This will be used to help decide whether or not to insert a new keyframe. 
   5. *last_relocalization_id* to identify the time interval from the last relocalization. This will be used when requesting relocalization and help system to identify the state of the tracking (e.g. if the relocalization is too often, the tracking may be unstable, and the matching should be more wider). 
   6. *const_velocity_model* the model (i.e. the transformation from last frame to current frame) used to track current frame from last frame based on the constant velocity assumption. 
   7. *num_inlier_matches* number of inlier matches survived in the optimization (during optimization, some correspondences (matches) will be marked as outliers according to the computed error). This variable will be updated in *TrackLocalMap* since *TrackLocalMap* is the last tracking method in the tracking process. And it will be used in *NeedNewKeyframe* to help decide whether or not to insert a new keyframe.
4. Frame related: 
   1. *frame_id* to identify the current frame or count the number of frames processed thus far (plus one to get the number).
   2. *calibration_matrix*. 
   3. *distortion_coefficients*.  
   4. *scale_infos*. The above four members along with the original image object will be used when constructing a new frame object.
5. Local map related: 
   1. *reference_keyframe* the keyframe sharing most map points with the current frame. It's used for tracking from reference keyframe, and local mapping optimization.
   2. *local_keyframes*. 
   3. *local_map_points*. This will be used to track local map.
6. trajectory related: 
   1. *reference_keyframes* to store for each frame the reference keyframe.
   2. *relative_frame_poses* to store for each frame the transformation from the reference keyframe. 