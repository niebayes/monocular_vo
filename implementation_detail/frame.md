# Frame
1. Focus: wrap the original image object and store the corresponding characteristics.

## Logic 
1. Frame wraps the original image object along with the objects related to it: e.g. **camera**, **features**. Hence, there's no need to additionally implement the two classes. 
2. Viewed as an image, these characteristics should be contained: 
   1. *image_id* to identify the image. 
   2. *image_bounds*.
   3. *keypoints*. 
   4. *descriptors*. Through 1-4, the original image object can be described in an abstract and high-level way.
   5. *map_points* all map points obversed by the frame.
   6. *matches_kpt_mpt* to associate keypoints and map points. This should reveal whether a keypoint has associated with a map point or not.
   7. *bow_vector* for image indexing. 
   8. *feature_vector* for fast matching. 
   9. *infos_of_scales* used when requesting scale infos.
3. Viewed as a camera, these characteristics should be contained: 
   1. *camera_pose* along with the corresponding *rotation*, *translation* and *camera_center* for convenience.
   2. *camera_instrinsics*: *distortion_coefficients*, *calibration_matrix* and *fx*, *fy*, *cx*, *cy* for convenience. These values shall be read from setting file and won't be changed throughout the program.
4. Several methods should be implemented to accompany the data members mentioned above: 
  1. Methods related to image:
      1. *ComputeImageBounds* to compute the image bounds. This method should be called only once.
      2. *ExtractORB* to extract ORB features and describe the salient keypoints. This function should be called only once during construction of each frame. Optionally, *UndistortKeypoints* should be called inside this function.
      3. *IsInFOV* to check whether a given map point is in field of view of "this" frame or not.
      4. *ComputeBoW* to compute the corresponding BoW vector and feature vector.
      5. *GetKeypoints* to obtain keypoints in given spatial space and scale space.
  2. Methods related to camera: 
     1. *SetPose* to set pose and update the *rotation*, *translation*, *camera_center*. Note there's no need to make this thread safe because **frame** class is only a wrapper which is used to construct **keyframe**.
5. Constructor: the **frame** constructor should take as inputs original image object, vocabulary and given camera intrinsics. During the construction, the data members should be set accordingly.