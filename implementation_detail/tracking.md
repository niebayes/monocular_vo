# Tracking 
1. Focus: process image stream frame by frame, track the current frame based on the last frame (i.e. estimate the pose T_curr_last (in fact, Tcw is returned with the initial frame fixed as the world frame)). 
2. Others: decide when to insert a new keyframe.

## Logic 
1. If not initialized, do initialization. To initialize, at least two frames are needed. Hence, tracker needs to maintain and incrementally update the *init_frame* and *curr_frame*. Thus a **frame** class needs to be implemented which wraps the original image object and stores corresponding characteristics.
2. To do the initialization, it's better to have an **initializer** to handle all the stuff involved. It takes as input *init_frame* to construct and grab *curr_frame* to form an initialzation system. The outputs are the initial map, relative pose between the first two frames (with the *init_frame* fixed as the world frame), and others.
3. 
