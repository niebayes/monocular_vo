### Class correlations
1. Features are detected in frame and no where else, thus frame owns features
exclusively: frame -> unique_ptr -> feature.
2. Map points are observed by frame through features, but they don't have to
own each other, knowing the existence suffices. Therefore, feature ->
weak_ptr -> map point, and map point -> weak_ptr -> feature.
3. Frames may be owned by tracker, initializer and map and also keyframe
database simultaneously, even though it can be declared of type
unique_ptr in some classes which may involve many move / release
operations which are tedious and cumbersome. Hence it's designed as of
type shared_ptr for the sake of simplicity.
4. For map points, it must be someone owning them which is the map we choosed
as intuition. 
5. Based on this design, with the help of smart pointers (especially the unowning weak_ptr), the removal of keyframes, features and map points is very simple. To delete a keyframe, simply remove it from map; to delete a feature, simply erase it from the frame in which it's detected; to delete a map point, simply remove it from map.