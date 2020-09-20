# Map point


## Data members 
1. ID related: 
   1. *id* to identify this map point. 
   2. 
2. *is_bad* flag to denote whether this map point was marked bad or not.
3. *world_pos* the 3D coordinates (position) of the map point in the world frame. 
4. *representative_descriptor* the best descriptor of the (associated keypoints of the) map point. This is found by finding the descriptor among the descriptors of all the keypoints associated with this map point which has the minimum median hamming distance with all the other descriptors.
5. Observation related: 
   1. *observations* a map structure (key: keyframe, value: index) to store the observations of the map point, i.e. this map point is associated with which keypoint (index) in which keyframe. This member could help decide whether a given map point is in a certain keyframe or not. 
   2. *num_observed* 
   3. *num_found* 
   4. *num_visible*
   5. *mean_view_direction* the mean unit vector of all its viewing directions. This member helps decide if this map point needs to be culled or not. 
6. Scale related: 
   1. *min_distance* the minimum distance at which the point can be observed. 
   2. *max_distance* the maximum distance at which the point can be observed. In practice, a factor will be multiplied with each of the distance to get the real scale invirance limits of the ORB feature.
7. *reference_keyframe* by which this map point is observed in the first time. 
8. 

## Methods
1. observation related: 
   1. *IsInKeyframe* check whether this map point was observed by the given keyframe or not. 
   2. *AddObservation* to add an item to the *observations*. 
   3. *EraseObservations* to delete an item of the *observations*.
2. Getter and setter: 
   1. *GetWorldPos*, *SetWorldPos*. 
   2. *GetMeanViewDirection*. 
   3. *GetReferenceKeyframe*. 
   4. *GetObservations*. 
   5. *GetNumObservations*.
   6. *IsBad*, *SetBadFlag*. 
   7. *GetFound*, *IncreaseFound*, *IncreaseVisible*. 
   8. 
3. *ComputeRepresentativeDescriptor* to obtain the *representative_descriptor*.
4. 