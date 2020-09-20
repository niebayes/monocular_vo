# Keyframe


## Data members 
1. *is_bad* flag to denote whether this keyframe was marked as bad or not.
2. Relocalization related: 
   1. *reloc_query_id* the id of the frame that querys the keyframe database for relocalization. 
   2. *reloc_num_shared_words* the number of common words between the query frame and this keyframe. These two variables are reset for each query operation, and are used to help rank the keyframes sharing words with the query frame. 
3. Covisibility graph related: 
   1. *co_keyframe_weights* a map structure (key=keyframe, value=weights(i.e. number of shared map points)) 
   2. *co_ordered_keyframes* sorted vector of keyframes wrt. weights. 
   3. *co_ordered_weights* sorted vector of weights in the descending order.

## Methods 
1. *ComputeSceneMedianDepth* compute the mean depth of scene. This method is used when dealing with depth.
2. 