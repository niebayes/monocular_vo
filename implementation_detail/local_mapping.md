# Local Mapping
1. Focus: process keyframes passed from tracking and perform local BA to achieve an optimal reconstruction in the surroundings of the current keyframe (by utilizing the covisibility graph).
2. Others: 

## Logic
1. 

## Data members 
1. Handles: 
   1. *map*. 
   2. *tracker*. 
2. Tracking related: 
   1. *idle* denoting whether the local mapper is idle or not.
3. Local mapping related: 
   1. *keyframe_list* list of keyframes to be processed. These keyframes are passed from tracking thread. 
   2. *current_keyframe* the keyframe being processed. 
4. Communication related: 
   1. *accept_new_keyframe* boolean value denoting whether or not **local_mapper** could take in new keyframe. 
   2. 

## Methods 
1. Keyframe related: 
   1. *AcceptNewKeyframe* called by tracker to inform the local mapper is idle or not.
   2. *InsertKeyframe* called by tracker to insert a new keyframe to the list of keyframes to be processed. 
   3. *CheckNewKeyframes* check if the list of keyframes to be processed is empty. 
   4. *ProcessNewKeyframes* process the list of keyframes waiting to be processed. 
   5. 