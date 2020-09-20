# Map
1. Focus: maintain the keyframes and map points inserted to it. Erase the bad ones if requested.

## Data member 
1. *max_keyframe_id* the maximal id of the keyframes inserted to the map so far. 
2. *keyframes* set of keyframes the map currently maintained. 
3. *map_points* set of map points the map currently maintained. 

## Methods 
1. *add_keyframe*, *erase_keyframe*.
2. *add_map_point*. *erase_map_point*.
3. *get_keyframes*, *get_map_points*. 
4. *get_num_keyframes*, *get_num_map_points*. 
5. *Clear* clear the map and all the objects related to it. Care has to taken on actually free the memory of the pointers.