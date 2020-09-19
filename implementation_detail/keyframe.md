# Keyframe


## Data members 
1. Relocalization related: 
   1. *reloc_query_id* the id of the frame that querys the keyframe database for relocalization. 
   2. *reloc_num_shared_words* the number of common words between the query frame and this keyframe. These two variables are reset for each query operation, and are used to help rank the keyframes sharing words with the query frame. 
   3. 