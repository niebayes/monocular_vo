# System 
Focus: called from the main function to ready all threads and load configs. 

## Logic 
1. Load vocubulary and config files. Voc supports fast feature matching and image indexing while config provides user-specified settings for the system. For voc, two members are created: *voc* and *keyframe_db* with the former holds the original voc and the latter keyframe database used for fast **relocalization**.
2. Create *tracker* and *mapper* objects and start their corresponding *threads* (by calling the entry function of each of them). The loaded configs (e.g. voc, kfdb) will be passed into the constructor of tracker and mapper.
3. Create *map* object which manipulates the keyframes and map points. Hence, **keyframe** and **mappoint** classes need to be implemented along with their container **map**.
4. Link the objects previously created (by calling setter of the objects). This establishes the tunnels for the objects.
5. Shutdown when requested.