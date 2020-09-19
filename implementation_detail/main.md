# Main 
1. Focus: create slam system and process CLI args.

## Logic: 
1. Init slam system. Hence, a **system** class needs to be implemented. 
2. Load image files and timestamps from CLI args. 
3. Send image (grayscaled) frame by frame to system object (which will soon be delivered to tracker (by calling the entry function of it)). Use timestamp to delay the sending till the correct time reached.
4. Exit when exhausted.