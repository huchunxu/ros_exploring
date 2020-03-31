# Contains the location of a stored point cloud scan of an object, 
# as well as additional metadata about that scan 

# the database id of the model
int32 model_id

# the location of the bag file storing the scan
string bagfile_location

# the source of the scan (e.g. simulation)
string scan_source

# the ground truth pose of the object that was scanned
geometry_msgs/PoseStamped pose

# the topic that the points in the bag are published on
string cloud_topic