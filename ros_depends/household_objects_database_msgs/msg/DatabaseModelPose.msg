# Informs that a specific model from the Model Database has been 
# identified at a certain location

# the database id of the model
int32 model_id

# if the object was recognized by the ORK pipeline, its type will be in here
# if this is not empty, then the string in here will be converted to a household_objects_database id
# leave this empty if providing an id in the model_id field
object_recognition_msgs/ObjectType type

# the pose that it can be found in
geometry_msgs/PoseStamped pose

# a measure of the confidence level in this detection result
float32 confidence

# the name of the object detector that generated this detection result
string detector_name
