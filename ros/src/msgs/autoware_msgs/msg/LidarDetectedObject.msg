std_msgs/Header header

# object id
uint32 id

# object type
string label

# version of object detector
string version

# pose
geometry_msgs/Pose pose

# size of bounding box
geometry_msgs/Vector3 size

# variance of position (range of error)
geometry_msgs/Vector3 variance

# feature of the object
float32[] feature

# points of the object
sensor_msgs/PointCloud2 cloud

# detection score
float32 score
