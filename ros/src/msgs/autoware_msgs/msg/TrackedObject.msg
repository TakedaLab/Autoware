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

# estimated velocity of the object
geometry_msgs/Twist velocity

# detection score
float32 score

# whether the object is on the same lane or not
# false: other lane, true: same lane
bool on_same_lane

# cut in and cut out
# 0: neither of them, 1: cut in, 2: cut out
int8 cut_in_and_out
int8 CUT_IN=1
int8 CUT_OUT=2

# status of the object
string object_status
