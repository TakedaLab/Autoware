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
geometry_msgs/Twist absolute_velocity
geometry_msgs/Twist relative_velocity

# GPS info
float64 lat
float64 lon

# detection score
float32 score

# on which lane the object exists
# 0: not analyzed, 1: oon the same lane, 2: on the different lane, 3: unknown
int8 lane
int8 NOT_ASSIGNED=0
int8 EGO_LANE=1
int8 NEXT_LANE=2
int8 INVALID=3

# the closest vehicle in front on the same lane
# true: closest vehicle, false: not closest vehicle
bool cipv_flag

# cut in and cut out
int8 cut_in_and_out
int8 UNDEFINED=0
int8 IN_HOST_LANE=1
int8 OUT_HOST_LANE=2
int8 CUT_IN=3
int8 CUT_OUT=4

# status of the object
string object_status
