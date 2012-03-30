uint32 keyframe_num

# Rotation and Translation
float32[] rotation
float32[] translation

# Image
uint32 height
uint32 width
uint8[] im

# Depth Points
uint32 numberOf3DPoints
uint32 point_step #used for getting point in single row based off x and y coords
uint8[] points

# Shi Tomasi Corners
uint32 numCorn1
float32[] corn1 # relates this keyframe to last
uint8[] status # which ones are good

uint32 numCorn2
float32[] corn2 # corners found in this keyframe to relate to next

# SURF
uint32 numSURF
float32[] features
float32[] descBuffer
int32[] surfMatches # index of matches in previous keyframe, -1 means no match

# IMU
float32 imux
float32 imuy
float32 imuz
