import math
PI = math.pi

## Set min value and max value of joints 
LEFT_JOINTS_MIN_VALUE = [
    -3.491,        # joint0 -200 degree
    -3.31,        # joint1 -190(3.31) degree (restrict to -80 degree)
    -PI / 2.0,     # joint2
    0.0,           # joint3 
    -PI / 2.0,     # joint4
    -PI / 2.0,     # joint5
    -PI / 2.0      # joint6
]
LEFT_JOINTS_MAX_VALUE = [
    1.396,         # joint0 80 degree
    0,             # joint1
    PI / 2.0,      # joint2
    2.443,         # joint3 140 degree
    PI / 2.0,      # joint4
    PI / 2.0,      # joint5
    PI / 2.0       # joint6
]

RIGHT_JOINTS_MIN_VALUE = [
    -1.396,        # joint0 -80 degree
    0,             # joint1 
    -PI / 2.0,     # joint2
    0.0,           # joint3 
    -PI / 2.0,     # joint4
    -PI / 2.0,     # joint5
    -PI / 2.0      # joint6
]
RIGHT_JOINTS_MAX_VALUE = [
    3.491,         # joint0 200 degree
    3.31,          # joint1 190
    PI / 2.0,      # joint2
    2.443,         # joint3 140 degree
    PI / 2.0,      # joint4
    PI / 2.0,      # joint5
    PI / 2.0       # joint6
]