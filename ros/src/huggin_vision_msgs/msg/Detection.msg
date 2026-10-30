# One cube, arrow or zone seen by the camera.

string KIND_CUBE=cube
string KIND_ARROW=arrow
string KIND_ZONE=zone

string COLOR_RED=red
string COLOR_GREEN=green
string COLOR_BLUE=blue

string LENGTH_CLOSE=close    # arrow body length 30 mm
string LENGTH_NORMAL=normal  # arrow body length 60 mm
string LENGTH_FAR=far        # arrow body length 100 mm

string kind
string color

# Center of the object in the frame of DetectionArray.header, which is on the
# floor (z up, x forward). Cube: its center. Zone: center of the square on the
# floor. Arrow: middle of the arrow, on the wall it is stuck to.
geometry_msgs/Pose pose

# Cube: yaw about z (rad), only known modulo 90 degrees.
# Zone: yaw of its sides (rad), modulo 90 degrees.
# Arrow: direction it points in the image (rad), counter-clockwise from the
# image right, so 0 = right, pi/2 = up.
float32 angle

# Arrow only: right, up, left, down, or UNK when in between
string direction

# Arrow only: close, normal or far (see LENGTH_*)
string length_class

# Zone: its corners on the floor (all visible vertices if partial).
# Arrow: [tail, tip] in the same frame as pose.
geometry_msgs/Point[] polygon

float32 confidence  # 0 to 1
bool partial        # zone cut by the image border: size and center are lower bounds
