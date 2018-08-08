# The family of the Tag
AprilTagFamily family

# The Decoded ID of the tag
int8 id

# How many error bits were corrected
int8 hamming

# A measure of the quality of tag localization, average contrast of the pixels
# around the border of the tag
float32 goodness

# A measure of the quality of the binary decoding process
float32 decision_margin

# The Homography Matrix Describing the projection
Matrix H

# The Center of the Detection
Point2D c

# The Corners of the Tag
Point2D[] p