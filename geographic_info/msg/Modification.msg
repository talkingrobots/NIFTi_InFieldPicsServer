# Filename of the picture on the hard drive (can be used for chronological sorting)
string filename

# Which actor modified this picture? A UGV, in-field rescuer Bob, rescuer Alice, etc.
string source

# User-supplied text describing the picture
string annotation

# This represents the position with GPS coordinates
#GeoCoordinate latitude
#GeoCoordinate longitude

# Represents the position and orientation of the picture with respect to the /map frame
#geometry_msgs/Pose pose

# Direction of the picture (with respect to North), as given by the camera or as computed from the /map frame
#float64 direction
