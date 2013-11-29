# Filename of the picture on the hard drive (can be used for chronological sorting)
string filename

# Time that the picture was taken. Format is ROS time and/or a string coming directly from the camera.
time dateTime
string dateTimeFormatted

# Which type of actor took this picture? An OCU or something external to the system?
uint8 sourceType

# Which actor took this picture? A UGV, in-field rescuer Bob, rescuer Alice, etc.
string source

# User-supplied text describing the picture
string annotation

# This represents the position with GPS coordinates
GeoCoordinate latitude
GeoCoordinate longitude

# Represents the position and orientation of the picture with respect to the /map frame
geometry_msgs/Pose pose

# Direction of the picture (with respect to North), as given by the camera or as computed from the /map frame
float64 direction

# All values in radians
float64 fieldOfViewHorizontal
float64 fieldOfViewVertical

# This contains the image, bit for bit
uint8[] completeFile


################################

uint8 SOURCE_TYPE_OCU 		= 1
uint8 SOURCE_TYPE_EXTERNAL 	= 2

