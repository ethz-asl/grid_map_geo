# Geomap Message
#

std_msgs/Header header


# ESPG of the coordinate the map is in
uint16 geo_coordinate

# Size of the map in pixels
uint16 width
uint16 height


# Resolution of the map
float64 x_resolution
float64 y_resolution

# x value in the specified geocoordinate system
float64 origin_x

# y value in the specified geocoordinate system
float64 origin_y

# Altitude in the specified vertical datum
float64 origin_altitude
