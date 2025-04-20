# Constants for action field
uint8 REMOVE_ACTION = 0
uint8 ADD_ACTION    = 1
uint8 UPDATE_ACTION = 2

# Constants for landmark_type field
uint8 NONE = 0
uint8 BUOY   = 1
uint8 BOAT   = 2
uint8 WALL  = 69

 # Constants for classification field
uint8 UNKNOWN = 0
uint8 RED_BUOY   = 1
uint8 GREEN_BUOY = 2
uint8 NORTH_MARK = 3
uint8 SOUTH_MARK = 4
uint8 EAST_MARK  = 5
uint8 WEST_MARK  = 6
uint8 MOVING_BOAT = 7
uint8 STATIC_BOAT = 8


uint8 landmark_type 0      # For specifying the type of the landmark (boat, buoy, etc)
uint32 id                 # Specific item id within type (type + id should uniquely define the landmark)
uint8 action 1            # Action to be taken on the landmark (remove, add, update)
uint8 classification 0     # Classification of the landmark (e.g. red, green, moving, static, etc)
nav_msgs/Odometry odom     # Position and orientation of the object, also includes reference frame and timestamp
shape_msgs/SolidPrimitive shape # Shape of the object
