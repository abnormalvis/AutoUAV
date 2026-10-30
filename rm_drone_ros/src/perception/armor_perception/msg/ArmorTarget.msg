# Perception result for a single detected armor plate, expressed in the world
# frame carried by header.frame_id.

std_msgs/Header header

uint8 id                    # index within this frame's array
uint8 armor_name            # auto_aim::ArmorName (0..8), 8 = not_armor
float32 confidence          # digit classifier confidence, 0 when disabled

# Target pose in the world frame. The translation is the plate center, the
# rotation is R_armor2world (x axis = plate outward normal, z axis = plate up).
geometry_msgs/PoseWithCovariance pose

float32 distance            # camera-to-plate range estimate [m]

uint8 LOST=0
uint8 DETECTING=1
uint8 TRACKING=2
uint8 TEMP_LOST=3
uint8 tracking_state
