# A single tracked hand, ready to be consumed by the robot controller.
std_msgs/Header header

string hand_id                      # "Left" / "Right" / "Hand_0" ...

geometry_msgs/Point   centroid      # center of mass of the hand zone, meters, camera frame
geometry_msgs/Vector3 direction     # unit pointing vector (wrist -> fingertips, "fingers forward")
geometry_msgs/Vector3 palm_normal   # unit palm normal (orientation hint)

float32 angle_deg                   # yaw of the direction vector in the image plane
float32 area_m2                     # area occupied by the hand zone
float32 volume_m3                   # approximate occupied volume
float32 distance_z                  # depth (centroid.z), meters
float32 ttc                         # time-to-collision, seconds (-1 if none)
float32 confidence                  # detection confidence [0..1]
string  gesture                     # "Open Hand" / "Fist (Stop)" / "Pinch Trigger"
