# One obstacle tracked by clustering_node from the LiDAR scans (one track).
# All positions in the frame of the LidarObstacleArray header (world frame).

int32 id                          # track ID, stays the same while the obstacle is tracked
uint8 STATIC = 0
uint8 LINEAR = 1
uint8 motion                      # STATIC or LINEAR
builtin_interfaces/Time last_seen # stamp of the last measurement: center and planes are from then
float64 lifetime                  # last_seen - stamp of the first measurement: how long it has been tracked [s]
int32 num_matches                 # number of scans it was matched in
geometry_msgs/Point center        # center of the last measured cluster [m]
geometry_msgs/Vector3 velocity    # velocity estimate [m/s], zero if STATIC
# Shape at last_seen. coef = [n_x, n_y, n_z, offset]: unit normal n out of the obstacle;
# the free side is n_x*x + n_y*y + n_z*z + offset >= 0 (the obstacle is the intersection of the other sides)
shape_msgs/Plane[] planes
