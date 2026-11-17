# All half-plane constraints computed from one scan.
# Published by scan_to_constraint_node, consumed by cbf_node.

std_msgs/Header header        # stamp = scan time, frame_id = frame the planes are expressed in
# coef = [n_x, n_y, n_z, offset]: unit normal n facing the free side (towards the LiDAR, away from the obstacle);
# the free side is n_x*x + n_y*y + n_z*z + offset >= 0
shape_msgs/Plane[] planes
