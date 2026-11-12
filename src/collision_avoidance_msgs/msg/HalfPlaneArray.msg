# All half-plane constraints computed from one scan.
# Published by scan_to_constraint_node, consumed by cbf_node.

std_msgs/Header header        # stamp = scan time, frame_id = frame the planes are expressed in
shape_msgs/Plane[] planes
