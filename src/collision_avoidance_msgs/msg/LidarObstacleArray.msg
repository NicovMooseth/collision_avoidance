# All obstacles tracked by clustering_node, published once per scan.
# Includes tracks not matched in this scan (until their timeout): compare last_seen with header.stamp.

std_msgs/Header header        # stamp = scan time, frame_id = world frame
LidarObstacle[] obstacles
