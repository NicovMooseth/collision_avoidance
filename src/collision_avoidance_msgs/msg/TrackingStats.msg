# Statistics of the LiDAR obstacle tracking of clustering_node, one message per scan.
# To judge how stable the tracks are: in a static scene few new / deleted tracks and long lifetimes are good.
# Unless stated otherwise, the counts and lifetimes are of the CONFIRMED tracks (matched in at least
# history.min_matches scans), i.e. of the published obstacles. Unconfirmed tracks are counted in num_tentative.

std_msgs/Header header             # stamp = scan time

int32 num_clusters                 # clusters found in this scan (input of the association)
int32 num_tracks                   # confirmed tracks alive after this scan (matched now or within the timeout)
int32 num_matched                  # confirmed tracks matched in this scan that were confirmed before
int32 num_new                      # tracks confirmed in this scan (with min_matches 1: created in this scan)
int32 num_deleted                  # confirmed tracks deleted in this scan (not seen for history.track_timeout)
int32 num_tentative                # unconfirmed tracks alive after this scan (not published)
int32 num_tentative_dropped        # unconfirmed tracks deleted in this scan (missed a scan before confirmation)
int32 num_moving                   # tracks alive with motion LINEAR

float64 mean_lifetime              # mean lifetime of the alive tracks [s]
float64 median_lifetime            # median lifetime of the alive tracks [s]
float64 max_lifetime               # longest lifetime of the alive tracks [s]
float64 mean_lifetime_deleted      # mean lifetime of the tracks deleted in this scan, 0 if none [s]
int32 total_tracks_created         # tracks created since the node started (confirmed or not)
int32 total_tracks_confirmed       # tracks confirmed since the node started
