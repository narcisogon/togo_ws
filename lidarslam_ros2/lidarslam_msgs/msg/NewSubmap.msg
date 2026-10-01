std_msgs/Header header            # stamp = submap stamp, frame_id = "map"
uint32 submap_index
geometry_msgs/Pose pose           # pre-optimization pose in map frame
sensor_msgs/PointCloud2 cloud     # submap cloud in the submap local frame
