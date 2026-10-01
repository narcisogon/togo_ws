std_msgs/Header header          # stamp = source submap stamp; frame_id = "map"
uint32 submap_index
geometry_msgs/Pose pose         # submap pose at patch-build time; composer re-projects with live poses
float32 resolution              # cell size [m]
uint32 width
uint32 height
float32 origin_x                # patch-local coords of cell (0,0), in submap-local frame
float32 origin_y
int8[] cost                     # row-major, -1 = unknown, 0..100 = cost (matches OccupancyGrid convention)
uint8[] safe_evidence           # one bounded safe vote per observed cell/submap
uint8[] unsafe_evidence         # severity-weighted hazard votes; zero means no hazard evidence
