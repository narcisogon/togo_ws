import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("run_20260812_142514/combined.pcd")
points = np.asarray(pcd.points)
x = points[:, 0]
x_norm = (x - x.min()) / (x.max() - x.min())
colors = np.zeros((len(x_norm), 3))
colors[:, 0] = x_norm
colors[:, 2] = 1 - x_norm
pcd.colors = o3d.utility.Vector3dVector(colors)

vis = o3d.visualization.Visualizer()
vis.create_window(window_name="PointsData Viewer", width=1280, height=720)
vis.add_geometry(pcd)

# Set black background
opt = vis.get_render_option()
opt.background_color = np.array([0, 0, 0])
opt.point_size = 1.0

# Spin loop
ctr = vis.get_view_control()
while True:
    ctr.rotate(5.0, 0.0)  # rotate 5 degrees per frame
    vis.poll_events()
    vis.update_renderer()
    if not vis.poll_events():
        break

vis.destroy_window()
