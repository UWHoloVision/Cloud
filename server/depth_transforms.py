import numpy as np

"""
Backward:
camera_coordinate_system * INV[frame_to_origin] => world_coordinate_system
camera_view_space * INV[camera_extrinsics] => camera_coordinate_system
camera_projection_space * INV[camera_intrinsics] => camera_view_space
"""

# get u, v unprojection mappings to multiply each pixel in the depth map
def get_cam_space_projection(projection_bin, depth_h, depth_w):
    # read binary file
    projection = np.fromfile(projection_bin, dtype = np.float32)
    x_list = [projection[i] for i in range(0,len(projection),2)]
    y_list = [projection[i] for i in range(1,len(projection),2)]
    # rearrange as array
    u = np.asarray(x_list).reshape(depth_w, depth_h).T
    v = np.asarray(y_list).reshape(depth_w, depth_h).T

    return [u, v]


def get_camera_view_to_world_coordinate_system(frame_to_origin, camera_extrinsics):
    """
    camera_coordinate_system * INV[frame_to_origin] => world_coordinate_system
    camera_view_space * INV[camera_extrinsics] => camera_coordinate_system

    Therefore:
    camera_view_space * (INV[camera_extrinsics] * INV[frame_to_origin]) => world_coordinate_system
    """
    return np.linalg.inv(
        np.dot(
            camera_extrinsics.T, 
            np.linalg.inv(frame_to_origin.T)
        )
    )


def get_pointcloud_camera_view(depth_values, u_proj, v_proj):
    """
    Use unprojection mapping in place of camera_intrinsics to get points_to_camera_view.
    Returns a point cloud (3xn matrix) in camera_view
    """
    output_frame = np.zeros((450, 448))
    unscaled_frame = np.zeros((450, 448))
    for i in range(448):
        for j in range(450):
            if depth_values[j][i] > 64000:
                continue
            output_frame[j][i] = depth_values[j][i]
            # Z = Distance / sqrt(u^2 + v^2 + 1)
            # for coord sys w/ camera -> -z, multiply by -1
            unscaled_frame[j][i] = -1.0 * output_frame[j][i] / np.sqrt(u_proj[j][i]**2 + v_proj[j][i]**2 + 1)

    eff_frame = unscaled_frame/1000
    # now we get [Z*u, Z*v, Z], our 3d points
    x_pts = u_proj * eff_frame
    y_pts = v_proj * eff_frame
    z_pts = eff_frame
    return np.stack((x_pts.reshape(-1), y_pts.reshape(-1), z_pts.reshape(-1)), axis=1)


def map_points_to_world(camera_view_pts, camera_view_to_world):
    """
    Given camera_view_to_world and a mapping of points in camera_view, we can get points in world coordinates
    """
    camera_view_pts_ones = np.ones((camera_view_pts.shape[0], camera_view_pts.shape[1]+1))
    camera_view_pts_ones[:,:-1]= camera_view_pts
    return np.dot(
        camera_view_to_world,
        camera_view_pts_ones.T
    ).T

