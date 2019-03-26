import numpy as np

"""
Forward:
world_coordinate_system * [frame_to_origin] => camera_coordinate_system
camera_coordinate_system * [camera_extrinsics] => camera_view_space
camera_view_space * [camera_intrinsics] => camera_projection_space
"""

def get_world_coordinate_system_to_camera_view(frame_to_origin, camera_extrinsics):
    """
    world_coordinate_system * [frame_to_origin] => camera_coordinate_system
    camera_coordinate_system * [camera_extrinsics] => camera_view_space

    Therefore:
    world_coordinate_system * ([frame_to_origin] * [camera_extrinsics]) => camera_view_space
    """
    return np.dot(
        camera_extrinsics.T,
        np.linalg.inv(frame_to_origin.T)
    )

def extract_colors(world_coordinates_3d, world_to_camera_view, camera_projection, pv_values):
    # Map this world data onto the PV camera pixel map

    # add row of 1s
    world_coordinates_pts_ones = np.ones((world_coordinates_3d.shape[0], world_coordinates_3d.shape[1]+1)) # ???
    world_coordinates_pts_ones[:,:-1]= world_coordinates_3d
    camera_view_pv = np.dot(
        world_to_camera_view,
        world_coordinates_pts_ones.T
    ).T # 4 x N

    camera_view_pv[:,-1] = np.ones(camera_view_pv.shape[0])
    camera_projection_pv = np.dot(
        camera_projection,
        camera_view_pv.T
    ).T #N x 4

    # Normalize by the third column (Homogenize) and proceed
    pixel_coordinates_2d_pv = (camera_projection_pv / camera_projection_pv[:,-2][:,None])[:,:-2]
    pixel_coordinates_2d_pv *= 0.5
    pixel_coordinates_2d_pv += np.array([0.5, 0.5])
    pixel_x, pixel_y = pixel_coordinates_2d_pv[:,0], pixel_coordinates_2d_pv[:,1]
    pixel_x *= 1280 # Image Width
    pixel_y = (1 - pixel_y)*720
    pixel_x, pixel_y = pixel_x.astype(int), pixel_y.astype(int)
    pixels = np.stack((pixel_x,pixel_y), axis=1)
    pc_colors = []
    for pi_x, pi_y in pixels:
        # filter only points belonging in image
        if pi_x < 1280 and pi_y < 720 and pi_x >= 0 and pi_y >= 0:
            pv_color = pv_values[int(pi_y)][int(pi_x)]
            pc_colors.append(pv_color)
        else:
            pc_colors.append((float(0), float(0), float(0)))

    return pc_colors
