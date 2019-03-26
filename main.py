import asyncio
import numpy as np
import pptk
import os

from server.frame_message import parse_body
from server.tcp_client import run_client
from server.color_transforms import *
from server.depth_transforms import *

np.set_printoptions(threshold=np.inf)


def get_point_cloud_world_coordinates(frame_id):
    # depth
    bin_file = open('./short_throw_depth_camera_space_projection.bin', 'rb')
    h = 450
    w = 448
    # get u, v unproject mappings
    u_proj, v_proj = get_cam_space_projection(bin_file, h, w)
    msg_file = open('./out/{}.bin'.format(frame_id), 'rb')
    body_lines = msg_file.read()
    body = parse_body(body_lines)
    camera_view_pts = get_pointcloud_camera_view(body['bitmap'], u_proj, v_proj) # for 3d
    camera_view_to_world = get_camera_view_to_world_coordinate_system(
        body['frame_to_origin'],
        body['extrinsics']
    )
    world_coordinates = map_points_to_world(camera_view_pts, camera_view_to_world)
    world_coordinates = world_coordinates[:,:-1]
    return world_coordinates


def get_colors(frame_id, world_coordinates):
    # color
    msg_file = open('./out/{}.bin'.format(frame_id), 'rb')
    body_lines = msg_file.read()
    body = parse_body(body_lines)

    world_to_camera_view = get_world_coordinate_system_to_camera_view(
        body['frame_to_origin'],
        body['extrinsics']
    )
    colors = extract_colors(
        world_coordinates,
        world_to_camera_view,
        body['intrinsics'],
        np.flipud(np.fliplr(body['bitmap']))
    )
    return colors
    
def render_point_cloud(frame_id):
    """Render just the points in pptk"""
    point_cloud_world_coordinates = get_point_cloud_world_coordinates(frame_id)
    # pptk
    v = pptk.viewer(point_cloud_world_coordinates)
    v.set(point_size=0.0001)
    v.color_map('cool', scale=[0, 5])


def get_render_points(d_id, pv_id):
    """get all (potentially colored) points to render"""
    point_cloud_world_coordinates = get_point_cloud_world_coordinates(d_id)
    np_colors = np.array(get_colors(pv_id, point_cloud_world_coordinates))

    # filter
    assert(len(point_cloud_world_coordinates) == len(np_colors))

    point_cloud_world_coordinates_filtered = []
    np_colors_filtered = []
    for i in range(len(point_cloud_world_coordinates)):
        if ~np.isnan(point_cloud_world_coordinates[i]).any():
            point_cloud_world_coordinates_filtered.append(point_cloud_world_coordinates[i])
            np_colors_filtered.append(np_colors[i])
        
    point_cloud_world_coordinates_filtered = np.array(point_cloud_world_coordinates_filtered)
    np_colors_filtered = np.array(np_colors_filtered)

    return point_cloud_world_coordinates_filtered, np_colors_filtered


def load_all_msg():
    """Utility for grabbing all messages in out"""
    files = os.listdir('./out')
    cs = sorted([x.replace('.ppm', '') for x in files if 'ppm' in x])
    ds = sorted([x.replace('.pgm', '') for x in files if 'pgm' in x])
    return zip(ds, cs)


def dbg():
    """Utility for rendering all messages in out as colored point clouds"""
    ds = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
    cs = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)

    # superimpose all from the out directory
    for (d, c) in load_all_msg():
        d0, c0 = get_render_points(d, c)
        ds = np.concatenate((ds, d0))
        cs = np.concatenate((cs, c0))

    v = pptk.viewer(ds)
    v.set(point_size=0.0001, phi=0, r=1, theta=0)
    v.set(lookat=np.array([0.0, 0.0, 0.0], dtype=np.float32))
    np_colors_filtered = cs.astype(float)
    np_colors_filtered /= 255
    np_colors_filtered = np.c_[np_colors_filtered, np.ones(np_colors_filtered.shape[0])]
    v.attributes(np_colors_filtered)

if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run_client(loop))
    loop.close()
