import asyncio
import numpy as np
import pptk
import time
import os

np.set_printoptions(threshold=np.inf)

# utils for saving pgm/ppm files
def write_pgm(filepath, bitmap, w, h):
    with open(filepath, 'wb') as f:
        f.write("P5\n{}\t{}\n65535\n".format(w, h).encode('ascii'))
        f.write(bitmap.tobytes())

def write_ppm(filepath, bitmap, w, h):
    with open(filepath, 'wb') as f:
        f.write("P6\n{}\t{}\n255\n".format(w, h).encode('ascii'))
        f.write(bitmap.tobytes())

# read from message body, and save backups
def parse_body(body):
    ptr = 0

    end = ptr + 1*np.dtype(np.int64).itemsize
    frame_id, = np.frombuffer(body[ptr:end], dtype=np.int64)
    ptr = end
    assert(ptr == 8)

    # backup
    with open('./out/{}.bin'.format(frame_id), 'wb') as f:
        f.write(body)

    end = ptr + (4*np.dtype(np.int32).itemsize)
    width, height, bytes_per_point, points_per_pixel = np.frombuffer(
        body[ptr:end],
        dtype=np.int32,
    )
    ptr = end
    assert(ptr == 24)

    end = ptr + 16*np.dtype(np.float32).itemsize
    frame_to_origin = np.reshape(
        np.frombuffer(
            body[ptr:end], dtype=np.float32,
        ),
        (4, 4),
    )
    ptr = end

    end = ptr + 16*np.dtype(np.float32).itemsize
    intrinsics = np.reshape(
        np.frombuffer(
            body[ptr:end], dtype=np.float32,
        ),
        (4, 4),
    )
    ptr = end

    end = ptr + 16*np.dtype(np.float32).itemsize
    extrinsics = np.reshape(
        np.frombuffer(
            body[ptr:end], dtype=np.float32,
        ),
        (4, 4),
    )
    ptr = end

    np.savetxt(
        "./out/{}_frame_to_origin.csv".format(frame_id), 
        frame_to_origin, 
        delimiter=",",
    )
    np.savetxt(
        "./out/{}_intrinsics.csv".format(frame_id),
        intrinsics,
        delimiter=",",
    )
    np.savetxt(
        "./out/{}_extrinsics.csv".format(frame_id),
        extrinsics,
        delimiter=",",
    )

    if (bytes_per_point == 2 and points_per_pixel == 1):
        # 2-byte (16-bit) depth values
        print("save depth")
        data = np.frombuffer(body[ptr:], dtype=np.uint16)

        bitmap = np.reshape(data, (height, width))
        write_pgm('./out/{}.pgm'.format(frame_id), bitmap, width, height)
    elif (bytes_per_point == 1 and points_per_pixel == 3):
        # 3-byte RGB values
        print("save rgb")
        data = np.frombuffer(body[ptr:], dtype=np.uint8)

        bitmap = np.reshape(data, (height, width, points_per_pixel))
        write_ppm('./out/{}.ppm'.format(frame_id), bitmap, width, height)
    else:
        print("Invalid msg bitmap")

    return {
        'frame_id': frame_id,
        'width': width,
        'height': height,
        'bytes_per_point': bytes_per_point,
        'points_per_pixel': points_per_pixel,
        'frame_to_origin': frame_to_origin,
        'intrinsics': intrinsics,
        'extrinsics': extrinsics,
        'bitmap': bitmap,
    }

# poll for messages
async def tcp_echo_client(loop):
    reader, writer = await asyncio.open_connection('192.168.0.102', 9090, loop=loop)
    
    for i in range(30):
        time.sleep(0.3)
        # first message is depth
        _header_len = await reader.readexactly(4)
        header_len = np.frombuffer(_header_len, dtype=np.int32)[0]
        _body_len = await reader.readexactly(4)
        body_len = np.frombuffer(_body_len, dtype=np.int32)[0]

        print('HEADER: {}, {}'.format(header_len, body_len))
        body = await reader.readexactly(body_len)
        print('BODY: {}'.format(len(body)))
        
        tup = parse_body(body)
        # second message is color
        _header_len = await reader.readexactly(4)
        header_len = np.frombuffer(_header_len, dtype=np.int32)[0]
        _body_len = await reader.readexactly(4)
        body_len = np.frombuffer(_body_len, dtype=np.int32)[0]

        print('HEADER: {}, {}'.format(header_len, body_len))
        body = await reader.readexactly(body_len)
        print('BODY: {}'.format(len(body)))
        
        tup = parse_body(body)
        time.sleep(0.3)
        
    print('Close the socket')
    writer.close()


""" HERE BE DRAGONS """

"""
Forward:
world_coordinate_system * [frame_to_origin] => camera_coordinate_system
camera_coordinate_system * [camera_extrinsics] => camera_view_space
camera_view_space * [camera_intrinsics] => camera_projection_space

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
    loop.run_until_complete(tcp_echo_client(loop))
    loop.close()
