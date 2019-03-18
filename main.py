import asyncio
import numpy as np
import pptk

byte_sz = np.dtype(np.byte).itemsize
int_sz = np.dtype(np.int32).itemsize
long_sz = np.dtype(np.int64).itemsize
float_sz = np.dtype(np.float32).itemsize
matrix_sz = 16*float_sz

np.set_printoptions(threshold=np.inf)

def write_pgm(filepath, bitmap, w, h):
    with open(filepath, 'wb') as f:
        f.write("P5\n{}\t{}\n65535\n".format(w, h).encode('ascii'))
        f.write(bitmap.tobytes())

def write_ppm(filepath, bitmap, w, h):
    with open(filepath, 'wb') as f:
        f.write("P6\n{}\t{}\n255\n".format(w, h).encode('ascii'))
        f.write(bitmap.tobytes())

# read from message body, and save backups
def parse_body(body, body_len):
    ptr = 0

    end = ptr + 1*long_sz
    frame_id, = np.frombuffer(body[ptr:end], dtype=np.int64)
    ptr = end
    assert(ptr == 8)

    # backup
    with open('./out/{}.bin'.format(frame_id), 'wb') as f:
        f.write(body)

    end = ptr + (4*int_sz)
    width, height, bytes_per_point, points_per_pixel = np.frombuffer(
        body[ptr:end],
        dtype=np.int32,
    )
    ptr = end
    assert(ptr == 24)

    end = ptr + matrix_sz
    frame_to_origin = np.reshape(
        np.frombuffer(
            body[ptr:end], dtype=np.float32,
        ),
        (4, 4),
    )
    ptr = end

    end = ptr + matrix_sz
    intrinsics = np.reshape(
        np.frombuffer(
            body[ptr:end], dtype=np.float32,
        ),
        (4, 4),
    )
    ptr = end

    end = ptr + matrix_sz
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
        print("recv depth")
        data = np.frombuffer(body[ptr:body_len], dtype=np.uint16)

        bitmap = np.reshape(data, (height, width))
        write_pgm('./out/{}.pgm'.format(frame_id), bitmap, width, height)
    elif (bytes_per_point == 1 and points_per_pixel == 3):
        # 3-byte RGB values
        print("recv rgb")
        data = np.frombuffer(body[ptr:body_len], dtype=np.uint8)

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
    
    for i in range(1):
        writer.write(b'\x01') # request a message
        # first message is depth
        _header_len = await reader.readexactly(4)
        header_len = np.frombuffer(_header_len, dtype=np.int32)[0]
        _body_len = await reader.readexactly(4)
        body_len = np.frombuffer(_body_len, dtype=np.int32)[0]

        print('HEADER: {}, {}'.format(header_len, body_len))
        body = await reader.readexactly(body_len)
        print('BODY: {}'.format(len(body)))
        
        tup = parse_body(body, len(body))
        # second message is color
        _header_len = await reader.readexactly(4)
        header_len = np.frombuffer(_header_len, dtype=np.int32)[0]
        _body_len = await reader.readexactly(4)
        body_len = np.frombuffer(_body_len, dtype=np.int32)[0]

        print('HEADER: {}, {}'.format(header_len, body_len))
        body = await reader.readexactly(body_len)
        print('BODY: {}'.format(len(body)))
        
        tup = parse_body(body, len(body))
        
    print('Close the socket')
    writer.close()

# here be dragons
def get_cam_space_projection(projection_bin, depth_h, depth_w):
    # read binary file
    projection = np.fromfile(projection_bin, dtype = np.float32)
    x_list = [projection[i] for i in range(0,len(projection),2)]
    y_list = [projection[i] for i in range(1,len(projection),2)]
    
    # rearrange as array
    u = np.asarray(x_list).reshape(depth_w,depth_h).T
    v = np.asarray(y_list).reshape(depth_w,depth_h).T

    return [u, v]

# camera_coordinate_system * camera_extrinsics = frame_to_origin
def get_cam_to_world_mtx(frame_to_origin, camera_extrinsics):
    # camera_projection -> camera_view -> world state
    return np.linalg.inv(
        np.dot(
            camera_extrinsics.T, 
            np.linalg.inv(frame_to_origin.T)
        )
    )


def get_camera_view_pts(depth_values, u_proj, v_proj):
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

def map_points_to_world(camera_view_pts, world_matrix):
    camera_view_pts_ones = np.ones((camera_view_pts.shape[0], camera_view_pts.shape[1]+1))
    camera_view_pts_ones[:,:-1]= camera_view_pts
    return np.dot(world_matrix, camera_view_pts_ones.T).T

def get_world_to_camera_view_pv(frame_to_origin, camera_view): # aka extrinsics
    # camera_projection -> camera_view -> world state
    # world_to_camera_view = np.dot(
    #     camera_view.T, # boo
    #     np.linalg.inv(frame_to_origin.T)
    # )
    # return world_to_camera_view
    return np.dot(
        camera_view.T,
        np.linalg.inv(frame_to_origin.T)
    )

def extract_colors(world_coordinates_3d, world_to_camera_view, camera_projection, pv_values):
    # Map this world data onto the PV camera pixel map
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
        if pi_x < 1280 and pi_y < 720 and pi_x >= 0 and pi_y >= 0:
            pv_color = pv_values[int(pi_y)][int(pi_x)]
            pc_colors.append(pv_color)
        else:
            pc_colors.append((float(0), float(0), float(0)))

    return pc_colors

def get_point_cloud(frame_id):
    # depth
    bin_file = open('./short_throw_depth_camera_space_projection.bin', 'rb')
    h = 450
    w = 448
    u_proj, v_proj = get_cam_space_projection(bin_file, h, w)
    msg_file = open('./out/{}.bin'.format(frame_id), 'rb')
    body_lines = msg_file.read()
    body = parse_body(body_lines, len(body_lines))
    camera_view_pts = get_camera_view_pts(body['bitmap'], u_proj, v_proj) # for 3d
    camera_view_to_world = get_cam_to_world_mtx(
        body['frame_to_origin'],
        body['extrinsics']
    )
    world_coordinates = map_points_to_world(camera_view_pts, camera_view_to_world)

    # needs to be filtered + indexed
    # camera_view_pts = np.array([pt for pt in camera_view_pts if ~np.isnan(pt).any() and np.nonzero(pt)])
    # world_coordinates = np.array([pt for pt in world_coordinates if ~np.isnan(pt).any() and np.nonzero(pt)])
    world_coordinates = world_coordinates[:,:-1]
    return world_coordinates

def get_colors(frame_id, world_coordinates):
    # color
    msg_file = open('./out/{}.bin'.format(frame_id), 'rb')
    body_lines = msg_file.read()
    body = parse_body(body_lines, len(body_lines))

    world_to_camera_view = get_world_to_camera_view_pv(
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
    world_coordinates = get_point_cloud(frame_id)
    # pptk
    v = pptk.viewer(world_coordinates)
    v.set(point_size=0.0001)
    v.color_map('cool', scale=[0, 5])


def render(d_id, pv_id):
    world_coordinates = get_point_cloud(d_id)
    np_colors = np.array(get_colors(pv_id, world_coordinates))

    # filter
    assert(len(world_coordinates) == len(np_colors))

    world_coordinates_filtered = []
    np_colors_filtered = []
    for i in range(len(world_coordinates)):
        if ~np.isnan(world_coordinates[i]).any():
            world_coordinates_filtered.append(world_coordinates[i])
            np_colors_filtered.append(np_colors[i])
        
    world_coordinates_filtered = np.array(world_coordinates_filtered)
    np_colors_filtered = np.array(np_colors_filtered)

    # camera_view_pts = np.array([pt for pt in camera_view_pts if ~np.isnan(pt).any() and np.nonzero(pt)])
    # world_coordinates = np.array([pt for pt in world_coordinates if ~np.isnan(pt).any() and np.nonzero(pt)])
    return world_coordinates_filtered, np_colors_filtered

def dbg():
    (d1, c1) = render(174072985617, 174071403575)
    (d2, c2) = render(174127310562, 174125690921)

    v = pptk.viewer(np.concatenate((d1, d2)))
    v.set(point_size=0.0001)
    v.set(lookat=np.array([0.0, 0.0, 0.0], dtype=np.float32))
    np_colors_filtered = np.concatenate((c1, c2)).astype(float)
    np_colors_filtered /= 255
    np_colors_filtered = np.c_[np_colors_filtered, np.ones(np_colors_filtered.shape[0])]
    v.attributes(np_colors_filtered)

if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(tcp_echo_client(loop))
    loop.close()
