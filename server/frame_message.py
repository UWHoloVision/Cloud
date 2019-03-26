import numpy as np
import os

from .utils import write_pgm, write_ppm

out_folder = os.path.join(os.getcwd(), 'out')

# read from message body, and save backups
def parse_body(body):
    ptr = 0

    end = ptr + 1*np.dtype(np.int64).itemsize
    frame_id, = np.frombuffer(body[ptr:end], dtype=np.int64)
    ptr = end
    assert(ptr == 8)

    # backup
    with open(os.path.join(out_folder, '{}.bin'.format(frame_id)), 'wb') as f:
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
        write_pgm(os.path.join(out_folder, '{}.pgm'.format(frame_id)), bitmap, width, height)
    elif (bytes_per_point == 1 and points_per_pixel == 3):
        # 3-byte RGB values
        print("save rgb")
        data = np.frombuffer(body[ptr:], dtype=np.uint8)

        bitmap = np.reshape(data, (height, width, points_per_pixel))
        write_ppm(os.path.join(out_folder, '{}.ppm'.format(frame_id)), bitmap, width, height)
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
