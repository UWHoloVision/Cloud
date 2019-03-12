import asyncio
import numpy as np
from PIL import Image

byte_sz = np.dtype(np.byte).itemsize
int_sz = np.dtype(np.int32).itemsize
long_sz = np.dtype(np.int64).itemsize
float_sz = np.dtype(np.float32).itemsize
matrix_sz = 16*float_sz

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

    # 16 bit depth img here; RGB is different obv
    data = np.frombuffer(body[ptr:body_len], dtype=np.uint16)

    bitmap = np.reshape(data, (height, width))
    img = Image.frombuffer('L', (width, height), bitmap.tobytes())
    img.save('./out/{}.pgm'.format(frame_id))
    return (
        frame_id,
        width, height, bytes_per_point, points_per_pixel,
        frame_to_origin,
        intrinsics,
        extrinsics,
        bitmap
    )

# poll for messages
async def tcp_echo_client(loop):
    reader, writer = await asyncio.open_connection('192.168.137.38', 9090, loop=loop)
    
    for i in range(10):
        # first message is always the header length (int)
        # header length is never 0
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

if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(tcp_echo_client(loop))
    loop.close()
