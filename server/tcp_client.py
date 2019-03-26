import asyncio
from .frame_message import parse_body

# poll for messages
async def run_client(loop):
    reader, writer = await asyncio.open_connection('192.168.0.102', 9090, loop=loop)
    
    for i in range(30):
        # first message is depth
        print("Receiving depth message {}".format(i))
        header_len_buf = await reader.readexactly(4)
        header_len = np.frombuffer(header_len_buf, dtype=np.int32)[0]
        body_len_buf = await reader.readexactly(4)
        body_len = np.frombuffer(body_len_buf, dtype=np.int32)[0]
        body = await reader.readexactly(body_len)
        print("Received depth message {}".format(i))
        _ = parse_body(body)
        print("Parsed depth message {}".format(i))
        
        # second message is color
        print("Receiving color message {}".format(i))
        header_len_buf = await reader.readexactly(4)
        header_len = np.frombuffer(header_len_buf, dtype=np.int32)[0]
        body_len = await reader.readexactly(4)
        body_len = np.frombuffer(body_len_buf, dtype=np.int32)[0]
        body = await reader.readexactly(body_len)
        print("Received color message {}".format(i))
        _ = parse_body(body)
        print("Parsed color message {}".format(i))
        time.sleep(0.3)
        
    print('Closed the socket')
    writer.close()
