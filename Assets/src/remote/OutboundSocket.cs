using UnityEngine;
using System;
using System.Threading.Tasks;
using System.Threading;

#if ENABLE_WINMD_SUPPORT
using System.Threading.Tasks.Dataflow;
using Windows.Networking.Sockets;
using Windows.Storage.Streams;
#endif // ENABLE_WINMD_SUPPORT

/*
 * After server connects to the Hololens, write to server through a stream.
 * Other threads should push outbound TCP messages to this class.
 * Must handle connection issues
 */
public class Connection
{
#if ENABLE_WINMD_SUPPORT
    // Connection details
    public string Port;
    public StreamSocketListener SocketListener;
    // Queued messages to send to server
    private BufferBlock<byte[]> MessageQueue;

    private Connection(string port, StreamSocketListener socketListener, BufferBlock<byte[]> messageQueue)
    {
        Port = port;
        SocketListener = socketListener;
        MessageQueue = messageQueue;
    }

    public static async Task<Connection> CreateAsync()
    {
        var conn = new Connection("9090", new StreamSocketListener(), new BufferBlock<byte[]>());
        conn.MessageQueue.Complete(); // Initialize with "dead" message queue
        conn.SocketListener.Control.KeepAlive = false;
        conn.SocketListener.ConnectionReceived += async (sender, args) =>
        {
            Debug.Log("Connection received");
            // get rid of old MessageQueue, which has completed
            var oldMessageQueue = Interlocked.Exchange(ref conn.MessageQueue, new BufferBlock<byte[]>());
            try
            {
                while (true)
                {
                    // wait until we have a message to send
                    var msg = await conn.MessageQueue.ReceiveAsync();
                    // compose and send a chunked message
                    using (var dw = new DataWriter(args.Socket.OutputStream))
                    {
                        dw.WriteBytes(msg);
                        await dw.StoreAsync();
                        // ...
                        await dw.FlushAsync();
                        dw.DetachStream();
                    }
                    Debug.Log($"Message {BitConverter.ToInt64(msg, 0)} sent");
                }
            }
            catch(Exception e)
            {
                Debug.Log($"Connection terminated: {e.Message}");
            }
            finally
            {
                // ensure MessageQueue won't process any more items
                conn.MessageQueue.Complete();
            }
        };
        Debug.Log($"OutboundSocket listening on port {conn.Port}");
        // may throw if port is unavailable
        await conn.SocketListener.BindServiceNameAsync(conn.Port);

        return conn;
    }

    // throws away data if connection is not established
    public async Task SendAsync(byte[] data)
    {
        await MessageQueue.SendAsync(data);
    }
#endif
}
