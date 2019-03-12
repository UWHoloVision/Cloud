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

    // TODO: consider coming up with a different mechanism; a lock, or 2 1-size buffers so we can sync
    // Queued messages to send to server
    private BufferBlock<MessageComposer.Payload> MessageQueue;
    
    private Connection(string port, StreamSocketListener socketListener, BufferBlock<MessageComposer.Payload> messageQueue)
    {
        Port = port;
        SocketListener = socketListener;
        MessageQueue = messageQueue;
    }

    static DataflowBlockOptions BlockOptions = new DataflowBlockOptions {
        BoundedCapacity = 1
    };

    public static async Task<Connection> CreateAsync()
    {
        var conn = new Connection("9090", new StreamSocketListener(), new BufferBlock<MessageComposer.Payload>(BlockOptions));
        conn.MessageQueue.Complete(); // Initialize with "dead" message queue
        conn.SocketListener.Control.KeepAlive = false;
        conn.SocketListener.ConnectionReceived += async (sender, args) =>
        {
            Debug.Log("Connection received");
            // get rid of old MessageQueue, which has completed
            var oldMessageQueue = Interlocked.Exchange(ref conn.MessageQueue, new BufferBlock<MessageComposer.Payload>(BlockOptions));
            try
            {
                while (true)
                {
                    // wait until we have a message to send
                    var msg = await conn.MessageQueue.ReceiveAsync();
                    Debug.Log($"ID: {msg.FrameId}");
                    Debug.Log($"FrameToOrigin: {msg.FrameToOrigin.ToString()}");
                    Debug.Log($"Intrinsics: {msg.Intrinsics.ToString()}");
                    Debug.Log($"Extrinsics: {msg.Extrinsics.ToString()}");
                    // compose and send a chunked message
                    using (var dw = new DataWriter(args.Socket.OutputStream))
                    {
                        dw.WriteBytes(MessageComposer.GetMessage(msg));
                        await dw.StoreAsync();
                        await dw.FlushAsync();
                        dw.DetachStream();
                    }
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
    public async Task SendAsync(MessageComposer.Payload p)
    {
        await MessageQueue.SendAsync(p);
    }
#endif
}
