using UnityEngine;
using System;
using System.Threading.Tasks;
using System.Threading;

#if ENABLE_WINMD_SUPPORT
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
    
    // update LatestDepthFrame as frequently as possible, unless locked by FrameArrived
    private SemaphoreSlim DepthLock;
    private MessageComposer.Payload? LatestDepthFrame;
    private long LatestDepthTicks; // atomic
    
    // update ClosestColorFrame only if it's closest to the current depthFrame
    private SemaphoreSlim ColorLock;
    private MessageComposer.Payload? ClosestColorFrame;
    private long LatestColorTicks; // atomic
    
    private Connection(string port, StreamSocketListener socketListener)
    {
        Port = port;
        SocketListener = socketListener;
        DepthLock = new SemaphoreSlim(1);
        LatestDepthFrame = null;
        LatestDepthTicks = 0;
        ColorLock = new SemaphoreSlim(1);
        ClosestColorFrame = null;
        LatestColorTicks = 0;
    }

    public async Task<Tuple<MessageComposer.Payload, MessageComposer.Payload>> GetFrames()
    {
        Tuple<MessageComposer.Payload, MessageComposer.Payload> tup = null;
        await ColorLock.WaitAsync();
        await DepthLock.WaitAsync();
        try {
            if (LatestDepthFrame != null && ClosestColorFrame != null)
            {
                tup = new Tuple<MessageComposer.Payload, MessageComposer.Payload>(
                    LatestDepthFrame.Value,
                    ClosestColorFrame.Value
                );
            }
        }
        finally
        {
            DepthLock.Release();
            ColorLock.Release();
        }
        return tup;
    }

    public async void SendColorFrame(MessageComposer.Payload p)
    {
        Debug.Log($"Sending color frame {p.FrameId}");
        var acq = await ColorLock.WaitAsync(100);
        if (!acq)
        {
            return;
        }
        try
        {
            // avoid duplicate frame
            if (Interlocked.Exchange(ref LatestColorTicks, p.FrameId) != p.FrameId)
            {
                var latest_depth = Interlocked.Read(ref LatestDepthTicks); // may be stale!
                var is_first = ClosestColorFrame == null;
                var is_closer = is_first || Math.Abs(latest_depth - p.FrameId) < Math.Abs(latest_depth - ClosestColorFrame.Value.FrameId);
                // update only if no previous Color frame, or if new frame is closer in time to latest depth
                if (is_closer)
                {
                    ClosestColorFrame = p;
                }
            }
        }
        finally
        {
            ColorLock.Release();
        }
    }

    public async void SendDepthFrame(MessageComposer.Payload p)
    {
        Debug.Log($"Sending depth frame {p.FrameId}");
        var acq = await DepthLock.WaitAsync(100);
        if (!acq)
        {
            return;
        }
        try
        {
            // avoid duplicate frame
            if (Interlocked.Exchange(ref LatestDepthTicks, p.FrameId) != p.FrameId) 
            {
                // update depth frame as frequently as possible
                LatestDepthFrame = p;
            }
        }
        finally
        {
            DepthLock.Release();
        }
    }

    public async Task ClearFrames()
    {
        await ColorLock.WaitAsync();
        await DepthLock.WaitAsync();
        try
        {
            LatestDepthFrame = null;
            ClosestColorFrame = null;

        }
        finally
        {
            DepthLock.Release();
            ColorLock.Release();
        }

    }

    public static async Task<Connection> CreateAsync()
    {
        var conn = new Connection("9090", new StreamSocketListener());
        conn.SocketListener.Control.KeepAlive = false;
        conn.SocketListener.ConnectionReceived += async (sender, args) =>
        {
            Debug.Log("Connection received");
            // get rid of old MessageQueue, which has completed
            // var oldMessageQueue = Interlocked.Exchange(ref conn.MessageQueue, new BufferBlock<MessageComposer.Payload>(BlockOptions));
            try
            {
                while (true)
                {
                    // wait for frame request from server
                    using (var dr = new DataReader(args.Socket.InputStream))
                    {
                        await dr.LoadAsync(1);
                    }
                    // get latest frames
                    Tuple<MessageComposer.Payload, MessageComposer.Payload> frames = null;
                    while (frames == null)
                    {
                        // until we have a pair of frames
                        frames = await conn.GetFrames();
                    }
                    // compose and send a chunked message
                    using (var dw = new DataWriter(args.Socket.OutputStream))
                    {
                        dw.WriteBytes(MessageComposer.GetMessage(frames.Item1)); // depth
                        await dw.StoreAsync();
                        dw.WriteBytes(MessageComposer.GetMessage(frames.Item2)); // color
                        await dw.StoreAsync();
                        // flush
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
                await conn.ClearFrames();
            }
        };
        Debug.Log($"OutboundSocket listening on port {conn.Port}");
        // may throw if port is unavailable
        await conn.SocketListener.BindServiceNameAsync(conn.Port);

        return conn;
    }
#endif
}
