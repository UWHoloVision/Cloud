using System.Collections;
using System.Collections.Generic;
using UnityEngine;

#if ENABLE_WINMD_SUPPORT
using System;
using Windows.Networking;
using Windows.Networking.Sockets;
using Windows.Storage.Streams;
#endif

/*
 * After server connects to the Hololens, write to server through a stream.
 */
public class Connection
{
#if ENABLE_WINMD_SUPPORT
    StreamSocket socket;
    StreamSocketListener listener;
    String port;
    SensorFeedBehaviour sensorFeed;
    public IOutputStream outStream = null;
    public IInputStream inStream = null;
#endif

    public Connection(SensorFeedBehaviour sensorFeed)
    {
#if ENABLE_WINMD_SUPPORT
        listener = new StreamSocketListener();
        port = "9090";
        listener.ConnectionReceived += Listener_ConnectionReceived;
        listener.Control.KeepAlive = false;

        Listener_Start();
#endif
    }


#if ENABLE_WINMD_SUPPORT
    private async void Listener_Start()
    {
        Debug.Log("Listener started");
        try
        {
            await listener.BindServiceNameAsync(port);
        }
        catch (Exception e)
        {
            Debug.Log("Listener failed: " + e.Message);
        }

        Debug.Log("Listening");
    }

    private async void Listener_ConnectionReceived(StreamSocketListener sender, StreamSocketListenerConnectionReceivedEventArgs args)
    {
        Debug.Log("Connection received");

        try
        {
            while (true)
            {

                inStream = args.Socket.InputStream;
                outStream = args.Socket.OutputStream;
                using (var dw = new DataWriter(args.Socket.OutputStream))
                {
                    dw.WriteString("Hello There");
                    await dw.StoreAsync();
                    dw.DetachStream();
                }

                using (var dr = new DataReader(args.Socket.InputStream))
                {
                    dr.InputStreamOptions = InputStreamOptions.Partial;
                    await dr.LoadAsync(12);
                    var input = dr.ReadString(12);
                    Debug.Log("received: " + input);

                }
            }
        }
        catch (Exception e)
        {
            Debug.Log("disconnected!!!!!!!! " + e);
        }

    }
    
#endif
}
