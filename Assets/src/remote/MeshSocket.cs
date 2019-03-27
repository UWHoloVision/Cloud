using UnityEngine;
using System;
using System.Threading.Tasks;
using System.Linq;
using System.Threading;

#if ENABLE_WINMD_SUPPORT
using Windows.Networking.Sockets;
using Windows.Storage.Streams;
#endif // ENABLE_WINMD_SUPPORT

public class MeshSocket
{

#if ENABLE_WINMD_SUPPORT
    public string Port;
    public StreamSocketListener SocketListener;
    
    public Vector3[] vertices;
    public int[] triangles;
    public long MeshId; // atomic

    private MeshSocket(string port, StreamSocketListener socketListener)
    {
        Port = port;
        SocketListener = socketListener;
        MeshId = 0;
    }

    public static async Task<MeshSocket> CreateAsync()
    {
        var conn = new MeshSocket("9091", new StreamSocketListener());
        conn.SocketListener.Control.KeepAlive = false;
        conn.SocketListener.ConnectionReceived += async (sender, args) =>
        {
            Debug.Log("MeshSocket connected");
            try
            {
                while (true)
                {
                    using (var r = new DataReader(args.Socket.InputStream))
                    {
                        r.InputStreamOptions = InputStreamOptions.ReadAhead;
                        r.ByteOrder = ByteOrder.LittleEndian;
                        Debug.Log("Reading MeshData");
                        // var meshData = await MeshMessage.ReadMeshData(dr);

                        var a = await r.LoadAsync(4);
                        Debug.Log($"A: {a}");
                        var NPoints = r.ReadInt32();
                        var b = await r.LoadAsync(4);
                        Debug.Log($"B: {b}");
                        var NTriangles = r.ReadInt32();

                        Debug.Log($"Ns: {NPoints} {NTriangles}");
                        
                        Vector3[] points = new Vector3[NPoints];
                        int[] triangles = new int[NTriangles * 3];

                        uint n = 4 * 3 * (uint)NPoints;
                        Debug.Log($"Reading {n} points");
                        var c = await r.LoadAsync(n);
                        Debug.Log($"C: {c}");
                        for (var i = 0; i < NPoints; i++)
                        {
                            //await r.LoadAsync(4);
                            var x = r.ReadSingle();
                            //await r.LoadAsync(4);
                            var y = r.ReadSingle();
                            //await r.LoadAsync(4);
                            var z = r.ReadSingle();
                            points[i] = new Vector3(x, y, z);
                        }
                        n = 4 * 3 * (uint)NTriangles;
                        Debug.Log($"Reading {n} triangles");
                        await r.LoadAsync(n);
                        for (var i = 0; i < NTriangles * 3; i++)
                        {
                            // await r.LoadAsync(4);
                            triangles[i] = r.ReadInt32();
                        }

                        conn.vertices = points;
                        conn.triangles = triangles;

                        Interlocked.Add(ref conn.MeshId, 1);
                        r.DetachStream();
                    }

                }
            }
            catch (Exception e)
            {
                Debug.Log($"MeshSocket connection terminated: {e.Message}");
            }
        };
        Debug.Log($"MeshSocket listening on port {conn.Port}");
        await conn.SocketListener.BindServiceNameAsync(conn.Port);
        return conn;
    }
#endif // ENABLE_WINMD_SUPPORT
}
