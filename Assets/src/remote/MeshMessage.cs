using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;

#if ENABLE_WINMD_SUPPORT
using Windows.Networking.Sockets;
using Windows.Storage.Streams;
#endif // ENABLE_WINMD_SUPPORT

public class MeshMessage
{
    public struct MeshData
    {
        public int NPoints;
        public int NTriangles;
        public Vector3[] points;
        public int[] triangles;
    };

#if ENABLE_WINMD_SUPPORT
    public static async Task<MeshData> ReadMeshData(DataReader r)
    {
        await r.LoadAsync(sizeof(Int32));
        var NPoints = r.ReadInt32();
        await r.LoadAsync(sizeof(Int32));
        var NTriangles = r.ReadInt32();
        Vector3[] points = new Vector3[NPoints];
        int[] triangles = new int[NTriangles * 3];
        Debug.Log("Reading points");
        for(var i = 0; i < NPoints; i++)
        {
            await r.LoadAsync(sizeof(Single));
            var x = r.ReadSingle();
            await r.LoadAsync(sizeof(Single));
            var y = r.ReadSingle();
            await r.LoadAsync(sizeof(Single));
            var z = r.ReadSingle();
            points[i] = new Vector3(x, y, z);
        }
        Debug.Log("Reading triangles");
        for(var i = 0; i < NTriangles * 3; i++)
        {
            await r.LoadAsync(sizeof(Int32));
            triangles[i] = r.ReadInt32();
        }
        return new MeshData() {
            NPoints = NPoints,
            NTriangles = NTriangles,
            points = points,
            triangles = triangles
        };
    }
#endif // ENABLE_WINMD_SUPPORT
}
