using UnityEngine;

public class MeshMessage
{
    public struct MeshData
    {
        public int NPoints;
        public int NTriangles;
        public Vector3[] points;
        public int[] triangles;
    };

}
