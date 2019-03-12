// Constructs FrameMessages
using System;
using System.Linq;
using System.Numerics;

#if ENABLE_WINMD_SUPPORT
#endif // ENABLE_WINMD_SUPPORT

public class MessageComposer
{
#if ENABLE_WINMD_SUPPORT
    public struct Payload
    {
        public long FrameId;
        public int Width;
        public int Height;
        public int BytesPerPoint;
        public int PointsPerPixel;
        public Matrix4x4 FrameToOrigin;
        public Matrix4x4 Intrinsics;
        public Matrix4x4 Extrinsics;
        public byte[] Data;
    };

    // helper for serializing Matrices
    static byte[] GetMatrix(Matrix4x4 m)
    {
        float[] matrix =
        {
            m.M11, m.M12, m.M13, m.M14,
            m.M21, m.M22, m.M23, m.M24,
            m.M31, m.M32, m.M33, m.M34,
            m.M41, m.M42, m.M43, m.M44,
        };
        return matrix.SelectMany(BitConverter.GetBytes).ToArray();
    }

    // returns byte[] header
    public static byte[] GetHeader(int bodyLength) {
        var header = new int[] { 0, bodyLength };
        header[0] = header.Length;
        
        return header.SelectMany(BitConverter.GetBytes).ToArray();
    }

    public static byte[] GetMessage(Payload p)
    {
        var body = BitConverter.GetBytes(p.FrameId)
            .Concat(BitConverter.GetBytes(p.Width))
            .Concat(BitConverter.GetBytes(p.Height))
            .Concat(BitConverter.GetBytes(p.BytesPerPoint))
            .Concat(BitConverter.GetBytes(p.PointsPerPixel))
            .Concat(GetMatrix(p.FrameToOrigin))
            .Concat(GetMatrix(p.Intrinsics))
            .Concat(GetMatrix(p.Extrinsics))
            .Concat(p.Data)
            .ToArray();

        return GetHeader(body.Length).Concat(body).ToArray();
    }
#endif // ENABLE_WINMD_SUPPORT
}
