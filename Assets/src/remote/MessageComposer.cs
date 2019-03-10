// Constructs FrameMessages
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Threading.Tasks;

#if ENABLE_WINMD_SUPPORT
using Windows.Storage.Streams;
#endif // ENABLE_WINMD_SUPPORT

public class MessageComposer
{
#if ENABLE_WINMD_SUPPORT
    public struct Payload
    {
        public ulong FrameId;
        public ushort Width;
        public ushort Height;
        public ushort BytesPerPoint;
        public ushort PointsPerPixel;
        public Matrix4x4 FrameToOrigin;
        public Matrix4x4 Intrinsics;
        public Matrix4x4 Extrinsics;
        public byte[] Data;
    };

    public static void ComposeDepth(Payload p, ref DataWriter w)
    {
        UnityEngine.Debug.Log("Composing ImageMetaData");
        ImageMetadata meta = new ImageMetadata
        {
            Header = ImageMetadata.ImageMetadataHeader,
            FrameId = p.FrameId,
            Width = p.Width,
            Height = p.Height,
            BytesPerPoint = p.BytesPerPoint,
            PointsPerPixel = p.PointsPerPixel,
            FrameToOrigin = p.FrameToOrigin,
            Intrinsics = p.Intrinsics,
            Extrinsics = p.Extrinsics
        };
        meta.Serialize(w);

        UnityEngine.Debug.Log("Completed sending metadata");
        // TODO: async
        for (uint Index = 0; Index < p.Data.Length; Index += ImageChunk.CHUNK_SIZE)
        {
            UnityEngine.Debug.Log($"Sending chunk at {Index}");
            // TODO: fix this
            byte[] data = p.Data.Skip((int)Index).Take(ImageChunk.CHUNK_SIZE).ToArray();
            ImageChunk chunk = new ImageChunk
            {
                Header = ImageChunk.ImageChunkHeader,
                FrameId = p.FrameId,
                Index = Index,
                Data = data
            };
            chunk.Serialize(w);
        }

        UnityEngine.Debug.Log("Completed sending chunks");
    }
#endif // ENABLE_WINMD_SUPPORT
}
