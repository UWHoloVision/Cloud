using System;
using System.Numerics;

#if ENABLE_WINMD_SUPPORT
using Windows.Storage.Streams;
#endif // ENABLE_WINMD_SUPPORT
/*
 * Message struct to be serialized into a byte array.
 * 
 * Note: Color images go up to 1.2 MB, while 
 * Depth images go up to 393.8 KB.
 * Absolute max TCP packet size goes up to 65535 bytes...
 * 
 * Matrix4x4 holds 16 * 4-byte [float]s.
 * 
 * Common header:
 * - (1 bytes)      | [ushort] message type
 * - (1 bytes)      | [ushort] length of header
 * - (2 bytes)      | [ushort] length of body
 */
public struct MessageHeader
{
#if ENABLE_WINMD_SUPPORT
    public byte MessageType;
    public byte HeaderLength;
    public ushort BodyLength;
    
    public static void Serialize(MessageHeader h, DataWriter w)
    {
        w.WriteByte(h.MessageType);
        w.WriteUInt16(h.HeaderLength);
        w.WriteUInt16(h.BodyLength);
    }
    public static MessageHeader Deserialize(DataReader r)
    {
        MessageHeader h = new MessageHeader();
        h.MessageType = r.ReadByte();
        h.HeaderLength = r.ReadByte();
        h.BodyLength = r.ReadUInt16();
        return h;
    }
#endif // ENABLE_WINMD_SUPPORT
};

public interface IFrameMessage
{
#if ENABLE_WINMD_SUPPORT
    void Serialize(DataWriter w);
    void Deserialize(DataReader r);
#endif // ENABLE_WINMD_SUPPORT
}


/*
 * 0 - Image Metadata
 * - (8 bytes)      | [ulong] frame identifier(system ticks)
 * - (2 bytes)      | [ushort] width of bitmap
 * - (2 bytes)      | [ushort] height of bitmap
 * - (2 bytes)      | [ushort] bytes per point
 * - (2 bytes)      | [ushort] points per pixel
 * - (64 bytes)     | [byte[]] FrameToOrigin matrix
 * - (64 bytes)     | [byte[]] Intrinsics matrix
 * - (64 bytes)     | [byte[]] Extrinsics matrix
 * = 208 bytes total
 */
public struct ImageMetadata : IFrameMessage
{
#if ENABLE_WINMD_SUPPORT
    public MessageHeader Header;
    public ulong FrameId;
    public ushort Width;
    public ushort Height;
    public ushort BytesPerPoint;
    public ushort PointsPerPixel;
    public Matrix4x4 FrameToOrigin;
    public Matrix4x4 Intrinsics;
    public Matrix4x4 Extrinsics;

    public static MessageHeader ImageMetadataHeader = new MessageHeader
    {
        MessageType = 0,
        HeaderLength = 4 + 208,
        BodyLength = 0
    };
    
    static void SerializeMatrix4x4(Matrix4x4 m, DataWriter w)
    {
        w.WriteSingle(m.M11);
        w.WriteSingle(m.M12);
        w.WriteSingle(m.M13);
        w.WriteSingle(m.M14);
        w.WriteSingle(m.M21);
        w.WriteSingle(m.M22);
        w.WriteSingle(m.M23);
        w.WriteSingle(m.M24);
        w.WriteSingle(m.M31);
        w.WriteSingle(m.M32);
        w.WriteSingle(m.M33);
        w.WriteSingle(m.M34);
        w.WriteSingle(m.M41);
        w.WriteSingle(m.M42);
        w.WriteSingle(m.M43);
        w.WriteSingle(m.M44);
    }
    static Matrix4x4 DeserializeMatrix4x4(DataReader r)
    {
        Matrix4x4 m = Matrix4x4.Identity;
        m.M11 = r.ReadSingle();
        m.M12 = r.ReadSingle();
        m.M13 = r.ReadSingle();
        m.M14 = r.ReadSingle();
        m.M21 = r.ReadSingle();
        m.M22 = r.ReadSingle();
        m.M23 = r.ReadSingle();
        m.M24 = r.ReadSingle();
        m.M31 = r.ReadSingle();
        m.M32 = r.ReadSingle();
        m.M33 = r.ReadSingle();
        m.M34 = r.ReadSingle();
        m.M41 = r.ReadSingle();
        m.M42 = r.ReadSingle();
        m.M43 = r.ReadSingle();
        m.M44 = r.ReadSingle();
        return m;
    }

    public void Deserialize(DataReader r)
    {
        Header = MessageHeader.Deserialize(r);
        FrameId = r.ReadUInt64();
        Width = r.ReadUInt16();
        Height = r.ReadUInt16();
        BytesPerPoint = r.ReadUInt16();
        PointsPerPixel = r.ReadUInt16();
        FrameToOrigin = DeserializeMatrix4x4(r);
        Intrinsics = DeserializeMatrix4x4(r);
        Extrinsics = DeserializeMatrix4x4(r);
    }

    public void Serialize(DataWriter w)
    {
        MessageHeader.Serialize(Header, w);
        w.WriteUInt64(FrameId);
        w.WriteUInt16(Width);
        w.WriteUInt16(Height);
        w.WriteUInt16(BytesPerPoint);
        w.WriteUInt16(PointsPerPixel);
        SerializeMatrix4x4(FrameToOrigin, w);
        SerializeMatrix4x4(Intrinsics, w);
        SerializeMatrix4x4(Extrinsics, w);
    }

#endif // ENABLE_WINMD_SUPPORT
}

/*
 * 1 - RGB Image chunk - 3 points per pixel, byte size
 * - (8 bytes)      | [ulong] frame identifier(system ticks)
 * - (4 bytes)      | [uint] start index
 * - (x bytes)      | [byte[]] data
 */
public struct ImageChunk : IFrameMessage
{
#if ENABLE_WINMD_SUPPORT
    public MessageHeader Header;
    public ulong FrameId;
    public uint Index;
    public byte[] Data;

    public static ushort CHUNK_SIZE = 32768;

    public static MessageHeader ImageChunkHeader = new MessageHeader()
    {
        MessageType = 1,
        HeaderLength = 4 + 8 + 4,
        BodyLength = CHUNK_SIZE
    };

    public void Deserialize(DataReader r)
    {
        Header = MessageHeader.Deserialize(r);
        FrameId = r.ReadUInt64();
        Index = r.ReadUInt32();
        Data = new byte[Header.BodyLength];
        r.ReadBytes(Data);
    }


    public void Serialize(DataWriter w)
    {
        MessageHeader.Serialize(Header, w);
        w.WriteUInt64(FrameId);
        w.WriteUInt32(Index);
        w.WriteBytes(Data);
    }
#endif // ENABLE_WINMD_SUPPORT
}