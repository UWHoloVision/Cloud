using System.Collections;
using System.Collections.Generic;
using System.Numerics;

/*
 * Serializable messsages to send down the wire
 */
public interface IRemoteMsg
{
    IRemoteMsg FromBytes(byte[] serialized);
    byte[] ToBytes();
}

public struct DepthMsg
{
    uint w;
    uint h;
    uint[] bitmap;
    Matrix4x4 frameToOrigin;
    
};