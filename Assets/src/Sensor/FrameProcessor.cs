
using System;
#if ENABLE_WINMD_SUPPORT
using Windows.Media.Capture.Frames;
using Windows.Perception.Spatial;
using System.Numerics;
using System.Runtime.InteropServices;
#endif // ENABLE_WINMD_SUPPORT

/*
 * Process a MediaFrameReference to be serialized as a message
 */
public class FrameProcessor
{
#if ENABLE_WINMD_SUPPORT
    /*
     * recall that:
     * <App coordinate system> -> SpatialCoordinateSystem transform -> 
     * <Camera coordinate system> -> CameraViewTransform(Extrinsics) -> 
     * <Camera view space> -> CameraProjectionTransform -> <Camera projection space>
     * 
     * so to get from img -> world coordinate, go backwards
     */

    // Stores the coordinate system of the captured frame; IUnknown (SpatialCoordinateSystem)
    static readonly Guid MFSampleExtension_Spatial_CameraCoordinateSystem = new Guid("9D13C82F-2199-4E67-91CD-D1A4181F2534");
    // Stores the camera's extrinsic transform in the coordinate system; Blob (Matrix4x4)
    static readonly Guid MFSampleExtension_Spatial_CameraViewTransform = new Guid("4E251FA4830F4770859A4B8D99AA809B");
    // Stores the camera's projection transform; Blob (Matrix4x4)
    /* Projection transform:
       Matrix4x4 format    Terms
       m11 m12 m13 m14      fx    0   0   0
       m21 m22 m23 m24     skew  fy   0   0
       m31 m32 m33 m34      cx   cy   A  -1
       m41 m42 m43 m44       0    0   B   0
    */
    static readonly Guid MFSampleExtension_Spatial_CameraProjectionTransform = new Guid("47F9FCB5-2A02-4F26-A477-792FDF95886A");

    static Matrix4x4 ByteArrayToMatrix(byte[] bytes)
    {
        var matrix = Matrix4x4.Identity;
        var handle = GCHandle.Alloc(bytes, GCHandleType.Pinned);
        matrix = Marshal.PtrToStructure<Matrix4x4>(handle.AddrOfPinnedObject());
        handle.Free();
        return (matrix);
    }

    // Get transform matrices from the MediaFrameReference
    public static Tuple<Matrix4x4, Matrix4x4, Matrix4x4> GetTransforms(MediaFrameReference colorFrameRef, SpatialCoordinateSystem unityWorldCoordinateSystem)
    {
        SpatialCoordinateSystem spatialCoordinateSystem = null;
        Matrix4x4 projectionTransform = Matrix4x4.Identity; // intrinsics; does not change
        Matrix4x4 viewTransform = Matrix4x4.Identity; // extrinsics; changes per frame

        // TODO: Unity has CameraToWorldMatrix provided by PhotoCaptureFrame class... Cam space -> world space
        // also has worldToCameraMatrix, can it replace cameraCoordinateSystem transforms?
        // UnityEngine.Matrix4x4 camToWorld = UnityEngine.Camera.main.cameraToWorldMatrix;

        object value;
        if (colorFrameRef.Properties.TryGetValue(MFSampleExtension_Spatial_CameraCoordinateSystem, out value))
        {
            spatialCoordinateSystem = value as SpatialCoordinateSystem;
        }
        if (colorFrameRef.Properties.TryGetValue(MFSampleExtension_Spatial_CameraProjectionTransform, out value))
        {
            projectionTransform = ByteArrayToMatrix(value as byte[]);
        }
        if (colorFrameRef.Properties.TryGetValue(MFSampleExtension_Spatial_CameraViewTransform, out value))
        {
            viewTransform = ByteArrayToMatrix(value as byte[]);
        }

        // Transform: Camera Coord System -> Unity world coord
        // See https://github.com/Microsoft/MixedRealityToolkit-Unity/blob/96cc9ab8998280edcd6871f41e89584030ee4f26/Assets/HoloToolkit-Preview/QRTracker/Scripts/SpatialGraphCoordinateSystem.cs#L94
        var cameraRGBToWorldTransform = spatialCoordinateSystem.TryGetTransformTo(unityWorldCoordinateSystem);
        Matrix4x4 frameToOrigin = cameraRGBToWorldTransform.Value;
        // Unity is lhs coordinate system, UWP is rhs => must multiply z-coords by -1...
        // 3rd col
        frameToOrigin.M13 = -frameToOrigin.M13;
        frameToOrigin.M23 = -frameToOrigin.M23;
        frameToOrigin.M43 = -frameToOrigin.M43;
        // 3rd row
        frameToOrigin.M31 = -frameToOrigin.M31;
        frameToOrigin.M32 = -frameToOrigin.M32;
        frameToOrigin.M34 = -frameToOrigin.M34;

        return Tuple.Create(frameToOrigin, projectionTransform, viewTransform);
    }
#endif // ENABLE_WINMD_SUPPORT
}
