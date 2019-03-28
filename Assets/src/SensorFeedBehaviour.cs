using UnityEngine;

using System;
using System.Threading;
using System.Runtime.InteropServices;
using UnityEngine.XR.WSA;
using System.Runtime.InteropServices.WindowsRuntime;

#if ENABLE_WINMD_SUPPORT
using Windows.Media.Capture.Frames;
using Windows.Perception.Spatial;
using Windows.Graphics.Imaging;
#endif // ENABLE_WINMD_SUPPORT

public class SensorFeedBehaviour : MonoBehaviour
{
#if ENABLE_WINMD_SUPPORT
    private Connection conn = null;
    private FrameWriter frameWriter = null;
    private SpatialCoordinateSystem unityWorldCoordinateSystem = null;

    // Start is called before the first frame update
    async void Start()
    {
        // TODO: do we need this? Which spacialcoordinates should we use?
        WorldAnchor anchor = gameObject.AddComponent<WorldAnchor>();
        // Initialize Unity world coordinate system
        unityWorldCoordinateSystem = Marshal.GetObjectForIUnknown(
            WorldManager.GetNativeISpatialCoordinateSystemPtr()) as SpatialCoordinateSystem;
        // TODO: Initialize connection
        conn = await Connection.CreateAsync();
        // Create FrameWriter if we want to save frames to Hololens
        // frameWriter = await FrameWriter.CreateFrameWriterAsync();

        // find sensors
        var mfSourceGroups = await MediaFrameSourceGroup.FindAllAsync();
        var mfColorReader = await HololensSensors.CreateMediaFrameReader(
            HololensSensors.ColorConfig, mfSourceGroups);
        var mfDepthReader = await HololensSensors.CreateMediaFrameReader(
            HololensSensors.DepthConfig, mfSourceGroups);

        mfDepthReader.FrameArrived += FrameArrived;
        mfColorReader.FrameArrived += FrameArrived;

        // start sensors
        await mfDepthReader.StartAsync();
        await mfColorReader.StartAsync();
    }

    // Update is called once per frame
    void Update()
    {
        
    }
    
    // Glorious FrameArrived Handler
    private void FrameArrived(MediaFrameReader sender, MediaFrameArrivedEventArgs args)
    {
        var mfRef = sender.TryAcquireLatestFrame();
        var softwareBitmap = mfRef?.VideoMediaFrame?.SoftwareBitmap;

        if (mfRef == null || softwareBitmap == null)
        {
            mfRef?.Dispose();
            return;
        }
        // Get Transform matrices
        var tup = FrameProcessor.GetTransforms(mfRef, unityWorldCoordinateSystem);
        if (tup == null)
        {
            return;
        }
        var frameToOrigin = tup.Item1;
        var intrinsics = tup.Item2;
        var extrinsics = tup.Item3;
        // Get System Ticks as a time value
        var systemTicks = mfRef.SystemRelativeTime.Value.Duration().Ticks;
        var w = softwareBitmap.PixelWidth;
        var h = softwareBitmap.PixelHeight;
        byte[] data;

        switch (softwareBitmap.BitmapPixelFormat)
        {
            case BitmapPixelFormat.Bgra8: // Color
                if (frameWriter != null) // debug
                    frameWriter.writeColorPNG(softwareBitmap, systemTicks);
                // Bgra8 has 4 bytes per pixel, apparently
                data = new byte[4 * w * h];
                softwareBitmap.CopyToBuffer(data.AsBuffer());
                var buf = new byte[3 * w * h];
                var c = 0;
                // get rid of useless byte
                for (var i = 0; i < w * h; i++)
                {
                    for (var j = 0; j < 3; j++)
                    {
                        buf[c] = data[i * 4 + 2 - j];
                        c++;
                    }
                }
                conn.SendColorFrame(new MessageComposer.Payload
                {
                    FrameId = systemTicks,
                    Width = softwareBitmap.PixelWidth,
                    Height = softwareBitmap.PixelHeight,
                    BytesPerPoint = 1,
                    PointsPerPixel = 3,
                    FrameToOrigin = frameToOrigin,
                    Intrinsics = intrinsics,
                    Extrinsics = extrinsics,
                    Data = buf
                });
                break;

            case BitmapPixelFormat.Gray16: // Depth
                if (frameWriter != null) // debug
                    frameWriter.writeDepthPGM(softwareBitmap, systemTicks);

                data = new byte[2 * w * h];
                softwareBitmap.CopyToBuffer(data.AsBuffer());
                conn.SendDepthFrame(new MessageComposer.Payload
                {
                    FrameId = systemTicks,
                    Width = softwareBitmap.PixelWidth,
                    Height = softwareBitmap.PixelHeight,
                    BytesPerPoint = 2,
                    PointsPerPixel = 1,
                    FrameToOrigin = frameToOrigin,
                    Intrinsics = intrinsics,
                    Extrinsics = extrinsics,
                    Data = data
                });

                break;

            default:
                Debug.LogError("Invalid Frame");
                break;
        }
        mfRef.Dispose();
    }
    
#endif // ENABLE_WINMD_SUPPORT
}
