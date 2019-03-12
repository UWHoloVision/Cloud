using UnityEngine;

using System;
using System.Threading;
using System.Runtime.InteropServices;
using UnityEngine.XR.WSA;
using System.Runtime.InteropServices.WindowsRuntime;
using System.Text;

#if ENABLE_WINMD_SUPPORT
using Windows.Media.Capture.Frames;
using Windows.Perception.Spatial;
using Windows.Graphics.Imaging;
using Windows.Storage.Streams;
#endif // ENABLE_WINMD_SUPPORT

public class SensorFeedBehaviour : MonoBehaviour
{
#if ENABLE_WINMD_SUPPORT
    private Connection conn = null;
    private FrameWriter frameWriter = null;
    private long latestDepth = 0;
    private long latestColor = 0;
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
    private async void FrameArrived(MediaFrameReader sender, MediaFrameArrivedEventArgs args)
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
        var frameToOrigin = tup.Item1;
        var intrinsics = tup.Item2;
        var extrinsics = tup.Item3;
        // Get System Ticks as a time value
        var systemTicks = mfRef.SystemRelativeTime.Value.Duration().Ticks;

        switch(softwareBitmap.BitmapPixelFormat)
        {
            case BitmapPixelFormat.Bgra8: // Color
                // check for duplicate frame
                if (Interlocked.Exchange(ref latestColor, systemTicks) == systemTicks)
                    break;
                if (frameWriter != null) // debug
                    await frameWriter.writeColorPNG(softwareBitmap, systemTicks);
                break;

            case BitmapPixelFormat.Gray16: // Depth
                // check for duplicate frame
                if (Interlocked.Exchange(ref latestDepth, systemTicks) == systemTicks)
                    break;
                if (frameWriter != null) // debug
                    await frameWriter.writeDepthPGM(softwareBitmap, systemTicks);

                var w = softwareBitmap.PixelWidth;
                var h = softwareBitmap.PixelHeight;
                byte[] data = new byte[2 * w * h];
                softwareBitmap.CopyToBuffer(data.AsBuffer());
                var p = new MessageComposer.Payload {
                    FrameId = (ulong)systemTicks,
                    Width = (ushort)softwareBitmap.PixelWidth,
                    Height = (ushort)softwareBitmap.PixelHeight,
                    BytesPerPoint = 8,
                    PointsPerPixel = 1,
                    FrameToOrigin = frameToOrigin,
                    Intrinsics = intrinsics,
                    Extrinsics = extrinsics,
                    Data = data
                };
                // Debug
                var d = BitConverter.GetBytes(systemTicks);
                await conn.SendAsync(d);

                break;

            default:
                Debug.LogError("Invalid Frame");
                break;
        }
        mfRef.Dispose();
    }
    
#endif // ENABLE_WINMD_SUPPORT
}
