using UnityEngine;

using System.Text;
using System.Threading;

#if ENABLE_WINMD_SUPPORT
using System;
using System.Linq;
using System.Collections.Generic;
using Windows.Media.Capture;
using Windows.Media.Capture.Frames;
using Windows.Media.MediaProperties;

#endif // ENABLE_WINMD_SUPPORT

public class SensorFeedBehaviour : MonoBehaviour
{
    // See https://github.com/Microsoft/HoloLensForCV/issues/63#issuecomment-438583793
    const int DEPTH_LOWER_RANGE = 200;
    const int DEPTH_UPPER_RANGE = 1000;

#if ENABLE_WINMD_SUPPORT
    private Connection conn = null;
    private FrameWriter frameWriter = null;
    private long latestDepth = 0;
    private long latestColor = 0;
#endif // ENABLE_WINMD_SUPPORT

    // BEGIN UNITY

    // Start is called before the first frame update
    async void Start()
    {
#if ENABLE_WINMD_SUPPORT
        conn = new Connection(this);
        frameWriter = await FrameWriter.CreateFrameWriterAsync();

        // find sensors
        var mfSourceGroups = await MediaFrameSourceGroup.FindAllAsync();
        var mfDepthSourceInfos = mfSourceGroups
            .SelectMany(group => group.SourceInfos)
            .Where(IsShortThrowDepthSensor);

        var mfColorSourceInfos = mfSourceGroups
            .SelectMany(group => group.SourceInfos)
            .Where(IsColorSensor);

        Debug.Assert(mfDepthSourceInfos.Count() == 1);
        Debug.Assert(mfColorSourceInfos.Count() == 1);
        var mfShortThrowDepth = mfDepthSourceInfos.First();
        Debug.Log("ShortThrow Depth Sensor acquired");
        var mfColor = mfColorSourceInfos.First();
        Debug.Log("Color Sensor acquired");

        var mcDepth = new MediaCapture();
        await mcDepth.InitializeAsync(new MediaCaptureInitializationSettings
        {
            SourceGroup = mfShortThrowDepth.SourceGroup,
            SharingMode = MediaCaptureSharingMode.SharedReadOnly,
            StreamingCaptureMode = StreamingCaptureMode.Video,
            MemoryPreference = MediaCaptureMemoryPreference.Cpu
        });
        var mfShortThrowDepthSource = mcDepth.FrameSources[mfShortThrowDepth.Id];
        var mfShortThrowDepthReader = await mcDepth.CreateFrameReaderAsync(mfShortThrowDepthSource, MediaEncodingSubtypes.D16);
        mfShortThrowDepthReader.FrameArrived += DepthFrameArrived;


        var mcColor = new MediaCapture();
        await mcColor.InitializeAsync(new MediaCaptureInitializationSettings
        {
            SourceGroup = mfColor.SourceGroup,
            SharingMode = MediaCaptureSharingMode.SharedReadOnly,
            StreamingCaptureMode = StreamingCaptureMode.Video,
            MemoryPreference = MediaCaptureMemoryPreference.Cpu
        });
        var mfColorSource = mcColor.FrameSources[mfColor.Id];
        var mfColorReader = await mcColor.CreateFrameReaderAsync(mfColorSource, MediaEncodingSubtypes.Bgra8);
        mfColorReader.FrameArrived += ColorFrameArrived;

        // start sensors
        await mfShortThrowDepthReader.StartAsync();
        await mfColorReader.StartAsync();

#endif // ENABLE_WINMD_SUPPORT

    }

    // Update is called once per frame
    void Update()
    {
        
    }


    // END UNITY
#if ENABLE_WINMD_SUPPORT

    static bool IsShortThrowDepthSensor(MediaFrameSourceInfo si)
    {
        return (new List<Func<MediaFrameSourceInfo, bool>>
        {
            x => x.MediaStreamType == MediaStreamType.VideoRecord,
            x => x.SourceKind == MediaFrameSourceKind.Depth,
            x => x.Id.Contains("Source#0")
        })
        .Aggregate(true, (acc, cond) => acc && cond(si));
    }
    static bool IsColorSensor(MediaFrameSourceInfo si)
    {
        return (new List<Func<MediaFrameSourceInfo, bool>>
        {
            x => x.MediaStreamType == MediaStreamType.VideoRecord,
            x => x.SourceKind == MediaFrameSourceKind.Color,
        })
        .Aggregate(true, (acc, cond) => acc && cond(si));
    }
    
    private async void DepthFrameArrived(MediaFrameReader sender, MediaFrameArrivedEventArgs args)
    {
        var mfRef = sender.TryAcquireLatestFrame();
        var softwareBitmap = mfRef?.VideoMediaFrame?.SoftwareBitmap;

        if (mfRef == null) return;
        if (softwareBitmap != null)
        {
            var systemTicks = mfRef.SystemRelativeTime.Value.Duration().Ticks;
            // check for duplicate frame
            if (Interlocked.Exchange(ref latestDepth, systemTicks) != systemTicks)
            {
                await frameWriter.writeDepthPGM(softwareBitmap, systemTicks);
            }
            mfRef.Dispose();
        }
    }

    private async void ColorFrameArrived(MediaFrameReader sender, MediaFrameArrivedEventArgs args)
    {
        var mfRef = sender.TryAcquireLatestFrame();
        var softwareBitmap = mfRef?.VideoMediaFrame?.SoftwareBitmap;

        if (mfRef == null) return;
        if (softwareBitmap != null)
        {
            var systemTicks = mfRef.SystemRelativeTime.Value.Duration().Ticks;
            // check for duplicate frame
            if (Interlocked.Exchange(ref latestColor, systemTicks) != systemTicks)
            {
                await frameWriter.writeColorPNG(softwareBitmap, systemTicks);
            }
            mfRef.Dispose();
        }
    }
#endif // ENABLE_WINMD_SUPPORT
}
