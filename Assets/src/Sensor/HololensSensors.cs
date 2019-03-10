using UnityEngine;

using System;
using System.Linq;
using System.Collections.Generic;
using System.Threading.Tasks;

#if ENABLE_WINMD_SUPPORT
using Windows.Media.Capture;
using Windows.Media.Capture.Frames;
using Windows.Media.MediaProperties;
#endif // ENABLE_WINMD_SUPPORT

/*
 * Helper class for Hololens sensors
 */
public class HololensSensors
{
#if ENABLE_WINMD_SUPPORT
    // Config object
    public struct SensorConfig
    {
        public string Name;
        public string MediaEncodingSubtype;
        public Func<MediaFrameSourceInfo, bool> Selector;
    }
    // Configs
    public static readonly SensorConfig DepthConfig = new SensorConfig
    {
        Name = "Short Throw Depth Sensor",
        MediaEncodingSubtype = MediaEncodingSubtypes.D16,
        Selector = si => (
            si.MediaStreamType == MediaStreamType.VideoRecord &&
            si.SourceKind == MediaFrameSourceKind.Depth &&
            si.Id.Contains("Source#0")
        )
    };
    public static readonly SensorConfig ColorConfig = new SensorConfig
    {
        Name = "RGB Color Sensor",
        MediaEncodingSubtype = MediaEncodingSubtypes.Bgra8,
        Selector = si => (
            si.MediaStreamType == MediaStreamType.VideoRecord &&
            si.SourceKind == MediaFrameSourceKind.Color
        )
    };

    // MediaFrameReader
    public static async Task<MediaFrameReader> CreateMediaFrameReader(SensorConfig cfg, IReadOnlyList<MediaFrameSourceGroup> sourceGroups)
    {
        // Step 1. Find correct sourceInfo
        var sourceInfos = sourceGroups
            .SelectMany(group => group.SourceInfos)
            .Where(cfg.Selector);
        Debug.Assert(sourceInfos.Count() == 1);
        var sourceInfo = sourceInfos.First();
        // Step 2. Create MediaCapture
        var mediaCapture = new MediaCapture();
        await mediaCapture.InitializeAsync(new MediaCaptureInitializationSettings
        {
            SourceGroup = sourceInfo.SourceGroup,
            SharingMode = MediaCaptureSharingMode.SharedReadOnly,
            StreamingCaptureMode = StreamingCaptureMode.Video,
            MemoryPreference = MediaCaptureMemoryPreference.Cpu
        });
        // Step 3. Create MediaFrameReader
        var mediaFrameReader = await mediaCapture.CreateFrameReaderAsync(
            mediaCapture.FrameSources[sourceInfo.Id], cfg.MediaEncodingSubtype);
        // Step 4. Return MediaFrameReader, add callbacks then call StartAsync
        Debug.Log($"{cfg.Name} acquired");
        return mediaFrameReader;
    }
#endif // ENABLE_WINMD_SUPPORT
}
