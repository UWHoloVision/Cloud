using UnityEngine;

using System.Text;

#if ENABLE_WINMD_SUPPORT
using System;
using System.Linq;
using System.Collections.Generic;
using Windows.Media.Capture;
using Windows.Media.Capture.Frames;
using Windows.Media.MediaProperties;
using Windows.Storage;
using Windows.Storage.Streams;
using Windows.Graphics.Imaging;
using System.Runtime.InteropServices.WindowsRuntime;
#endif // ENABLE_WINMD_SUPPORT

public class SensorFeedBehaviour : MonoBehaviour
{
    // See https://github.com/Microsoft/HoloLensForCV/issues/63#issuecomment-438583793
    const int DEPTH_LOWER_RANGE = 200;
    const int DEPTH_UPPER_RANGE = 1000;

#if ENABLE_WINMD_SUPPORT
    private StorageFolder rootImgFolder = null;
    private StorageFolder depthImgFolder = null;
#endif // ENABLE_WINMD_SUPPORT

    // BEGIN UNITY

    // Start is called before the first frame update
    async void Start()
    {

#if ENABLE_WINMD_SUPPORT
        // set up folders to save images to
        rootImgFolder = await ApplicationData.Current.TemporaryFolder
            .CreateFolderAsync($"cloud_{DateTimeOffset.UtcNow.ToUnixTimeSeconds()}");
        depthImgFolder = await rootImgFolder.CreateFolderAsync("depth");

        // find sensors
        var mfSourceGroups = await MediaFrameSourceGroup.FindAllAsync();
        var mfDepthSourceInfos = mfSourceGroups
            .SelectMany(group => group.SourceInfos)
            .Where(IsShortThrowDepthSensor);

        Debug.Assert(mfSourceGroups.Count == 1);
        var mfShortThrowDepth = mfDepthSourceInfos.First();
        Debug.Log("ShortThrow Depth Sensor acquired");

        var mediaCapture = new MediaCapture();
        await mediaCapture.InitializeAsync(new MediaCaptureInitializationSettings
        {
            SourceGroup = mfShortThrowDepth.SourceGroup,
            SharingMode = MediaCaptureSharingMode.SharedReadOnly,
            StreamingCaptureMode = StreamingCaptureMode.Video,
            MemoryPreference = MediaCaptureMemoryPreference.Cpu
        });
        var mfShortThrowDepthSource = mediaCapture.FrameSources[mfShortThrowDepth.Id];
        var mfShortThrowDepthReader = await mediaCapture.CreateFrameReaderAsync(mfShortThrowDepthSource, MediaEncodingSubtypes.D16);
        mfShortThrowDepthReader.FrameArrived += FrameArrived;

        // start sensor
        await mfShortThrowDepthReader.StartAsync();

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

    // Converts a SoftwareBitmap to a PGM file, filtering out depth points with [upper, lower)
    private static byte[] ConvertFrameToPGM(SoftwareBitmap inputBitmap, int upper, int lower)
    {
        var w = inputBitmap.PixelWidth;
        var h = inputBitmap.PixelHeight;
        // PGM header
        var header = $"P5\n{w} {h}\n65535\n";
        byte[] headerBuf = Encoding.ASCII.GetBytes(header);

        // Byte array of 16-bit pixel values
        byte[] inBuf = new byte[2 * w * h];
        inputBitmap.CopyToBuffer(inBuf.AsBuffer());

        // Filter pixels outside of range [200, 1000)
        for(var i = 0; i < inBuf.Length; i += 2)
        {
            ushort pixel = BitConverter.ToUInt16(inBuf, i);
            if (pixel < lower || pixel > upper) // invalid pixel
            {
                inBuf[i] = 0;
                inBuf[i + 1] = 0;
            }
        }
        // Outbound PGM buffer
        byte[] outBuf = new byte[inBuf.Length + headerBuf.Length];
        // copy header
        System.Buffer.BlockCopy(headerBuf, 0, outBuf, 0, headerBuf.Length);
        // copy pixel values
        System.Buffer.BlockCopy(inBuf, 0, outBuf, headerBuf.Length, inBuf.Length);
        return outBuf;
    }
    
    private async void FrameArrived(MediaFrameReader sender, MediaFrameArrivedEventArgs args)
    {
        var mfRef = sender.TryAcquireLatestFrame();
        if (mfRef != null)
        {
            var softwareBitmap = mfRef?.VideoMediaFrame?.SoftwareBitmap;
            if (softwareBitmap != null)
            {
                var systemTicks = mfRef.SystemRelativeTime.Value.Duration().Ticks;
                var outputFile = await depthImgFolder.CreateFileAsync($"{systemTicks.ToString()}.pgm");

                using (var outStream = await outputFile.OpenAsync(FileAccessMode.ReadWrite))
                using (var writer = new DataWriter(outStream))
                {
                    var pgmBytes = ConvertFrameToPGM(softwareBitmap, DEPTH_UPPER_RANGE, DEPTH_LOWER_RANGE);
                    var w = softwareBitmap.PixelWidth;
                    var h = softwareBitmap.PixelHeight;

                    writer.WriteBytes(pgmBytes);
                    await writer.StoreAsync();
                    await writer.FlushAsync();
                    writer.DetachStream();
                    Debug.Log($"File written to {outputFile.Path.ToString()}!");
                }
            }
            mfRef.Dispose();
        }
    }
#endif // ENABLE_WINMD_SUPPORT
}
