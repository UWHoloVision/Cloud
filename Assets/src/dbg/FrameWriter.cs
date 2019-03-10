using UnityEngine;

using System;
using System.Threading.Tasks;
using System.Text;

#if ENABLE_WINMD_SUPPORT
using Windows.Storage;
using Windows.Storage.Streams;
using Windows.Graphics.Imaging;
using System.Runtime.InteropServices.WindowsRuntime;
#endif // ENABLE_WINMD_SUPPORT
/*
 * Utility to write frames to Hololens file system
 */
public class FrameWriter
{
#if ENABLE_WINMD_SUPPORT

    // See https://github.com/Microsoft/HoloLensForCV/issues/63#issuecomment-438583793
    const int DEPTH_LOWER_RANGE = 200;
    const int DEPTH_UPPER_RANGE = 1000;

    private StorageFolder rootImgFolder;
    private StorageFolder depthImgFolder;
    private StorageFolder colorImgFolder;

    private FrameWriter(StorageFolder rootImgFolder, StorageFolder depthImgFolder, StorageFolder colorImgFolder)
    {
        this.rootImgFolder = rootImgFolder;
        this.depthImgFolder = depthImgFolder;
        this.colorImgFolder = colorImgFolder;
    }
    
    // Async constructor
    public static async Task<FrameWriter> CreateFrameWriterAsync()
    {
        // set up folders to save images to
        var rootImgFolder = await ApplicationData.Current.TemporaryFolder
            .CreateFolderAsync($"cloud_{DateTimeOffset.UtcNow.ToUnixTimeSeconds()}");
        var depthImgFolder = await rootImgFolder.CreateFolderAsync("depth");
        var colorImgFolder = await rootImgFolder.CreateFolderAsync("color");

        return new FrameWriter(rootImgFolder, depthImgFolder, colorImgFolder);
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
        for (var i = 0; i < inBuf.Length; i += 2)
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

    public async Task writeDepthPGM(SoftwareBitmap softwareBitmap, long systemTicks)
    {
        Debug.LogAssertion(softwareBitmap.BitmapPixelFormat == BitmapPixelFormat.Gray16); // depth
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

            Debug.Log($"PGM File written to {outputFile.Path.ToString()}!");
        }
    }

    public async Task writeColorPNG(SoftwareBitmap softwareBitmap, long systemTicks)
    {
        Debug.LogAssertion(softwareBitmap.BitmapPixelFormat == BitmapPixelFormat.Bgra8); // color
        var outputFile = await colorImgFolder.CreateFileAsync($"{systemTicks.ToString()}.png");
        using (var outStream = await outputFile.OpenAsync(FileAccessMode.ReadWrite))
        {
            var encoder = await BitmapEncoder.CreateAsync(BitmapEncoder.PngEncoderId, outStream);
            encoder.SetSoftwareBitmap(softwareBitmap);
            await encoder.FlushAsync();

            Debug.Log($"PNG File written to {outputFile.Path.ToString()}!");
        }
    }
    
#endif // ENABLE_WINMD_SUPPORT
}
