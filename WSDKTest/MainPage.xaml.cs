using System;
using System.Diagnostics;
using System.Timers;
using System.Threading.Tasks;
using System.Runtime.InteropServices.WindowsRuntime;
using DJI.WindowsSDK;
using Windows.UI.Popups;
using Windows.UI.Xaml.Media.Imaging;
using ZXing;
using ZXing.Common;
using ZXing.Multi.QrCode;
using Windows.Graphics.Imaging;
using System.Collections.Generic;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Data;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Navigation;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.Storage;
using Windows.Storage.Streams;
using Windows.Media.Editing;
using Microsoft.Graphics.Canvas.UI.Xaml;
using Microsoft.Graphics.Canvas;
using Windows.Media.Transcoding;
using Windows.UI.Core;
using Windows.Foundation;
using System.IO;

namespace WSDKTest
{
    public sealed partial class MainPage : Page
    {
        private DJIVideoParser.Parser videoParser;
        public WriteableBitmap VideoSource;

        //Worker task (thread) for reading barcode
        //As reading barcode is computationally expensive
        private Task readerWorker = null;
        private ISet<string> readed = new HashSet<string>();

        private object bufLock = new object();
        //these properties are guarded by bufLock
        private int width, height;
        private byte[] decodedDataBuf;


        public MainPage()
        {
            this.InitializeComponent();
            //Listen for registration success
            DJISDKManager.Instance.SDKRegistrationStateChanged += async (state, result) =>
            {
                if (state != SDKRegistrationState.Succeeded)
                {
                    var md = new MessageDialog(result.ToString());
                    await Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, async () => await md.ShowAsync());
                    return;
                }
                //wait till initialization finish
                //use a large enough time and hope for the best
                await Task.Delay(1000);
                videoParser = new DJIVideoParser.Parser();
                videoParser.Initialize();
                videoParser.SetVideoDataCallack(0, 0, ReceiveDecodedData);
                DJISDKManager.Instance.VideoFeeder.GetPrimaryVideoFeed(0).VideoDataUpdated += OnVideoPush;

                await DJISDKManager.Instance.ComponentManager.GetFlightAssistantHandler(0, 0).SetObstacleAvoidanceEnabledAsync(new BoolMsg() { value = false });
                await DJISDKManager.Instance.ComponentManager.GetFlightAssistantHandler(0, 0).SetVisionAssistedPositioningEnabledAsync(new BoolMsg() { value = true });
                await DJISDKManager.Instance.ComponentManager.GetFlightAssistantHandler(0, 0).SetLandingProtectionEnabledAsync(new BoolMsg() { value = false });
                await DJISDKManager.Instance.ComponentManager.GetFlightAssistantHandler(0, 0).StartAlignedPushDataAsync();
                
                await Task.Delay(5000);
                
            };
            DJISDKManager.Instance.RegisterApp("3ad4ddfffc9f656725e12c34");

            //subscribing event handlers
            //DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).TakeoffLocationAltitudeChanged += getTakeoffLocationAltitudeChanged;
            DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).AttitudeChanged += GetAttitudeChanged;
            DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).AltitudeChanged += GetAltitudeChanged;
            DJISDKManager.Instance.ComponentManager.GetFlightAssistantHandler(0, 0).AlignedAircraftLocationChanged += GetAlignedAircraftLocationChanged;
            //DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).AircraftLocationChanged += getAircraftLocationChanged;

        }

        void OnVideoPush(VideoFeed sender, [ReadOnlyArray] ref byte[] bytes)
        {
            videoParser.PushVideoData(0, 0, bytes, bytes.Length);
        }

        void createWorker()
        {
            //create worker thread for reading barcode
            readerWorker = new Task(async () =>
            {
                //use stopwatch to time the execution, and execute the reading process repeatedly
                var watch = System.Diagnostics.Stopwatch.StartNew();
                var reader = new QRCodeMultiReader();
                SoftwareBitmap bitmap;
                HybridBinarizer binarizer;
                while (true)
                {
                    try
                    {
                        lock (bufLock)
                        {
                            bitmap = new SoftwareBitmap(BitmapPixelFormat.Bgra8, width, height);
                            bitmap.CopyFromBuffer(decodedDataBuf.AsBuffer());
                        }
                    }
                    catch
                    {
                        //the size maybe incorrect due to unknown reason
                        await Task.Delay(10);
                        continue;
                    }
                    var source = new SoftwareBitmapLuminanceSource(bitmap);
                    binarizer = new HybridBinarizer(source);
                    var results = reader.decodeMultiple(new BinaryBitmap(binarizer));
                    if (results != null && results.Length > 0)
                    {
                        await Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, () =>
                        {
                            foreach (var result in results)
                            {
                                if (!readed.Contains(result.Text))
                                {
                                    readed.Add(result.Text);
                                    Textbox.Text += result.Text + "\n";
                                }
                            }
                        });
                    }

                    //System.Diagnostics.Debug.WriteLine("Looping...\n");
                    watch.Stop();
                    int elapsed = (int)watch.ElapsedMilliseconds;

                    //broadcasting messages here
                    System.Diagnostics.Debug.WriteLine("currentAltitude:{0}", currentAltitude);
                    //System.Diagnostics.Debug.WriteLine("latitude:{0}\tlongitude:{1}", latitude, longitude);
                    System.Diagnostics.Debug.WriteLine("Aircraft heading direction:{0}", true_north_heading);
                    //System.Diagnostics.Debug.WriteLine("3D-plane coordinate: x={0}\ty={1}\tz={2}", N_position, E_position, D_position);
                    System.Diagnostics.Debug.WriteLine("2D-plane coordinate: x={0}\ty={1}", current2Dpostion.x, current2Dpostion.y);

                    //Update GUI View
                    await Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, () =>
                    {
                        StateBox.Text = "Current State: " + mission_state;
                        AltitudeBox.Text = "Current Altitude: " + currentAltitude;
                        PositionBox.Text = "Current Position: " + Math.Round(current2Dpostion.x, 5) + " " + Math.Round(current2Dpostion.y, 5);
                        RotationBox.Text = "Current Heading: " + true_north_heading;
                        if (current2Dpostion.x == 0 && current2Dpostion.y == 0)
                        {
                            Message.Text = "Initialization Failed. Restart the App after connected to drone.";
                        }
                    });

                    //run at max 5Hz
                    await Task.Delay(Math.Max(0, 200 - elapsed));
                    await DJISDKManager.Instance.ComponentManager.GetCameraHandler(0, 0).SetCameraColorAsync(new CameraColorMsg() { value = CameraColor.BLACK_WHITE });

                }
            });
        }

        async void ReceiveDecodedData(byte[] data, int width, int height)
        {
            //basically copied from the sample code
            lock (bufLock)
            {
                //lock when updating decoded buffer, as this is run in async
                //some operation in this function might overlap, so operations involving buffer, width and height must be locked
                if (decodedDataBuf == null)
                {
                    decodedDataBuf = data;
                }
                else
                {
                    if (data.Length != decodedDataBuf.Length)
                    {
                        Array.Resize(ref decodedDataBuf, data.Length);
                    }
                    data.CopyTo(decodedDataBuf.AsBuffer());
                    this.width = width;
                    this.height = height;
                }
            }
            await Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, () =>
            {
                //dispatch to UI thread to do UI update (image)
                //WriteableBitmap is exclusive to UI thread
                if (VideoSource == null || VideoSource.PixelWidth != width || VideoSource.PixelHeight != height)
                {
                    VideoSource = new WriteableBitmap((int)width, (int)height);
                    fpvImage.Source = VideoSource;
                    //Start barcode reader worker after the first frame is received
                    if (readerWorker == null)
                    {
                        createWorker();
                        readerWorker.Start();
                    }
                }
                lock (bufLock)
                {
                    //copy buffer to the bitmap and draw the region we will read on to notify the users
                    decodedDataBuf.AsBuffer().CopyTo(VideoSource.PixelBuffer);
                }
                //Invalidate cache and trigger redraw
                VideoSource.Invalidate();
                if(shooting_enabled)
                    SaveImage(VideoSource);
                if(video_enabled)
                    PrepareVideo(VideoSource, video_enabled);
            });
        }

        private async void Stop_Button_Click(object sender, RoutedEventArgs e)
        {
            SaveResult(Textbox.Text.ToString());
            string command = @"C:\Users\USER\AppData\Local\Packages\Sample_82w491b8nw3hm\LocalState\testpy.py";

            var options = new Windows.System.LauncherOptions();
            options.TreatAsUntrusted = true;
            bool success = await Windows.System.Launcher.LaunchUriAsync(new System.Uri(command), options);
            
                if (success)
                {
                    Message.Text = "success";
                }
                else
                {
                Message.Text = "not success";
            }
            
            throttle = 0;
            roll = 0;
            pitch = 0;
            yaw = 0;
            mission_state = -1;
            script_started = false;

            try
            {
                if (DJISDKManager.Instance != null)
                    DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue(throttle, yaw, pitch, roll);
            }
            catch (Exception err)
            {
                System.Diagnostics.Debug.WriteLine(err);
            }
        }

        private void Apply_Button_Click(object sender, RoutedEventArgs e)
        {
            rollPID.tune(Convert.ToDouble(rollP.Text), Convert.ToDouble(rollI.Text), Convert.ToDouble(rollD.Text));
            pitchPID.tune(Convert.ToDouble(pitchP.Text), Convert.ToDouble(pitchI.Text), Convert.ToDouble(pitchD.Text));
            yawPID.tune(Convert.ToDouble(yawP.Text), Convert.ToDouble(yawI.Text), Convert.ToDouble(yawD.Text));
            Message.Text = "PID updated";
        }
        private async void StartRecording_Button_Click(object sender, RoutedEventArgs e)
        {
            //await DJISDKManager.Instance.ComponentManager.GetCameraHandler(0, 0).StartRecordAsync();
            StartRecording();
        }

        private async void StopRecording_Button_Click(object sender, RoutedEventArgs e)
        {
            //await DJISDKManager.Instance.ComponentManager.GetCameraHandler(0, 0).StopRecordAsync();
            StartRecording();
        }

        private static float throttle = 0;
        private static float roll = 0;
        private static float pitch = 0;
        private static float yaw = 0;

        //custom var
        private static System.Timers.Timer TimerAsync;
        private static System.Timers.Timer VideoProtectionTimerAsync;
        bool script_started = false;
        static int mission_state = 0;
        static DateTime lastframe = DateTime.Now;
        static DateTime thisframe;
        static double currentAltitude;
        static double currentAltitudeNotBaro;
        static double latitude;
        static double longitude;
        static double true_north_heading;
        static double local_heading;
        static double N_position;
        static double E_position;
        static double D_position;
        static double Y_offset = 0;
        static double X_offset = 0;
        static double WP_tolarence = 0.07f;
        static double angle_tolarence = 0.2f;
        static float max_yaw_speed = 0.3f;
        static float max_speed = 0.2f;
        static float max_vert_speed = 0.5f;
        static double angle_remain;
        static double hold_rotation = 0;
        static int image_count = 1;
        static int video_count = 1;
        const int frame_rate = 30;
        bool video_enabled = false;
        bool video_saved = false;
        bool shooting_enabled = false;
        bool first_time = true;
        bool fast_yaw = false;
        MediaComposition composition = new MediaComposition();

        struct Position
        {
            public double x;
            public double y;
            public double rotation;
            public double tolarence;

            public Position(double x0, double y0, double rotation0 = 0, double tolarence0 = 1) : this()
            {
                x = x0;
                y = y0;
                rotation = rotation0;
                tolarence = tolarence0;
            }

            public double compare(Position target)
            {
                return Math.Sqrt((x - target.x) * (x - target.x) + (y - target.y) * (y - target.y));
            }

            public void updateYaw(Position currentPos)
            {
                int sign = 1;
                if (x - currentPos.x >= 0)          //to rightward
                    sign = 1;
                else if (x - currentPos.x < 0)      //to left
                    sign = -1;
                if (y - currentPos.y < 0)           //to backward
                {
                    rotation = 90.0 + sign * CompassBoundary(Math.Round(ToDegree(Math.Atan((x - currentPos.x) / (y - currentPos.y))), 1));
                }
                else                            //to forward
                {
                    rotation = sign * CompassBoundary(Math.Round(ToDegree(Math.Atan((x - currentPos.x) / (y - currentPos.y))), 1));
                }
            }

            public double offsetFromPath(Position lastWP, Position currentPos)
            {
                //return the distance between current position and the line alone last and next waypoint
                //-ve = LHS, +ve = RHS
                return ((y - lastWP.y) * currentPos.x - (x - lastWP.x) * currentPos.y + x * lastWP.y - y * lastWP.x) / Math.Sqrt((y - lastWP.y) * (y - lastWP.y) + (x - lastWP.x) * (x - lastWP.x));
            }
        }
        static Position current2Dpostion;
        static Position[] myWP = new Position[10];

        struct PID
        {
            double Kp;
            double Ki;
            double Kd;

            double integral;
            double derivative;
            double max_in;
            double last_error;
            float max_output;

            public PID(double p, double i, double d, float speed) : this()
            {
                Kp = p;
                Ki = i;
                Kd = d;

                integral = 0;
                derivative = 0;
                max_in = 0;
                last_error = 0;
                max_output = speed;
            }
            public float get(double error)
            {
                integral += error;
                if (integral >= max_in)
                    integral = max_in;
                else if (integral <= (-max_in))
                    integral = -max_in;
                derivative = error - last_error;
                last_error = error;
                
                float output;
                if(Ki == 0)
                    output = Convert.ToSingle(Kp * error + Kd * derivative);
                else
                    output = Convert.ToSingle(Kp * error + (1.0 / Ki) * integral + Kd * derivative);
                if (output >= max_output)
                    return max_output;
                else if (output <= -max_output)
                    return -max_output;
                else
                    return output;
            }
            public void tune(double p, double i, double d)
            {
                Kp = p;
                Ki = i;
                Kd = d;
            }
        }

        static PID rollPID = new PID(1, 0, 0.1, max_speed);
        static PID pitchPID = new PID(1.01, 0.1, 0.11, max_speed);
        static PID yawPID = new PID(0.00104, 0.001, 0.0045, max_yaw_speed);
        static PID throttlePID = new PID(1.5, 0, 0.1, max_vert_speed);
        static PID fastyawPID = new PID(2, 0, 0.1, 0.5f);

        public static double CompassBoundary(double compassValue)
        {
            compassValue %= 360.0f;
            if (compassValue > 180)
            {
                return -(360.0f - compassValue);
            }
            else if (compassValue < -180)
            {
                return 360.0f + compassValue;
            }
            else
            {
                return compassValue;
            }
        }
        public static double ToRadian(double angle)
        {
            return (Math.PI / 180) * angle;
        }

        public static double ToDegree(double rad)
        {
            return rad * (180.0 / Math.PI);
        }

        static Position RotationMatrix(Position originPlane, double degree)
        {
            double x0 = Math.Cos(ToRadian(degree)) * originPlane.x + -Math.Sin(ToRadian(degree)) * originPlane.y;
            double y0 = Math.Sin(ToRadian(degree)) * originPlane.x + Math.Cos(ToRadian(degree)) * originPlane.y;
            Position newPlane = new Position(x0, y0);
            return newPlane;
        }

        private void SetWP()
        {
            Position temp_wp;
            myWP[0].x = 0.0f;
            myWP[0].y = 0.0f;
            myWP[0].rotation = CompassBoundary(0.0f + local_heading);
            myWP[0].tolarence = WP_tolarence;

            temp_wp = RotationMatrix(new Position(2.0, 0), CompassBoundary(-local_heading));
            myWP[1].x = temp_wp.x;
            myWP[1].y = temp_wp.y;
            myWP[1].rotation = CompassBoundary(0.0f + local_heading);
            myWP[1].tolarence = WP_tolarence;

            temp_wp = RotationMatrix(new Position(0, 0), CompassBoundary(-local_heading));
            myWP[2].x = temp_wp.x;
            myWP[2].y = temp_wp.y;
            myWP[2].rotation = CompassBoundary(0.0f + local_heading);
            myWP[2].tolarence = WP_tolarence;

            temp_wp = RotationMatrix(new Position(2.0, 0), CompassBoundary(-local_heading));
            myWP[3].x = temp_wp.x;
            myWP[3].y = temp_wp.y;
            myWP[3].rotation = CompassBoundary(180.0f + local_heading);
            myWP[3].tolarence = WP_tolarence;

            temp_wp = RotationMatrix(new Position(0, 0), CompassBoundary(-local_heading));
            myWP[4].x = temp_wp.x;
            myWP[4].y = temp_wp.y;
            myWP[4].rotation = CompassBoundary(180.0f + local_heading);
            myWP[4].tolarence = WP_tolarence;

            temp_wp = RotationMatrix(new Position(2.0, 0), CompassBoundary(-local_heading));
            myWP[5].x = temp_wp.x;
            myWP[5].y = temp_wp.y;
            myWP[5].rotation = CompassBoundary(180.0f + local_heading);
            myWP[5].tolarence = WP_tolarence;

            temp_wp = RotationMatrix(new Position(0, 0), CompassBoundary(-local_heading));
            myWP[6].x = temp_wp.x;
            myWP[6].y = temp_wp.y;
            myWP[6].rotation = CompassBoundary(180.0f + local_heading);
            myWP[6].tolarence = WP_tolarence;

            for (int i = 0; i < myWP.Length; i++)
                System.Diagnostics.Debug.WriteLine("WP[{0}] {1} {2} {3}", i, myWP[i].x, myWP[i].y, myWP[i].rotation);
        }

        private async void SaveImage(WriteableBitmap bmp)
        {
            //Create folder
            StorageFolder myfolder = await Windows.Storage.ApplicationData.Current.LocalFolder.CreateFolderAsync("Images", CreationCollisionOption.OpenIfExists);
            string filename_img = "img" + image_count + ".jpeg";
            image_count++;
            StorageFile sampleFile = await myfolder.CreateFileAsync(filename_img, CreationCollisionOption.ReplaceExisting);
            byte[] jpg_buffer = await EncodeJpeg(bmp);
            await FileIO.WriteBytesAsync(sampleFile, jpg_buffer);
        }
        private async void SaveResult(string result)
        {
            string filename_csv = "result.csv";
            StorageFolder myfolder2 = await Windows.Storage.ApplicationData.Current.LocalFolder.CreateFolderAsync("Doc", CreationCollisionOption.OpenIfExists);
            StorageFile sampleFile2 = await myfolder2.CreateFileAsync(filename_csv, CreationCollisionOption.ReplaceExisting);
            File.WriteAllText(filename_csv, result);
        }
        private async Task<byte[]> EncodeJpeg(WriteableBitmap bmp)
        {
            SoftwareBitmap soft = SoftwareBitmap.CreateCopyFromBuffer(bmp.PixelBuffer, BitmapPixelFormat.Bgra8, bmp.PixelWidth, bmp.PixelHeight);
            byte[] array = null;

            using (var ms = new InMemoryRandomAccessStream())
            {
                BitmapEncoder encoder = await BitmapEncoder.CreateAsync(BitmapEncoder.JpegEncoderId, ms);
                encoder.SetSoftwareBitmap(soft);

                try
                {
                    await encoder.FlushAsync();
                }
                catch { }

                array = new byte[ms.Size];
                await ms.ReadAsync(array.AsBuffer(), (uint)ms.Size, InputStreamOptions.None);
            }

            return array;
        }

        private void PrepareVideo(WriteableBitmap bmp, bool EN)
        {
            if (EN)
            {
                SoftwareBitmap frame = SoftwareBitmap.CreateCopyFromBuffer(bmp.PixelBuffer, BitmapPixelFormat.Bgra8, bmp.PixelWidth, bmp.PixelHeight, BitmapAlphaMode.Premultiplied);
                CanvasRenderTarget rendertarget = null;
                using (CanvasBitmap canvas = CanvasBitmap.CreateFromSoftwareBitmap(CanvasDevice.GetSharedDevice(), frame))
                {
                    rendertarget = new CanvasRenderTarget(CanvasDevice.GetSharedDevice(), canvas.SizeInPixels.Width, canvas.SizeInPixels.Height, 96);
                    using (CanvasDrawingSession ds = rendertarget.CreateDrawingSession())
                    {
                        ds.Clear(Windows.UI.Colors.Black);
                        ds.DrawImage(canvas);
                    }
                }
                thisframe = DateTime.Now;
                MediaClip m = MediaClip.CreateFromSurface(rendertarget, thisframe - lastframe);
                lastframe = thisframe;
                composition.Clips.Add(m);
            }
        }
        
        private async Task SaveVideo(MediaComposition video_com)
        {
            StorageFolder myfolder = await Windows.Storage.ApplicationData.Current.LocalFolder.CreateFolderAsync("Videos", CreationCollisionOption.OpenIfExists);
            string filename_vid = "video" + video_count + ".mp4";
            video_count++;
            StorageFile videoFile = await myfolder.CreateFileAsync(filename_vid, CreationCollisionOption.ReplaceExisting);
            var saveOperation = video_com.RenderToFileAsync(videoFile, MediaTrimmingPreference.Precise);
            System.Diagnostics.Debug.WriteLine("Now saving: {0}", filename_vid);
            saveOperation.Progress = new AsyncOperationProgressHandler<TranscodeFailureReason, double>(async (info, progress) =>
            {
                await this.Dispatcher.RunAsync(CoreDispatcherPriority.Normal, new DispatchedHandler(() =>
                {
                    System.Diagnostics.Debug.WriteLine(string.Format("Saving file... Progress: {0:F0}%", progress));
                }));
            });
            saveOperation.Completed = new AsyncOperationWithProgressCompletedHandler<TranscodeFailureReason, double>(async (info, status) =>
            {
                await this.Dispatcher.RunAsync(CoreDispatcherPriority.Normal, new DispatchedHandler(() =>
                {
                    try
                    {
                        var results = info.GetResults();
                        if (results != TranscodeFailureReason.None || status != AsyncStatus.Completed)
                        {
                            System.Diagnostics.Debug.WriteLine("Saving was unsuccessful");
                        }
                        else
                        {
                            video_saved = true;
                            System.Diagnostics.Debug.WriteLine("Trimmed clip saved to file");
                        }
                    }
                    finally
                    {
                        // Update UI whether the operation succeeded or not
                    }

                }));
            });
        }

        private void StartRecording()
        {
            composition = new MediaComposition();
            lastframe = DateTime.Now;
            thisframe = DateTime.Now;
            video_enabled = true;
            video_saved = false;
            if (VideoProtectionTimerAsync != null)
            {
                VideoProtectionTimerAsync.Stop();
                VideoProtectionTimerAsync.Dispose();
            }
            SetVideoProtectionTimer();
            System.Diagnostics.Debug.WriteLine("Video Recording Started");
        }

        private void StopRecording()
        {
            if (video_enabled == true)
            {
                System.Diagnostics.Debug.WriteLine("Video Recording Ended");
                video_enabled = false;
                SaveVideo(composition);
                lastframe = DateTime.Now;
                thisframe = DateTime.Now;
                if (VideoProtectionTimerAsync != null)
                {
                    VideoProtectionTimerAsync.Stop();
                    VideoProtectionTimerAsync.Dispose();
                }
            }
        }

        private async void SetCamera()
        {
            //await DJISDKManager.Instance.ComponentManager.GetCameraHandler(0, 0).SetShutterSpeedAsync(new CameraShutterSpeedMsg() { value =  CameraShutterSpeed.SHUTTER_SPEED1_15});
            //await DJISDKManager.Instance.ComponentManager.GetCameraHandler(0, 0).SetCameraColorAsync(new CameraColorMsg() { value = CameraColor.BLACK_WHITE });
            //await DJISDKManager.Instance.ComponentManager.GetCameraHandler(0, 0).SetVideoResolutionAndFrameRateAsync(new VideoResolutionAndFrameRate() { resolution = VideoResolution.RESOLUTION_1280x720, frameRate = VideoFrameRate.RATE_120FPS });
        }
        private async void Grid_KeyUp(object sender, KeyRoutedEventArgs e)
        {
            switch (e.Key)
            {
                case Windows.System.VirtualKey.W:
                case Windows.System.VirtualKey.S:
                    {
                        throttle = 0;
                        break;
                    }
                case Windows.System.VirtualKey.A:
                case Windows.System.VirtualKey.D:
                    {
                        yaw = 0;
                        break;
                    }
                case Windows.System.VirtualKey.I:
                case Windows.System.VirtualKey.K:
                    {
                        pitch = 0;
                        break;
                    }
                case Windows.System.VirtualKey.J:
                case Windows.System.VirtualKey.L:
                    {
                        roll = 0;
                        break;
                    }
                case Windows.System.VirtualKey.G:
                    {
                        local_heading = true_north_heading;
                        Y_offset = current2Dpostion.y;
                        X_offset = current2Dpostion.x;

                        var res = await DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).StartTakeoffAsync();
                        System.Diagnostics.Debug.Write("takeoff\n");
                        break;
                    }
                case Windows.System.VirtualKey.H:
                    {
                        var res = await DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).StartAutoLandingAsync();
                        break;
                    }
                case Windows.System.VirtualKey.N:
                    {
                        var res = await DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).StopAutoLandingAsync();
                        break;
                    }
                case Windows.System.VirtualKey.X:
                    {
                        throttle = 0;
                        roll = 0;
                        pitch = 0;
                        yaw = 0;
                        mission_state = -1;
                        script_started = false;

                        try
                        {
                            if (DJISDKManager.Instance != null)
                                DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue(throttle, yaw, pitch, roll);
                        }
                        catch (Exception err)
                        {
                            System.Diagnostics.Debug.WriteLine(err);
                        }
                        break;
                    }
                case Windows.System.VirtualKey.Z:
                    {
                        if (script_started == false)
                        {
                            Message.Text = "Auto-Mission Starting";
                            script_started = true;
                            mission_state = 0;
                            SetTimer();
                            local_heading = true_north_heading;
                            hold_rotation = local_heading;
                            Y_offset = current2Dpostion.y;
                            X_offset = current2Dpostion.x;
                            SetWP();
                        }
                        break;
                    }
            }

            try
            {
                if (DJISDKManager.Instance != null)
                    DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue(throttle, yaw, pitch, roll);
            }
            catch (Exception err)
            {
                System.Diagnostics.Debug.WriteLine(err);
            }
        }

        private async void Grid_KeyDown(object sender, KeyRoutedEventArgs e)
        {
            switch (e.Key)
            {
                case Windows.System.VirtualKey.W:
                    {
                        throttle += 0.02f;
                        if (throttle > 0.5f)
                            throttle = 0.5f;
                        break;
                    }
                case Windows.System.VirtualKey.S:
                    {
                        throttle -= 0.02f;
                        if (throttle < -0.5f)
                            throttle = -0.5f;
                        break;
                    }
                case Windows.System.VirtualKey.A:
                    {
                        yaw -= 0.05f;
                        if (yaw > 0.5f)
                            yaw = 0.5f;
                        break;
                    }
                case Windows.System.VirtualKey.D:
                    {
                        yaw += 0.05f;
                        if (yaw < -0.5f)
                            yaw = -0.5f;
                        break;
                    }
                case Windows.System.VirtualKey.I:
                    {
                        pitch += 0.05f;
                        if (pitch > 0.5)
                            pitch = 0.5f;
                        break;
                    }
                case Windows.System.VirtualKey.K:
                    {
                        pitch -= 0.05f;
                        if (pitch < -0.5f)
                            pitch = -0.5f;
                        break;
                    }
                case Windows.System.VirtualKey.J:
                    {
                        roll -= 0.05f;
                        if (roll < -0.5f)
                            roll = -0.5f;
                        break;
                    }
                case Windows.System.VirtualKey.L:
                    {
                        roll += 0.05f;
                        if (roll > 0.5)
                            roll = 0.5f;
                        break;
                    }
                case Windows.System.VirtualKey.Number0:
                    {
                        GimbalAngleRotation rotation = new GimbalAngleRotation()
                        {
                            mode = GimbalAngleRotationMode.RELATIVE_ANGLE,
                            pitch = 90,
                            roll = 45,
                            yaw = 45,
                            pitchIgnored = false,
                            yawIgnored = false,
                            rollIgnored = false,
                            duration = 0.5
                        };

                        System.Diagnostics.Debug.Write("pitch = 45\n");

                        // Defined somewhere else
                        var gimbalHandler = DJISDKManager.Instance.ComponentManager.GetGimbalHandler(0, 0);

                        //angle
                        //var gimbalRotation = new GimbalAngleRotation();
                        //gimbalRotation.pitch = 90;
                        //gimbalRotation.pitchIgnored = false;
                        //gimbalRotation.duration = 1;
                        //await gimbalHandler.RotateByAngleAsync(gimbalRotation);

                        //Speed
                        //var gimbalRotation_speed = new GimbalSpeedRotation();
                        //gimbalRotation_speed.pitch = 10;
                        //await gimbalHandler.RotateBySpeedAsync(gimbalRotation_speed);

                        await gimbalHandler.RotateByAngleAsync(rotation);

                        break;
                    }
                case Windows.System.VirtualKey.P:
                    {
                        GimbalAngleRotation rotation = new GimbalAngleRotation()
                        {
                            mode = GimbalAngleRotationMode.RELATIVE_ANGLE,
                            pitch = 45,
                            roll = 45,
                            yaw = 45,
                            pitchIgnored = false,
                            yawIgnored = false,
                            rollIgnored = false,
                            duration = 0.5
                        };

                        System.Diagnostics.Debug.Write("pitch - 10\n");

                        // Defined somewhere else
                        var gimbalHandler = DJISDKManager.Instance.ComponentManager.GetGimbalHandler(0, 0);

                        //Speed
                        var gimbalRotation_speed = new GimbalSpeedRotation();
                        gimbalRotation_speed.pitch = -10;
                        await gimbalHandler.RotateBySpeedAsync(gimbalRotation_speed);

                        //await DJISDKManager.Instance.ComponentManager.GetGimbalHandler(0,0).RotateByAngleAsync(rotation);

                        break;
                    }
                case Windows.System.VirtualKey.O:
                    {
                        GimbalAngleRotation rotation = new GimbalAngleRotation()
                        {
                            mode = GimbalAngleRotationMode.RELATIVE_ANGLE,
                            pitch = 45,
                            roll = 45,
                            yaw = 45,
                            pitchIgnored = false,
                            yawIgnored = false,
                            rollIgnored = false,
                            duration = 0.5
                        };

                        System.Diagnostics.Debug.Write("pitch + 10\n");

                        // Defined somewhere else
                        var gimbalHandler = DJISDKManager.Instance.ComponentManager.GetGimbalHandler(0, 0);

                        //Speed
                        var gimbalRotation_speed = new GimbalSpeedRotation();
                        gimbalRotation_speed.pitch = 10;
                        await gimbalHandler.RotateBySpeedAsync(gimbalRotation_speed);

                        //await DJISDKManager.Instance.ComponentManager.GetGimbalHandler(0,0).RotateByAngleAsync(rotation);

                        break;
                    }
                case Windows.System.VirtualKey.Z:
                    {
                        //System.Diagnostics.Debug.WriteLine("Start Auto Mission\n");
                        //SetTimer();
                        /*
                        System.Diagnostics.Debug.WriteLine("Emergency Stop!\n");
                        throttle = 0;
                        yaw = 0;
                        pitch = 0;
                        roll = 0;
                        if (DJISDKManager.Instance != null)
                            DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue(throttle, yaw, pitch, roll);
                            */
                        break;
                    }
                case Windows.System.VirtualKey.X:
                    {
                        /*
                        yaw += 0.05f;
                        if (yaw < -0.5f)
                            yaw = -0.5f;

                        pitch -= 0.05f;
                        if (pitch < -0.5f)
                            pitch = -0.5f;
                            */
                        break;
                    }
            }

            try
            {
                if (DJISDKManager.Instance != null)
                    DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue(throttle, yaw, pitch, roll);
            }
            catch (Exception err)
            {
                System.Diagnostics.Debug.WriteLine(err);
            }
        }

        private void SetTimer()
        {
            // Create a timer with a two second interval.
            TimerAsync = new System.Timers.Timer(100);        //in ms
            // Hook up the Elapsed event for the timer. 
            TimerAsync.Elapsed += State_change;
            TimerAsync.AutoReset = true;
            TimerAsync.Enabled = true;
        }

        private void SetVideoProtectionTimer()
        {
            VideoProtectionTimerAsync = new System.Timers.Timer(15000);
            VideoProtectionTimerAsync.Elapsed += VideoProtection;
            VideoProtectionTimerAsync.AutoReset = false;
            VideoProtectionTimerAsync.Enabled = true;
        }
        
        private void VideoProtection(Object source, ElapsedEventArgs e)
        {
            video_enabled = false;
            SaveVideo(composition);
            //composition = new MediaComposition();
            System.Diagnostics.Debug.WriteLine("Video Recording Ended as 15s pasted");
            lastframe = DateTime.Now;
            thisframe = DateTime.Now;
            if (VideoProtectionTimerAsync != null)
            {
                VideoProtectionTimerAsync.Stop();
                VideoProtectionTimerAsync.Dispose();
            }
        }

        private void GetAltitudeChanged(object sender, DoubleMsg? value)
        {
            if (value.HasValue)
            {
                currentAltitude = value.Value.value;
            }
        }
        private void GetTakeoffLocationAltitudeChanged(object sender, DoubleMsg? value)
        {
            if (value.HasValue)
            {
                currentAltitudeNotBaro = value.Value.value;
            }
        }
        private void GetAircraftLocationChanged(object sender, LocationCoordinate2D? value)
        {
            if (value.HasValue)
            {
                latitude = value.Value.latitude;
                longitude = value.Value.longitude;
            }
        }
        private void GetAttitudeChanged(object sender, Attitude? value)
        {
            if (value.HasValue)
            {
                true_north_heading = value.Value.yaw;
            }
        }
        private void GetAlignedAircraftLocationChanged(object sender, DoublePoint3D? value)
        {
            if (value.HasValue)
            {
                //System.Diagnostics.Debug.WriteLine("DoublePoint3D updated");
                N_position = value.Value.x;
                E_position = value.Value.y;
                D_position = value.Value.z;
                current2Dpostion.x = value.Value.y - X_offset;
                current2Dpostion.y = value.Value.x - Y_offset;
            }
        }

        private static void Attitude_control(float throttle = 0, float roll = 0, float pitch = 0, float yaw = 0)
        {
            if (DJISDKManager.Instance != null)
                DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue(throttle, yaw, pitch, roll);
        }
        private static void Attitude_control()
        {
            throttle = 0;
            roll = 0;
            pitch = 0;
            yaw = 0;
        }
        private async void State_change(Object source, ElapsedEventArgs e)
        {
            /*
            startTime = DateTime.Now;
            endTime = DateTime.Now;
            Double elapsedMillisecs = ((TimeSpan)(endTime - startTime)).TotalMilliseconds;
            System.Diagnostics.Debug.WriteLine("Checking state, it has started {0}\n", elapsedMillisecs);
            */
            //script_state++;
            System.Diagnostics.Debug.WriteLine("Current State: {0}", mission_state);
            switch (mission_state)
            {
                case 0:
                    {
                        System.Diagnostics.Debug.WriteLine("Taking off...");
                        var res = await DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).StartTakeoffAsync();
                        mission_state=1;
                        break;
                    }
                case 1:     //take off
                    {
                        if (currentAltitude >= 1.1)
                        {
                            System.Diagnostics.Debug.WriteLine("Take off completed");
                            Attitude_control();
                            StartRecording();
                            mission_state++;
                            GimbalResetCommandMsg resetMsg = new GimbalResetCommandMsg() { value = GimbalResetCommand.UNKNOWN };
                            await DJISDKManager.Instance.ComponentManager.GetGimbalHandler(0, 0).ResetGimbalAsync(resetMsg);
                        }
                        else
                        {
                            System.Diagnostics.Debug.WriteLine("Climbing...");
                        }
                        break;
                    }
                case 2:     //go to WP1
                    {
                        if (myWP[1].compare(current2Dpostion) <= myWP[1].tolarence)
                        {
                            System.Diagnostics.Debug.WriteLine("WP1 arrived");
                            Attitude_control();
                            StopRecording();
                            System.Diagnostics.Debug.WriteLine("current position {0}, {1}", current2Dpostion.x, current2Dpostion.y);
                            mission_state++;
                        }
                        else
                        {
                            roll = max_speed;
                            pitch = pitchPID.get(myWP[1].offsetFromPath(myWP[0], current2Dpostion));
                            System.Diagnostics.Debug.WriteLine("current position {0}, {1}", current2Dpostion.x, current2Dpostion.y);
                            System.Diagnostics.Debug.WriteLine("Flying to WP1, distance = {0}", myWP[1].compare(current2Dpostion));
                            System.Diagnostics.Debug.WriteLine("Perpendicular distance = {0}", myWP[1].offsetFromPath(myWP[0], current2Dpostion));
                        }
                        break;
                    }
                case 3:     //down to 0.7
                    {
                        if (video_saved == true)
                        {
                            if (Math.Round(currentAltitude, 1) <= 0.7)
                            {
                                Attitude_control();
                                StartRecording();
                                mission_state++;
                            }
                            else
                            {
                                throttle = throttlePID.get(0.7 - currentAltitude);
                                System.Diagnostics.Debug.WriteLine("Going down to 0.7...throttle:{0}", throttle);
                            }
                        }
                        break;
                    }
                case 4:     //go to WP2
                    {
                        if (myWP[2].compare(current2Dpostion) <= myWP[2].tolarence)
                        {
                            System.Diagnostics.Debug.WriteLine("WP2 arrived");
                            Attitude_control();
                            StopRecording();
                            System.Diagnostics.Debug.WriteLine("current position {0}, {1}", current2Dpostion.x, current2Dpostion.y);
                            //mission_state=5;
                            mission_state++;
                        }
                        else
                        {
                            roll = -max_speed;
                            pitch = -pitchPID.get(myWP[2].offsetFromPath(myWP[1], current2Dpostion));
                            System.Diagnostics.Debug.WriteLine("current position {0}, {1}", current2Dpostion.x, current2Dpostion.y);
                            System.Diagnostics.Debug.WriteLine("Flying to WP2, distance = {0}", myWP[2].compare(current2Dpostion));
                            System.Diagnostics.Debug.WriteLine("Perpendicular distance = {0}", myWP[2].offsetFromPath(myWP[1], current2Dpostion));
                        }
                        break;
                    }
                case 5:     //down to 0.2
                    {
                        if (video_saved == true)
                        {
                            if (currentAltitude <= 0.2)
                            {
                                Attitude_control();
                                StartRecording();
                                mission_state++;
                            }
                            else
                            {
                                throttle = throttlePID.get(0.2 - currentAltitude);
                                System.Diagnostics.Debug.WriteLine("Going down to 0.2...throttle:{0}", throttle);
                            }
                        }
                        break;
                    }
                case 6:     //go to WP3
                    {
                        if (myWP[3].compare(current2Dpostion) <= myWP[3].tolarence)
                        {
                            System.Diagnostics.Debug.WriteLine("WP3 arrived");
                            Attitude_control();
                            StopRecording();
                            System.Diagnostics.Debug.WriteLine("current position {0}, {1}", current2Dpostion.x, current2Dpostion.y);
                            //mission_state++;
                            mission_state=13;
                        }
                        else
                        {
                            roll = max_speed;
                            pitch = pitchPID.get(myWP[3].offsetFromPath(myWP[2], current2Dpostion));
                            System.Diagnostics.Debug.WriteLine("current position {0}, {1}", current2Dpostion.x, current2Dpostion.y);
                            System.Diagnostics.Debug.WriteLine("Flying to WP1, distance = {0}", myWP[3].compare(current2Dpostion));
                            System.Diagnostics.Debug.WriteLine("Perpendicular distance = {0}", myWP[3].offsetFromPath(myWP[2], current2Dpostion));
                        }
                        break;
                    }
                case 7:     //rotate 180degree
                    {
                        System.Diagnostics.Debug.WriteLine("change hold rotation from {0} to {1}", hold_rotation, CompassBoundary(hold_rotation + 180.0));
                        if (first_time == true)
                        {
                            hold_rotation = CompassBoundary(hold_rotation + 180.0);
                            first_time = false;
                        }

                        angle_remain = -CompassBoundary(hold_rotation - true_north_heading);
                        System.Diagnostics.Debug.WriteLine("{0} degree left", angle_remain);
                        if ((angle_remain <= angle_tolarence) && (angle_remain >= -angle_tolarence))
                        {
                            System.Diagnostics.Debug.WriteLine("rotation completed");
                            if (video_saved == true)
                            {
                                StartRecording();
                                mission_state++;
                            }
                        }
                        break;
                    }
                case 8:     //go to WP4
                    {
                        if (myWP[4].compare(current2Dpostion) <= myWP[4].tolarence)
                        {
                            System.Diagnostics.Debug.WriteLine("WP4 arrived");
                            Attitude_control();
                            StopRecording();
                            System.Diagnostics.Debug.WriteLine("current position {0}, {1}", current2Dpostion.x, current2Dpostion.y);
                            mission_state++;
                        }
                        else
                        {
                            roll = max_speed;
                            pitch = pitchPID.get(myWP[4].offsetFromPath(myWP[3], current2Dpostion));
                            System.Diagnostics.Debug.WriteLine("current position {0}, {1}", current2Dpostion.x, current2Dpostion.y);
                            System.Diagnostics.Debug.WriteLine("Flying to WP1, distance = {0}", myWP[4].compare(current2Dpostion));
                            System.Diagnostics.Debug.WriteLine("Perpendicular distance = {0}", myWP[4].offsetFromPath(myWP[3], current2Dpostion));
                        }
                        break;
                    }
                case 9:     //rise to 0.7
                    {
                        if (video_saved == true)
                        {
                            if (Math.Round(currentAltitude, 1) >= 0.7)
                            {
                                Attitude_control();
                                StartRecording();
                                mission_state++;
                            }
                            else
                            {
                                throttle = throttlePID.get(0.7 - currentAltitude);
                                System.Diagnostics.Debug.WriteLine("Going Up to 0.7...throttle:{0}", throttle);
                            }
                        }
                        break;
                    }
                case 10:    //go to WP5
                    {
                        if (myWP[5].compare(current2Dpostion) <= myWP[5].tolarence)
                        {
                            System.Diagnostics.Debug.WriteLine("WP5 arrived");
                            Attitude_control();
                            StopRecording();
                            System.Diagnostics.Debug.WriteLine("current position {0}, {1}", current2Dpostion.x, current2Dpostion.y);
                            mission_state++;
                        }
                        else
                        {
                            roll = -max_speed;
                            pitch = -pitchPID.get(myWP[5].offsetFromPath(myWP[4], current2Dpostion));
                            System.Diagnostics.Debug.WriteLine("current position {0}, {1}", current2Dpostion.x, current2Dpostion.y);
                            System.Diagnostics.Debug.WriteLine("Flying to WP2, distance = {0}", myWP[5].compare(current2Dpostion));
                            System.Diagnostics.Debug.WriteLine("Perpendicular distance = {0}", myWP[5].offsetFromPath(myWP[4], current2Dpostion));
                        }
                        break;
                    }
                case 11:    //rise to 1.2
                    {
                        if (video_saved == true)
                        {
                            if (currentAltitude >= 1.2)
                            {
                                Attitude_control();
                                StartRecording();
                                mission_state++;
                            }
                            else
                            {
                                throttle = throttlePID.get(1.2 - currentAltitude);
                                System.Diagnostics.Debug.WriteLine("Going up to 1.2...throttle:{0}", throttle);
                            }
                        }
                        break;
                    }
                case 12:    //go to WP6
                    {
                        if (myWP[6].compare(current2Dpostion) <= myWP[6].tolarence)
                        {
                            System.Diagnostics.Debug.WriteLine("WP4 arrived");
                            Attitude_control();
                            StopRecording();
                            System.Diagnostics.Debug.WriteLine("current position {0}, {1}", current2Dpostion.x, current2Dpostion.y);
                            mission_state++;
                        }
                        else
                        {
                            roll = max_speed;
                            pitch = pitchPID.get(myWP[6].offsetFromPath(myWP[5], current2Dpostion));
                            System.Diagnostics.Debug.WriteLine("current position {0}, {1}", current2Dpostion.x, current2Dpostion.y);
                            System.Diagnostics.Debug.WriteLine("Flying to WP1, distance = {0}", myWP[6].compare(current2Dpostion));
                            System.Diagnostics.Debug.WriteLine("Perpendicular distance = {0}", myWP[6].offsetFromPath(myWP[5], current2Dpostion));
                        }
                        break;
                    }
                case 13:    //wait for saving
                    {
                        if (video_saved == true)
                            mission_state++;
                        break;
                    }
                case 14:
                    {
                        var res = await DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).StartAutoLandingAsync();
                        System.Diagnostics.Debug.WriteLine("Mission Completed. Press H to land. It will auto land anyway");
                        System.Diagnostics.Debug.WriteLine("Result: {0}", Textbox.Text);
                        break;
                    }
                default:
                    {
                        //StopRecording();
                        System.Diagnostics.Debug.WriteLine("Mission Inpterrupted");
                        break;
                    }
            }
            //periodically adjust yaw angle
            if (!fast_yaw)
                yaw = yawPID.get(hold_rotation - true_north_heading);
            else
                yaw = fastyawPID.get(hold_rotation - true_north_heading);
            //System.Diagnostics.Debug.WriteLine("yaw = {0}", yaw);
            if (DJISDKManager.Instance != null)
                DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue(throttle, yaw, pitch, roll);

        }
    }
}