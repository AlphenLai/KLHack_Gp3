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
                await DJISDKManager.Instance.ComponentManager.GetFlightAssistantHandler(0, 0).StartAlignedPushDataAsync();

                await Task.Delay(5000);
                GimbalResetCommandMsg resetMsg = new GimbalResetCommandMsg() { value = GimbalResetCommand.UNKNOWN };

                await DJISDKManager.Instance.ComponentManager.GetGimbalHandler(0, 0).ResetGimbalAsync(resetMsg);
            };
            DJISDKManager.Instance.RegisterApp("3ad4ddfffc9f656725e12c34");

            //subscribing event handlers
            //DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).TakeoffLocationAltitudeChanged += getTakeoffLocationAltitudeChanged;
            DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).AttitudeChanged += getAttitudeChanged;
            DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).AltitudeChanged += getAltitudeChanged;
            DJISDKManager.Instance.ComponentManager.GetFlightAssistantHandler(0, 0).AlignedAircraftLocationChanged += getAlignedAircraftLocationChanged;
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
                    //System.Diagnostics.Debug.WriteLine("currentAltitude:{0}", currentAltitude);
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
                //SaveImage(VideoSource);
            });
        }

        private async void Stop_Button_Click(object sender, RoutedEventArgs e)
        {
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
            

            //Process process = new Process();
            //process.StartInfo.FileName = command;
            //process.Start();

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

        private void Set_Button_Click(object sender, RoutedEventArgs e)
        {
            myWP[1] = rotationMatrix(new position(Convert.ToDouble(WP1x.Text), Convert.ToDouble(WP1y.Text)), local_heading);
            myWP[2] = rotationMatrix(new position(Convert.ToDouble(WP2x.Text), Convert.ToDouble(WP2y.Text)), local_heading);
            myWP[3] = rotationMatrix(new position(Convert.ToDouble(WP3x.Text), Convert.ToDouble(WP3y.Text)), local_heading);
            myWP[4] = rotationMatrix(new position(Convert.ToDouble(WP4x.Text), Convert.ToDouble(WP4y.Text)), local_heading);
            Message.Text = "Waypoints updated";
        }

        private static float throttle = 0;
        private static float roll = 0;
        private static float pitch = 0;
        private static float yaw = 0;

        //custom var
        private static System.Timers.Timer TimerAsync;
        bool script_started = false;
        static int mission_state = 0;
        static DateTime startTime;
        static DateTime endTime;
        static DateTime takeoffstarttime;
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
        static double WP_tolarence = 0.15f;
        static double angle_tolarence = 0.2f;
        static float max_yaw_speed = 1.0f;
        static float max_speed = 0.5f;
        static double angle_remain;
        static double hold_rotation = 0;
        static int image_count = 1;

        struct position
        {
            public double x;
            public double y;
            public double rotation;
            public double tolarence;

            public position(double x0, double y0, double rotation0 = 0, double tolarence0 = 1) : this()
            {
                x = x0;
                y = y0;
                rotation = rotation0;
                tolarence = tolarence0;
            }

            public double compare(position target)
            {
                return Math.Sqrt((x - target.x) * (x - target.x) + (y - target.y) * (y - target.y));
            }

            public void updateYaw(position currentPos)
            {
                int sign = 1;
                if (x - currentPos.x >= 0)          //to rightward
                    sign = 1;
                else if (x - currentPos.x < 0)      //to left
                    sign = -1;
                if (y - currentPos.y < 0)           //to backward
                {
                    rotation = 90.0 + sign * compassBoundary(Math.Round(toDegree(Math.Atan((x - currentPos.x) / (y - currentPos.y))), 1));
                }
                else                            //to forward
                {
                    rotation = sign * compassBoundary(Math.Round(toDegree(Math.Atan((x - currentPos.x) / (y - currentPos.y))), 1));
                }
            }

            public double offsetFromPath(position lastWP, position currentPos)
            {
                //return the distance between current position and the line alone last and next waypoint
                //-ve = LHS, +ve = RHS
                return ((y - lastWP.y) * currentPos.x - (x - lastWP.x) * currentPos.y + x * lastWP.y - y * lastWP.x) / Math.Sqrt((y - lastWP.y) * (y - lastWP.y) + (x - lastWP.x) * (x - lastWP.x));
            }
        }
        static position current2Dpostion;
        static position[] myWP = new position[10];

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

                float output = Convert.ToSingle(Kp * error + (1 / Ki) * integral + Kd * derivative);
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

        static PID rollPID = new PID(1.5, 1, 1, max_speed);
        static PID pitchPID = new PID(1.5, 1, 1, max_speed);
        static PID yawPID = new PID(0.05, 1, 0.3, max_yaw_speed);

        public static double toRadian(double angle)
        {
            return (Math.PI / 180) * angle;
        }

        public static double toDegree(double rad)
        {
            return rad * (180.0 / Math.PI);
        }

        static position rotationMatrix(position originPlane, double degree)
        {
            double x0 = Math.Cos(toRadian(degree)) * originPlane.x + -Math.Sin(toRadian(degree)) * originPlane.y;
            double y0 = Math.Sin(toRadian(degree)) * originPlane.x + Math.Cos(toRadian(degree)) * originPlane.y;
            position newPlane = new position(x0, y0);
            return newPlane;
        }

        private void setWP()
        {
            myWP[0].x = 0.0f;
            myWP[0].y = 0.0f;
            myWP[0].rotation = compassBoundary(0.0f + local_heading);
            myWP[0].tolarence = WP_tolarence;

            myWP[1].x = myWP[0].x - 1.0f * Math.Cos(toRadian(180.0f +local_heading));
            myWP[1].y = myWP[0].y + 1.0f * Math.Sin(toRadian(180.0f +local_heading));
            myWP[1].rotation = compassBoundary(0.0f + local_heading);
            myWP[1].tolarence = WP_tolarence;

            myWP[2].x = myWP[1].x + 1.0f * Math.Sin(toRadian(180 + local_heading));
            myWP[2].y = myWP[1].y + 1.0f * Math.Cos(toRadian(180 + local_heading));
            myWP[2].rotation = compassBoundary(180.0f + local_heading);
            myWP[2].tolarence = WP_tolarence;

            myWP[3].x = myWP[2].x + 1.0f * Math.Cos(toRadian(180 + local_heading));
            myWP[3].y = myWP[2].y - 1.0f * Math.Sin(toRadian(180 + local_heading));
            myWP[3].rotation = compassBoundary(180.0f + local_heading);
            myWP[3].tolarence = WP_tolarence;

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
            byte[] dummy = await EncodeJpeg(bmp);
            await FileIO.WriteBytesAsync(sampleFile, dummy);
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

                        setWP();
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
                case Windows.System.VirtualKey.Z:
                    {
                        /*
                        if (TimerAsync != null)
                        {N_offset
                            TimerAsync.Stop();
                            TimerAsync.Dispose();
                        }
                        */

                        if (script_started == false)
                        {
                            Message.Text = "Auto-Mission Starting";
                            script_started = true;
                            mission_state = 0;
                            //startTime = DateTime.Now;
                            SetTimer();
                            local_heading = true_north_heading;
                            hold_rotation = local_heading;
                            Y_offset = current2Dpostion.y;
                            X_offset = current2Dpostion.x;
                            setWP();
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

        private async void StartRecording(object sender, RoutedEventArgs e)
        {
            try
            {
                var error = await DJISDKManager.Instance.ComponentManager.GetCameraHandler(0, 0).StartRecordAsync();

                if (error != SDKError.NO_ERROR)
                {
                    System.Diagnostics.Debug.WriteLine(error); ;
                }
            }
            catch (Exception err)
            {
                System.Diagnostics.Debug.WriteLine(err);
            }
        }

        private async void StopRecording(object sender, RoutedEventArgs e)
        {
            try
            {
                var error = await DJISDKManager.Instance.ComponentManager.GetCameraHandler(0, 0).StopRecordAsync();
                if (error != SDKError.NO_ERROR)
                {
                    System.Diagnostics.Debug.WriteLine(error);
                }
            }
            catch (Exception err)
            {
                System.Diagnostics.Debug.WriteLine(err);
            }
        }

        private static void SetTimer()
        {
            // Create a timer with a two second interval.
            TimerAsync = new System.Timers.Timer(100);        //in ms
            // Hook up the Elapsed event for the timer. 
            TimerAsync.Elapsed += state_change;
            TimerAsync.AutoReset = true;
            TimerAsync.Enabled = true;
        }

        private void getAltitudeChanged(object sender, DoubleMsg? value)
        {
            if (value.HasValue)
            {
                currentAltitude = value.Value.value;
            }
        }
        private void getTakeoffLocationAltitudeChanged(object sender, DoubleMsg? value)
        {
            if (value.HasValue)
            {
                currentAltitudeNotBaro = value.Value.value;
            }
        }
        private void getAircraftLocationChanged(object sender, LocationCoordinate2D? value)
        {
            if (value.HasValue)
            {
                latitude = value.Value.latitude;
                longitude = value.Value.longitude;
            }
        }
        private void getAttitudeChanged(object sender, Attitude? value)
        {
            if (value.HasValue)
            {
                true_north_heading = value.Value.yaw;
            }
        }
        private void getAlignedAircraftLocationChanged(object sender, DoublePoint3D? value)
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

        private static void attitude_control(float throttle = 0, float roll = 0, float pitch = 0, float yaw = 0)
        {
            if (DJISDKManager.Instance != null)
                DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue(throttle, yaw, pitch, roll);
        }

        private static double compassBoundary(double compassValue)
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

        private static async void state_change(Object source, ElapsedEventArgs e)
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
                case 0:     //take off
                    {
                        System.Diagnostics.Debug.WriteLine("Taking off...");
                        takeoffstarttime = DateTime.Now;
                        var res = await DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).StartTakeoffAsync();
                        mission_state++;
                        break;
                    }
                case 1:     //roll
                    {
                        if (currentAltitude >= 1.1)
                        {
                            System.Diagnostics.Debug.WriteLine("Take off completed");
                            System.Diagnostics.Debug.WriteLine("Rolling...");
                            roll = 0.5f;
                            mission_state++;
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
                            roll = 0;
                            System.Diagnostics.Debug.WriteLine("current position {0}, {1}", current2Dpostion.x, current2Dpostion.y);

                            System.Diagnostics.Debug.WriteLine("Rotating...");
                            hold_rotation = myWP[2].rotation;
                            mission_state = 6;                            
                        }
                        else
                        {
                            pitch = pitchPID.get(myWP[1].offsetFromPath(myWP[0], current2Dpostion));
                            System.Diagnostics.Debug.WriteLine("Flying to WP1, distance = {0}", myWP[1].compare(current2Dpostion));
                            System.Diagnostics.Debug.WriteLine("Perpendicular distance = {0}", myWP[1].offsetFromPath(myWP[0], current2Dpostion));
                        }
                        break;
                    }
                case 3:     //rotate to new rotation
                    {
                        
                        angle_remain = -compassBoundary(true_north_heading - hold_rotation);
                        if ((angle_remain <= angle_tolarence) && (angle_remain >= -angle_tolarence)) 
                        {
                            System.Diagnostics.Debug.WriteLine("Rotation completed");
                            System.Diagnostics.Debug.WriteLine("Pitching...");
                            pitch = 0.5f;
                            mission_state++;
                        }
                        else
                        {
                            System.Diagnostics.Debug.WriteLine("Rotating...{0}degree left", angle_remain);
                        }
                        System.Diagnostics.Debug.WriteLine("YAW ={0}", yaw);
                        break;
                    }
                case 4:     //go to WP2
                    {
                        if (myWP[2].compare(current2Dpostion) <= myWP[2].tolarence)
                        {
                            System.Diagnostics.Debug.WriteLine("WP2 arrived");
                            pitch = 0;
                            System.Diagnostics.Debug.WriteLine("Rolling...");
                            roll = 0.5f;
                            System.Diagnostics.Debug.WriteLine("current position {0}, {1}", current2Dpostion.x, current2Dpostion.y);
                            mission_state++;
                        }
                        else
                        {
                            roll = rollPID.get(myWP[2].offsetFromPath(myWP[1], current2Dpostion));
                            System.Diagnostics.Debug.WriteLine("Flying to WP2, distance = {0}", myWP[2].compare(current2Dpostion));
                            System.Diagnostics.Debug.WriteLine("Perpendicular distance = {0}", myWP[2].offsetFromPath(myWP[1], current2Dpostion));
                            System.Diagnostics.Debug.WriteLine("current postion {0}, {1}", current2Dpostion.x, current2Dpostion.y);
                            System.Diagnostics.Debug.WriteLine("target postion {0}, {1}", myWP[2].x, myWP[2].y);
                        }
                        break;
                    }
                case 5:     //go to WP3
                    {
                        if (myWP[3].compare(current2Dpostion) <= myWP[3].tolarence)
                        {
                            System.Diagnostics.Debug.WriteLine("WP3 arrived");
                            roll = 0;
                            pitch = 0;
                            mission_state++;
                        }
                        else
                        {
                            pitch = pitchPID.get(myWP[3].offsetFromPath(myWP[2], current2Dpostion));
                            System.Diagnostics.Debug.WriteLine("Flying to WP3, distance = {0}", myWP[3].compare(current2Dpostion));
                            System.Diagnostics.Debug.WriteLine("Perpendicular distance = {0}", myWP[3].offsetFromPath(myWP[2], current2Dpostion));
                        }
                        break;
                    }
                case 6:
                    {
                        System.Diagnostics.Debug.WriteLine("Mission Completed. Press H to land");
                        break;
                    }
                default:
                    {
                        System.Diagnostics.Debug.WriteLine("Mission Inpterrupted");
                        break;
                    }
            }
            //periodically adjust yaw angle
            yaw = yawPID.get(hold_rotation - true_north_heading);

            if (DJISDKManager.Instance != null)
                DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue(throttle, yaw, pitch, roll);

        }
    }
}