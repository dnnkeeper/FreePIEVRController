﻿using FreePIE.Core.Contracts;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Windows.Devices.Bluetooth.GenericAttributeProfile;
using Windows.Devices.Enumeration;
using Windows.Foundation;
using System.Runtime.InteropServices.WindowsRuntime;
using Windows.Storage.Streams;
using Windows.Devices.Bluetooth;

namespace FreePIEVRController
{
    class Constants
    {
        public static readonly Guid DAYDREAM_SERVICE_UUID = new Guid("0000fe55-0000-1000-8000-00805f9b34fb");
        public static readonly Guid DAYDREAM_CHARACTERISTICS_UUID = new Guid("00000001-1000-1000-8000-00805f9b34fb");
        public static readonly Guid GEARVR_SERVICE_UUID = new Guid("4f63756c-7573-2054-6872-65656d6f7465");
        public static readonly Guid GEARVR_DATA_CHARACTERISTICS_UUID = new Guid("c8c51726-81bc-483b-a052-f7a14ea3d281");
        public static readonly Guid GEARVR_COMMAND_CHARACTERISTICS_UUID = new Guid("c8c51726-81bc-483b-a052-f7a14ea3d282");
        // GearVR Controller reports 3 samples per single packet. Packet rate are 60Hz.
        public static readonly float GEARVR_HZ = 180f;
        public static readonly short GEARVR_COMMAND_SENSOR = 0x0100;
        public static readonly short GEARVR_COMMAND_VR_MODE = 0x0800;
        public static readonly int TYPE_DAYDREAM = 1;
        public static readonly int TYPE_GEARVR = 2;
    }

    class GlobalIndexer<T, TIndex>
    {
        private readonly Func<TIndex, T> initilizer;
        private readonly Dictionary<TIndex, T> globals;

        public GlobalIndexer(Func<TIndex, T> initilizer)
        {
            this.initilizer = initilizer;
            globals = new Dictionary<TIndex, T>();
        }

        public T this[TIndex index]
        {
            get
            {
                if (!globals.ContainsKey(index))
                {
                    globals[index] = initilizer(index);
                }

                return globals[index];
            }
        }
    }

    [GlobalType(Type = typeof(VRControllerGlobal), IsIndexed = true)]
    public class VRControllerFreePIEPlugin : IPlugin
    {
        private List<VRControllerGlobal> globals;
        private DeviceManager manager = new DeviceManager();

        public object CreateGlobal()
        {
            globals = new List<VRControllerGlobal>();
            return new GlobalIndexer<VRControllerGlobal, int>(CreateGlobal);
        }

        public Action Start()
        {
            return null;
        }

        public void Stop()
        {
            foreach (var global in globals)
            {
                global.Close();
            }
            globals.Clear();
        }

        public event EventHandler Started;

        public string FriendlyName
        {
            get { return "VR Controller"; }
        }

        public bool GetProperty(int index, IPluginProperty property)
        {
            if (index == 0)
            {
                property.Name = "SomeProperty";
                property.Caption = "VR Controller is installed";
                property.HelpText = "VR Controller is installed";

                property.Choices.Add("VR Controller is installed", 1);

                property.DefaultValue = 1;
                return true;
            }
            return false;
        }

        public bool SetProperties(Dictionary<string, object> properties)
        {
            return true;
        }

        private VRControllerGlobal CreateGlobal(int index)
        {
            var device = manager.GetNext();
            if (device == null)
            {
                return null;
            }
            var global = new VRControllerGlobal(index, device);
            globals.Add(global);

            return global;
        }

        public void DoBeforeNextExecute()
        {
            foreach (var global in globals)
            {
                global.Update();
            }
        }
    }

    [Global(Name = "vrcontroller")]
    public class VRControllerGlobal
    {
        DeviceInfo currentDevice;

        internal VRControllerGlobal(int index, DeviceInfo device)
        {
            currentDevice = device;
            Connect(device).Wait();
        }

        public void Close()
        {
            if (service != null)
            {
                service.Dispose();
                service = null;
            }
        }


        private GattDeviceService service;
        private GattCharacteristic daydreamCharacteristic;
        private GattCharacteristic gearvrDataCharacteristic;
        private GattCharacteristic gearvrCommandCharacteristic;

        private AHRS.MadgwickAHRS AHRS;

        public void Update()
        {
        }

        static int ParseInt(int start, int end, bool signed, byte[] buf)
        {
            int ret = 0;

            for (int i = start; i < end; i++)
            {
                ret |= ((buf[i / 8] >> (7 - (i % 8))) & 1) << (end - i - 1);
            }
            if (signed)
            {
                int shift = 32 - (end - start);
                // Copy sign bit
                ret = (ret << shift) >> shift;
            }
            return ret;
        }

        static int Get32(int start, byte[] buf)
        {
            return buf[start] | (buf[start + 1] << 8) | (buf[start + 2] << 16) | (buf[start + 3] << 24);
        }

        static int Get16(int start, byte[] buf)
        {
            int ret = buf[start + 0] | (buf[start + 1] << 8);
            return (ret << 16) >> 16;
        }

        static string BDAddrToString(ulong bdaddr)
        {
            string addr = "";
            for (int i = 0; i < 6; i++)
            {
                addr += ((bdaddr >> ((5 - i) * 8)) & 0xFF).ToString("X2");
                if (i != 5)
                {
                    addr += ":";
                }
            }
            return addr;
        }

        void CallbackDaydream(GattCharacteristic sender, GattValueChangedEventArgs eventArgs)
        {
            // https://stackoverflow.com/questions/40730809/use-daydream-controller-on-hololens-or-outside-daydream/40753551#40753551
            // https://github.com/mrdoob/daydream-controller.js/blob/master/DaydreamController.js
            var buf = eventArgs.CharacteristicValue.ToArray();

            // Orientation
            double scale = 2.0 * Math.PI / 4095.0;
            orientation[0] = ParseInt(14, 27, true, buf) * scale;
            orientation[1] = ParseInt(27, 40, true, buf) * scale;
            orientation[2] = ParseInt(40, 53, true, buf) * scale;

            double abs = Math.Sqrt(orientation[0] * orientation[0]
                + orientation[1] * orientation[1] + orientation[2] * orientation[2]);
            double sin = Math.Sin(abs / 2);
            quaternion[0] = orientation[0] / abs * sin;
            quaternion[1] = orientation[1] / abs * sin;
            quaternion[2] = orientation[2] / abs * sin;
            quaternion[3] = Math.Cos(abs / 2);

            position = ArmModel.CalculateModel(quaternion);

            // Acceleration
            double ascale = 8.0 * 9.8 / 4095.0;
            acceleration[0] = ParseInt(53, 66, true, buf) * ascale;
            acceleration[1] = ParseInt(66, 79, true, buf) * ascale;
            acceleration[2] = ParseInt(79, 92, true, buf) * ascale;

            // Gyro
            double gscale = 2048.0 / 180 * Math.PI / 4095.0;
            gyroscope[0] = ParseInt(92, 105, true, buf) * gscale;
            gyroscope[1] = ParseInt(105, 118, true, buf) * gscale;
            gyroscope[2] = ParseInt(118, 131, true, buf) * gscale;

            // Touch position
            double tscale = 1 / 255.0;
            int tx = ParseInt(131, 139, false, buf);
            int ty = ParseInt(139, 147, false, buf);

            // from -1.0(left) to +1.0(right)
            trackpad[0] = tx * tscale * 2.0 - 1.0;
            // from -1.0(bottom) to +1.0(top)
            trackpad[1] = 1.0 - ty * tscale * 2.0;
            button[TOUCH] = tx != 0 || ty != 0;

            // Vol up
            button[VOLUP] = ParseInt(147, 148, false, buf) != 0;
            // Vol down
            button[VOLDOWN] = ParseInt(148, 149, false, buf) != 0;
            // App
            button[APP] = ParseInt(149, 150, false, buf) != 0;
            // Home
            button[HOME] = ParseInt(150, 151, false, buf) != 0;
            // Click
            button[CLICK] = ParseInt(151, 152, false, buf) != 0;

            // Version
            version = ParseInt(152, 160, false, buf);
        }

        void CallbackGearVR(GattCharacteristic sender, GattValueChangedEventArgs eventArgs)
        {
            var buf = eventArgs.CharacteristicValue.ToArray();
            if (buf.Length < 3)
            {
                return;
            }

            // The data array consist of multiple data which size is 16bytes and footer 12bytes
            // On current version, the data array is 60 bytes and dataCount = 3
            const int DATA_SIZE = 16;
            const int FOOTER_SIZE = 6 + 3 + 1 + 1 + 1; // 12 bytes = 6(mag) + 3(touch) + 1(temperature) + 1(button) + 1(unknown)
            int dataCount = (buf.Length - FOOTER_SIZE) / DATA_SIZE;

            var mscale = 0.06 * 0.06;
            magnetometer[0] = Get16(DATA_SIZE * dataCount + 0, buf) * mscale; // 48 ~ 54
            magnetometer[1] = Get16(DATA_SIZE * dataCount + 2, buf) * mscale;
            magnetometer[2] = Get16(DATA_SIZE * dataCount + 4, buf) * mscale;
            var mmag = Math.Sqrt(magnetometer[0] * magnetometer[0]
                + magnetometer[1] * magnetometer[1] + magnetometer[2] * magnetometer[2]);

            for (int i = 0; i < dataCount; i++)
            {
                var time2 = Get32(DATA_SIZE * i, buf);

                double ascale = 9.80665 / 2048.0;
                var ax = Get16(DATA_SIZE * i + 4, buf) * ascale;
                var ay = Get16(DATA_SIZE * i + 6, buf) * ascale;
                var az = Get16(DATA_SIZE * i + 8, buf) * ascale;
                var amag = Math.Sqrt(ax * ax + ay * ay + az * az);

                acceleration[0] = ax;
                acceleration[1] = ay;
                acceleration[2] = az;

                double gscale = 0.017453292 / 14.285;
                var gx = Get16(DATA_SIZE * i + 10, buf) * gscale;
                var gy = Get16(DATA_SIZE * i + 12, buf) * gscale;
                var gz = Get16(DATA_SIZE * i + 14, buf) * gscale;
                var gmag = Math.Sqrt(gx * gx + gy * gy + gz * gz);

                gyroscope[0] = gx;
                gyroscope[1] = gy;
                gyroscope[2] = gz;

                if (useMagnetometer)
                    AHRS.Update((float)gx, (float)gy, (float)gz, (float)ax, (float)ay, (float)az, (float)magnetometer[0], (float)magnetometer[1], (float)magnetometer[2]);
                else
                    AHRS.Update((float)gx, (float)gy, (float)gz, (float)ax, (float)ay, (float)az);
            }

            // We need to fix the difference of coordinate system between AHRS and VR app.
            // I don't know how to introduce correct conversion, but it seems roughly right (working).
            quaternion[0] = AHRS.Quaternion[1];
            quaternion[1] = AHRS.Quaternion[3];
            quaternion[2] = -AHRS.Quaternion[2];
            quaternion[3] = AHRS.Quaternion[0];

            position = ArmModel.CalculateModel(quaternion);

            temperature = buf[DATA_SIZE * dataCount + 6 + 3]; // 57

            int touchPos = DATA_SIZE * dataCount + 6; // 54
            var touchFlag = ParseInt(touchPos * 8, touchPos * 8 + 4, false, buf);
            var tx = ParseInt(touchPos * 8 + 4, touchPos * 8 + 14, false, buf);
            var ty = ParseInt(touchPos * 8 + 14, touchPos * 8 + 24, false, buf);

            double tscale = 1.0 / 320.0;
            // from -1.0(left) to +1.0(right)
            trackpad[0] = tx * tscale * 2.0 - 1.0;
            // from -1.0(bottom) to +1.0(top)
            trackpad[1] = 1.0 - ty * tscale * 2.0;
            button[TOUCH] = touchFlag == 1;

            // trigger, home, back, touch click, vol up, vol down
            int buttonPos = DATA_SIZE * dataCount + 6 + 3 + 1; // 58;
            button[TRIGGER] = (buf[buttonPos] & (1 << 0)) != 0;
            button[HOME] = (buf[buttonPos] & (1 << 1)) != 0;
            button[APP] = (buf[buttonPos] & (1 << 2)) != 0;
            button[CLICK] = (buf[buttonPos] & (1 << 3)) != 0;
            button[VOLUP] = (buf[buttonPos] & (1 << 4)) != 0;
            button[VOLDOWN] = (buf[buttonPos] & (1 << 5)) != 0;

            version = buf[DATA_SIZE * dataCount + 6 + 3 + 1 + 1]; // 59
        }

        async Task Connect(DeviceInfo device)
        {
            bool connected = false;
            if (device.type == Constants.TYPE_DAYDREAM)
            {
                connected = await ConnectDaydream(device.info);
            }
            else if (device.type == Constants.TYPE_GEARVR)
            {
                connected = await ConnectGearVR(device.info);
            }
            if (!connected)
            {
                if (service != null)
                {
                    service.Dispose();
                    service = null;
                }
            }
        }

        async Task<bool> ConnectDaydream(DeviceInformation info)
        {
            try
            {
                Console.WriteLine(string.Format("Trying to connect {0} {1}", info.Name, info.Id));
                service = await AsTask(GattDeviceService.FromIdAsync(info.Id));
                if (service == null)
                {
                    Console.WriteLine("Error: Another program is using the device.");
                    return false;
                }

                var list = service.GetCharacteristics(Constants.DAYDREAM_CHARACTERISTICS_UUID);
                if (list.Count == 0)
                {
                    Console.WriteLine("Error: No characteristics. uuid=DAYDREAM_CHARACTERISTICS_UUID.");
                    return false;
                }

                daydreamCharacteristic = list[0];
                daydreamCharacteristic.ValueChanged += CallbackDaydream;
                var status = await AsTask(daydreamCharacteristic.WriteClientCharacteristicConfigurationDescriptorAsync(GattClientCharacteristicConfigurationDescriptorValue.Notify));
                //Console.WriteLine("Write: {0}", status.ToString());
                if (status == GattCommunicationStatus.Success)
                {
                    Console.WriteLine("Connected.");
                    type = Constants.TYPE_DAYDREAM;
                    bdaddr = BDAddrToString(service.Device.BluetoothAddress);
                    deviceId = service.Device.DeviceId;
                    deviceName = info.Name;
                    return true;
                }
                else
                {
                    Console.WriteLine("Error: Cannot connect. (Timeout)");
                    return false;
                }
            }
            catch (Exception e)
            {
                Console.WriteLine("Error: Caught exception. " + e.Message);
            }

            return false;
        }

        async Task<bool> SetNotify()
        {
            var status = await AsTask(gearvrDataCharacteristic.WriteClientCharacteristicConfigurationDescriptorAsync(GattClientCharacteristicConfigurationDescriptorValue.Notify));
            if (status == GattCommunicationStatus.Unreachable)
            {
                Console.WriteLine("Error: Failed to enable notify.");
                return false;
            }
            //
            // Start to get sensor data
            //
            status = await gearvrCommandCharacteristic.WriteValueAsync(getBuffer(Constants.GEARVR_COMMAND_VR_MODE));
            if (status == GattCommunicationStatus.Unreachable)
            {

                Console.WriteLine("Error: Failed to enable notify.");
                return false;
            }
            return true;
        }

        async Task<bool> ConnectGearVR(DeviceInformation info)
        {
            if (service != null)
            {
                Close();
            }

            try
            {
                Console.WriteLine(string.Format("Trying to connect {0} {1}", info.Name, info.Id));
                service = await AsTask(GattDeviceService.FromIdAsync(info.Id));
                if (service == null)
                {
                    Console.WriteLine("Error: Another program is using the device.");
                    return false;
                }
                
                service.Session.SessionStatusChanged += Session_SessionStatusChanged;
                //
                // Get characteristics
                //

                var list = service.GetCharacteristics(Constants.GEARVR_DATA_CHARACTERISTICS_UUID);
                if (list.Count == 0)
                {
                    Console.WriteLine("Error: No characteristics. uuid=GEARVR_DATA_CHARACTERISTICS_UUID.");
                    return false;
                }
                Console.WriteLine("gearvrDataCharacteristics count " + list.Count);
                gearvrDataCharacteristic = list[0];

                list = service.GetCharacteristics(Constants.GEARVR_COMMAND_CHARACTERISTICS_UUID);
                if (list.Count == 0)
                {
                    Console.WriteLine("Error: No characteristics. uuid=GEARVR_COMMAND_CHARACTERISTICS_UUID.");
                    return false;
                }
                gearvrCommandCharacteristic = list[0];

                //
                // Set callbak
                //

                gearvrDataCharacteristic.ValueChanged += CallbackGearVR;

                //var status = await AsTask(gearvrDataCharacteristic.WriteClientCharacteristicConfigurationDescriptorAsync(GattClientCharacteristicConfigurationDescriptorValue.Notify));
                //if (status == GattCommunicationStatus.Unreachable)
                //{
                //    Console.WriteLine("Error: Failed to enable notify.");
                //    return false;
                //}

                await SetNotify();

                var status = await gearvrCommandCharacteristic.WriteValueAsync(getBuffer(Constants.GEARVR_COMMAND_SENSOR));
                if (status == GattCommunicationStatus.Success)
                {
                    Console.WriteLine("Connected.");
                    AHRS = new AHRS.MadgwickAHRS(1f / Constants.GEARVR_HZ, 0.01f);
                    type = Constants.TYPE_GEARVR;
                    bdaddr = BDAddrToString(service.Device.BluetoothAddress);
                    deviceId = service.Device.DeviceId;
                    deviceName = info.Name;
                    return true;
                }
                else
                {
                    Console.WriteLine("Error: Cannot connect. (Timeout)");
                    return false;
                }

            }
            catch (Exception e)
            {
                Console.WriteLine("Error: Caught exception. " + e.Message);
            }

            return false;
        }

        private void Session_SessionStatusChanged(GattSession sender, GattSessionStatusChangedEventArgs args)
        {
            Console.WriteLine("Session_SessionStatusChanged " + args.Status);
        }

        IBuffer getBuffer(short i)
        {
            var writer = new Windows.Storage.Streams.DataWriter();
            writer.WriteInt16(i);
            return writer.DetachBuffer();
        }

        private Task<T> AsTask<T>(IAsyncOperation<T> operation)
        {
            var tcs = new TaskCompletionSource<T>();
            operation.Completed = delegate
            {
                switch (operation.Status)
                {
                    case AsyncStatus.Completed: tcs.SetResult(operation.GetResults()); break;
                    case AsyncStatus.Error: tcs.SetException(operation.ErrorCode); break;
                    case AsyncStatus.Canceled: tcs.SetCanceled(); break;
                }
            };
            return tcs.Task;
        }

        public bool HasService { get { return service != null && service.Session.SessionStatus == GattSessionStatus.Active; } }
        public bool HasData { get { return gearvrDataCharacteristic != null; } }

        public int TimeMS { get { return DateTime.Now.Millisecond; } }

        public void SetAHRSBeta(float v)
        {
            AHRS.Beta = v;
        }

        public void SetAHRSPeriod(float v)
        {
            AHRS.SamplePeriod = v;
        }


        public void SetNotifyTest()
        {
            SetNotify().Wait();
        }

        public bool Reconnect()
        {
            if (gearvrDataCharacteristic != null)
            {
                Close();
                Connect(currentDevice).Wait();
                return true;
            }

            return false;
        }

        public void Recenter(double[] hmd_rotation)
        {
            return;
        }

        // Controller orientation in quaternion(x,y,z,w) rad.
        public double[] quaternion { get; private set; } = new double[4];
        // Orientation raw (x,y,z) (only for Daydream)
        public double[] orientation { get; private set; } = new double[3];
        // Acceleration in (x,y,z) ms^-2
        public double[] acceleration { get; private set; } = new double[3];
        // Gyroscope
        public double[] gyroscope { get; private set; } = new double[3];
        // Magnetometer (only for GearVR)
        public double[] magnetometer { get; private set; } = new double[3];
        // Position (caluclated by arm model)
        public double[] position { get; private set; } = new double[3];
        // Touch X,Y
        public double[] trackpad { get; private set; } = new double[2];
        // Buttons (touchpad touch, touchpad click, home, app(daydream)/back(gearvr), vol down, vol up, trigger(gearvr only))
        private const int CONST_BUTTONS = 7;
        public bool[] button { get; private set; } = new bool[CONST_BUTTONS];

        public bool useMagnetometer { get; set; } = true;

        public int TOUCH { get; } = 0;
        public int CLICK { get; } = 1;
        public int HOME { get; } = 2;
        public int APP { get; } = 3;
        public int VOLDOWN { get; } = 4;
        public int VOLUP { get; } = 5;
        public int TRIGGER { get; } = 6;
        public int BUTTONS { get; } = CONST_BUTTONS;

        public bool touch { get { return button[TOUCH]; } }
        public bool click { get { return button[CLICK]; } }
        public bool home { get { return button[HOME]; } }
        public bool app { get { return button[APP]; } }
        public bool voldown { get { return button[VOLDOWN]; } }
        public bool volup { get { return button[VOLUP]; } }
        public bool trigger { get { return button[TRIGGER]; } }

        // Version from last byte
        public int version { get; private set; } = 0;

        public int temperature { get; private set; } = 0;

        // 1=daydream controller, 2=gearvr controller
        public int type { get; private set; } = 0;

        // Device Name
        public string deviceName { get; private set; } = "";
        // Bluetooth address
        public string bdaddr { get; private set; } = "";
        // Device Id
        public string deviceId { get; private set; } = "";
    }

    class DeviceInfo
    {
        public DeviceInfo(DeviceInformation info, int type)
        {
            this.info = info;
            this.type = type;
        }

        public readonly DeviceInformation info;
        public readonly int type;
    }

    class DeviceManager
    {
        private bool initialized = false;
        private DeviceInformationCollection collectionDaydream;
        private DeviceInformationCollection collectionGearVR;
        private int next = 0;
        private object lockObject = new object();
        
        private async Task Initialize()
        {
            var selector = GattDeviceService.GetDeviceSelectorFromUuid(Constants.DAYDREAM_SERVICE_UUID);
            collectionDaydream = await DeviceInformation.FindAllAsync(selector);

            selector = GattDeviceService.GetDeviceSelectorFromUuid(Constants.GEARVR_SERVICE_UUID);
            collectionGearVR = await DeviceInformation.FindAllAsync(selector);
        }

        public DeviceInfo GetNext()
        {
            lock (lockObject)
            {
                if (!initialized)
                {
                    Initialize().Wait();
                    initialized = true;
                }
                if (next < collectionDaydream.Count)
                {
                    return new DeviceInfo(collectionDaydream[next++], Constants.TYPE_DAYDREAM);
                }
                int index = next - collectionDaydream.Count;
                if (index >= collectionGearVR.Count)
                {
                    return null;
                }
                next++;
                return new DeviceInfo(collectionGearVR[index], Constants.TYPE_GEARVR);
            }
        }
    }
}
