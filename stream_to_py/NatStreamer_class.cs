using System;
using System.IO;
using System.Collections.Generic;
using System.Collections;
using System.ComponentModel;
using System.Data;
using System.Text;
using System.Windows.Forms;
using System.Net;
using System.Threading;
using System.Runtime.InteropServices;

using NatNetML;
using System.Reflection;

using System.Net.NetworkInformation;
using System.Text;
using System.Diagnostics;
using HDF5DotNet;

namespace stream_to_py
{
    class NatStreamer
    {
        /*  [NatNet] Network connection configuration    */
        private static NatNetML.NatNetClientML mNatNet;    // The client instance

        // [NatNet] Our NatNet Frame of Data object
        private NatNetML.FrameOfMocapData m_FrameOfData = new NatNetML.FrameOfMocapData();


        private static string mStrLocalIP = "127.0.0.1";   // Local IP address (string)
        private static string mStrServerIP = "127.0.0.1";  // Server IP address (string)
        private static NatNetML.ConnectionType mConnectionType = ConnectionType.Multicast; // Multicast or Unicast mode


        // [NatNet] Description of the Active Model List from the server (e.g. Motive)
        NatNetML.ServerDescription desc = new NatNetML.ServerDescription();

        /*  List for saving each of datadescriptors */
        private static List<NatNetML.DataDescriptor> mDataDescriptor = new List<NatNetML.DataDescriptor>();

        /*  Lists and Hashtables for saving data descriptions   */
        private static Hashtable mHtSkelRBs = new Hashtable();
        private static List<RigidBody> mRigidBodies = new List<RigidBody>();
        private static List<Skeleton> mSkeletons = new List<Skeleton>();
        private static List<ForcePlate> mForcePlates = new List<ForcePlate>();

        //server constants
        double m_ServerFramerate = 1.0f;
        float m_ServerToMillimeters = 1000.0f; //from m to mm
        int mDroppedFrames = 0;
        int mLastFrame = 0;

        //quene for writing data
        // [NatNet] Queue holding our incoming mocap frames the NatNet server (e.g. Motive)
        private Queue<NatNetML.FrameOfMocapData> m_FrontQueue = new Queue<NatNetML.FrameOfMocapData>();
        private Queue<NatNetML.FrameOfMocapData> m_BackQueue = new Queue<NatNetML.FrameOfMocapData>();
        private static object FrontQueueLock = new object();
        private static object BackQueueLock = new object();


        // Records the age of each frame in m_FrameQueue at the time it arrived.
        private Queue<double> m_FrameTransitLatencies = new Queue<double>();
        private Queue<double> m_TotalLatencies = new Queue<double>();
        bool mRecording = false;
        TextWriter mWriter;

        /*  boolean value for detecting change in asset */
        private static bool mAssetChanged = false;

        // frame timing information
        double m_fLastFrameTimestamp = 0.0f;
        QueryPerfCounter m_FramePeriodTimer = new QueryPerfCounter();
        QueryPerfCounter m_ProcessingTimer = new QueryPerfCounter();
        private double interframeDuration;
        private int droppedFrameIndicator = 0;
        int mUIBusyCount = 0;

        //update the mocap frame thread
        delegate void UpdateUICallback();
        Thread bmi3d_service_thread;
        bool mApplicationRunning = true;
        private string _track_type;
        private int _object_num;
        private double[] frame_result;

        private static bool debug = false;
        private Stopwatch stopwatch = new Stopwatch();

        public NatStreamer()
        {

        }

        public int connect_to_motive() {
            Console.WriteLine("SampleClientML managed client application starting...\n");
            /*  [NatNet] Initialize client object and connect to the server  */
            connectToServer();                          // Initialize a NatNetClient object and connect to a server.

            Console.WriteLine("============================ SERVER DESCRIPTOR ================================\n");
            /*  [NatNet] Confirming Server Connection. Instantiate the server descriptor object and obtain the server description. */
            bool connectionConfirmed = fetchServerDescriptor();    // To confirm connection, request server description data

            if (connectionConfirmed)                         // Once the connection is confirmed.
            {
                Console.WriteLine("============================= DATA DESCRIPTOR =================================\n");
                Console.WriteLine("Now Fetching the Data Descriptor.\n");
                fetchDataDescriptor();                  //Fetch and parse data descriptor

                //set up datalistener
                mNatNet.OnFrameReady += new NatNetML.FrameReadyEventHandler(m_NatNet_OnFrameReady);
                return 1;
            }
            else
            {
                return 1;
            }

        }
        public double get_frame_rate() {
            int nBytes = 0;
            byte[] response = new byte[10000];
            int rc;
            rc = mNatNet.SendMessageAndWait("FrameRate", out response, out nBytes);
            if (rc == 0)
            {
                try
                {
                    m_ServerFramerate = BitConverter.ToSingle(response, 0);
                    return m_ServerFramerate;
                }
                catch (System.Exception ex)
                {
                    Console.WriteLine(ex.Message);
                }
            }
            return -1;

        }
        public double[] get_last_frame(string track_type, int object_num) {
            /*
             * when asking for data set, default to the first marker set 
             */
            //grab a frame
            FrameOfMocapData data = mNatNet.GetLastFrameOfData();

            
            //
            switch (track_type) {
                default: 
                    Console.WriteLine("Unrecognized track object type {0}", track_type);
                    return null;

                case "markers": //defailt to return first marker of the firsr marker set
                    NatNetML.MarkerSetData ms = data.MarkerSets[0];
                    if (object_num < ms.nMarkers)
                    {
                        return new double[3] {ms.Markers[object_num].x * m_ServerToMillimeters,
                                              ms.Markers[object_num].x * m_ServerToMillimeters,
                                              ms.Markers[object_num].z * m_ServerToMillimeters
                            };
                    }
                    else {
                        Console.WriteLine("the specified object number( {0} ) is greater than the total markers ({1})", object_num, ms.nMarkers);
                        return null;
                 
                    }



                case "rb":

                    if (debug)
                    {
                        Console.WriteLine("asked for rigid body number {0}", object_num);
                        Console.WriteLine("number of rigid body: {0}", data.nRigidBodies);

                    }

                    if (object_num < data.nRigidBodies)
                    {

                        NatNetML.RigidBodyData rb = data.RigidBodies[object_num];
                        //get the quotenions
                        float[] quat = new float[4] { rb.qx, rb.qy, rb.qz, rb.qw };
                        float[] eulers = new float[3];
                        eulers = NatNetClientML.QuatToEuler(quat, NATEulerOrder.NAT_XYZr);
                        double x = RadiansToDegrees(eulers[0]);     // convert to degrees
                        double y = RadiansToDegrees(eulers[1]);
                        double z = RadiansToDegrees(eulers[2]);
                        if (debug) {
                            Console.WriteLine("received x y z positions");
                            Console.WriteLine("{0}, {1}, {2}", x, y, z);
                            stopwatch.Stop();
                            Console.WriteLine("Reading the quene takes {0} ms", stopwatch.ElapsedMilliseconds);
                        }


                        return new double[7] { rb.x * m_ServerToMillimeters, //mm for obvious reaons
                                               rb.y * m_ServerToMillimeters,
                                               rb.z * m_ServerToMillimeters,
                                               x, y, z, data.iFrame }; //angles in degrees
                    }
                    else
                        return null;
            }
        }
        private static void connectToServer()
        {
            /*  [NatNet] Instantiate the client object  */
            mNatNet = new NatNetML.NatNetClientML();

            /*  [NatNet] Checking verions of the NatNet SDK library  */
            int[] verNatNet = new int[4];           // Saving NatNet SDK version number
            verNatNet = mNatNet.NatNetVersion();
            Console.WriteLine("NatNet SDK Version: {0}.{1}.{2}.{3}", verNatNet[0], verNatNet[1], verNatNet[2], verNatNet[3]);

            /*  [NatNet] Connecting to the Server    */
            Console.WriteLine("\nConnecting...\n\tLocal IP address: {0}\n\tServer IP Address: {1}\n\n", mStrLocalIP, mStrServerIP);

            NatNetClientML.ConnectParams connectParams = new NatNetClientML.ConnectParams();
            connectParams.ConnectionType = mConnectionType;
            connectParams.ServerAddress = mStrServerIP;
            connectParams.LocalAddress = mStrLocalIP;
            mNatNet.Connect(connectParams);
        }
        static bool fetchServerDescriptor()
        {
            NatNetML.ServerDescription m_ServerDescriptor = new NatNetML.ServerDescription();
            int errorCode = mNatNet.GetServerDescription(m_ServerDescriptor);

            if (errorCode == 0)
            {
                Console.WriteLine("Success: Connected to the server\n");
                parseSeverDescriptor(m_ServerDescriptor);
                return true;
            }
            else
            {
                Console.WriteLine("Error: Failed to connect. Check the connection settings.");
                Console.WriteLine("Program terminated (Enter ESC to exit)");
                return false;
            }
        }
        static void parseSeverDescriptor(NatNetML.ServerDescription server)
        {
            Console.WriteLine("Server Info:");
            Console.WriteLine("\tHost: {0}", server.HostComputerName);
            Console.WriteLine("\tApplication Name: {0}", server.HostApp);
            Console.WriteLine("\tApplication Version: {0}.{1}.{2}.{3}", server.HostAppVersion[0], server.HostAppVersion[1], server.HostAppVersion[2], server.HostAppVersion[3]);
            Console.WriteLine("\tNatNet Version: {0}.{1}.{2}.{3}\n", server.NatNetVersion[0], server.NatNetVersion[1], server.NatNetVersion[2], server.NatNetVersion[3]);
        }
        static void fetchDataDescriptor()
        {
            /*  [NatNet] Fetch Data Descriptions. Instantiate objects for saving data descriptions and frame data    */
            bool result = mNatNet.GetDataDescriptions(out mDataDescriptor);
            if (result)
            {
                Console.WriteLine("Success: Data Descriptions obtained from the server.");
                parseDataDescriptor(mDataDescriptor);
            }
            else
            {
                Console.WriteLine("Error: Could not get the Data Descriptions");
            }
            Console.WriteLine("\n");
        }
        static void parseDataDescriptor(List<NatNetML.DataDescriptor> description)
        {
            //  [NatNet] Request a description of the Active Model List from the server. 
            //  This sample will list only names of the data sets, but you can access 
            int numDataSet = description.Count;
            Console.WriteLine("Total {0} data sets in the capture:", numDataSet);

            for (int i = 0; i < numDataSet; ++i)
            {
                int dataSetType = description[i].type;
                // Parse Data Descriptions for each data sets and save them in the delcared lists and hashtables for later uses.
                switch (dataSetType)
                {
                    case ((int)NatNetML.DataDescriptorType.eMarkerSetData):
                        NatNetML.MarkerSet mkset = (NatNetML.MarkerSet)description[i];
                        Console.WriteLine("\tMarkerSet ({0})", mkset.Name);
                        break;


                    case ((int)NatNetML.DataDescriptorType.eRigidbodyData):
                        NatNetML.RigidBody rb = (NatNetML.RigidBody)description[i];
                        Console.WriteLine("\tRigidBody ({0})", rb.Name);

                        // Saving Rigid Body Descriptions
                        mRigidBodies.Add(rb);
                        break;


                    case ((int)NatNetML.DataDescriptorType.eSkeletonData):
                        NatNetML.Skeleton skl = (NatNetML.Skeleton)description[i];
                        Console.WriteLine("\tSkeleton ({0}), Bones:", skl.Name);

                        //Saving Skeleton Descriptions
                        mSkeletons.Add(skl);

                        // Saving Individual Bone Descriptions
                        for (int j = 0; j < skl.nRigidBodies; j++)
                        {

                            Console.WriteLine("\t\t{0}. {1}", j + 1, skl.RigidBodies[j].Name);
                            int uniqueID = skl.ID * 1000 + skl.RigidBodies[j].ID;
                            int key = uniqueID.GetHashCode();
                            mHtSkelRBs.Add(key, skl.RigidBodies[j]); //Saving the bone segments onto the hashtable
                        }
                        break;


                    case ((int)NatNetML.DataDescriptorType.eForcePlateData):
                        NatNetML.ForcePlate fp = (NatNetML.ForcePlate)description[i];
                        Console.WriteLine("\tForcePlate ({0})", fp.Serial);

                        // Saving Force Plate Channel Names
                        mForcePlates.Add(fp);

                        for (int j = 0; j < fp.ChannelCount; j++)
                        {
                            Console.WriteLine("\t\tChannel {0}: {1}", j + 1, fp.ChannelNames[j]);
                        }
                        break;

                    default:
                        // When a Data Set does not match any of the descriptions provided by the SDK.
                        Console.WriteLine("\tError: Invalid Data Set");
                        break;
                }
            }
        }
        private void RecordButton_Click()
        {
            string command = "StartRecording";

            int nBytes = 0;
            byte[] response = new byte[10000];
            int rc = mNatNet.SendMessageAndWait(command, 3, 100, out response, out nBytes);
            if (rc != 0)
            {
                OutputMessage(command + " not handled by server");
            }
            else
            {
                int opResult = System.BitConverter.ToInt32(response, 0);
                if (opResult == 0)
                    OutputMessage(command + " handled and succeeded.");
                else
                    OutputMessage(command + " handled but failed.");
            }
        }
        private void StopRecordButton_Click()
        {
            string command = "StopRecording";

            int nBytes = 0;
            byte[] response = new byte[10000];
            int rc = m_NatNet.SendMessageAndWait(command, out response, out nBytes);

            if (rc != 0)
            {
                OutputMessage(command + " not handled by server");
            }
            else
            {
                int opResult = System.BitConverter.ToInt32(response, 0);
                if (opResult == 0)
                    OutputMessage(command + " handled and succeeded.");
                else
                    OutputMessage(command + " handled but failed.");
            }
        }
        static void fetchFrameData(NatNetML.FrameOfMocapData data, NatNetML.NatNetClientML client)
        {

            /*  Exception handler for cases where assets are added or removed.
                Data description is re-obtained in the main function so that contents
                in the frame handler is kept minimal. */
            if ((data.bTrackingModelsChanged == true || data.nRigidBodies != mRigidBodies.Count || data.nSkeletons != mSkeletons.Count || data.nForcePlates != mForcePlates.Count))
            {
                mAssetChanged = true;
            }

            /*  Processing and ouputting frame data every 200th frame.
                This conditional statement is included in order to simplify the program output */
            if (data.iFrame % 200 == 0)
            {
                if (data.bRecording == false)
                    Console.WriteLine("Frame #{0} Received:", data.iFrame);
                else if (data.bRecording == true)
                    Console.WriteLine("[Recording] Frame #{0} Received:", data.iFrame);

                processFrameData(data);
            }
        }
        public void start_thread() {
            // create and run an Update UI thread
            //UpdateUICallback d = new UpdateUICallback(UpdateUI);

            bmi3d_service_thread.Start();

        }
        void m_NatNet_OnFrameReady(NatNetML.FrameOfMocapData data, NatNetML.NatNetClientML client)
        {
            // measure time between frame arrival (inter frame)
            m_FramePeriodTimer.Stop();
            interframeDuration = m_FramePeriodTimer.Duration();

            // measure processing time (intra frame)
            m_ProcessingTimer.Start();

            // process data
            // NOTE!  do as little as possible here as we're on the data servicing thread
            ProcessFrameOfData(ref data);

            // report if we are taking longer than a mocap frame time
            // which eventually will back up the network receive buffer and result in frame drop
            m_ProcessingTimer.Stop();
            double appProcessingTimeMSecs = m_ProcessingTimer.Duration();
            double mocapFramePeriodMSecs = (1.0f / m_ServerFramerate) * 1000.0f;
            if (appProcessingTimeMSecs > mocapFramePeriodMSecs)
            {
                //changed for now
                Console.WriteLine("Warning : Frame handler taking longer than frame period: " + appProcessingTimeMSecs.ToString("F2"));
            }

            m_FramePeriodTimer.Start();
        }
        static void processFrameData(NatNetML.FrameOfMocapData data)
        {
            /*  Parsing Rigid Body Frame Data   */
            for (int i = 0; i < mRigidBodies.Count; i++)
            {
                int rbID = mRigidBodies[i].ID;              // Fetching rigid body IDs from the saved descriptions

                for (int j = 0; j < data.nRigidBodies; j++)
                {
                    if (rbID == data.RigidBodies[j].ID)      // When rigid body ID of the descriptions matches rigid body ID of the frame data.
                    {
                        NatNetML.RigidBody rb = mRigidBodies[i];                // Saved rigid body descriptions
                        NatNetML.RigidBodyData rbData = data.RigidBodies[j];    // Received rigid body descriptions

                        if (rbData.Tracked == true)
                        {
                            Console.WriteLine("\tRigidBody ({0}):", rb.Name);
                            Console.WriteLine("\t\tpos ({0:N3}, {1:N3}, {2:N3})", rbData.x, rbData.y, rbData.z);

                            // Rigid Body Euler Orientation
                            float[] quat = new float[4] { rbData.qx, rbData.qy, rbData.qz, rbData.qw };
                            float[] eulers = new float[3];

                            eulers = NatNetClientML.QuatToEuler(quat, NATEulerOrder.NAT_XYZr); //Converting quat orientation into XYZ Euler representation.
                            double xrot = RadiansToDegrees(eulers[0]);
                            double yrot = RadiansToDegrees(eulers[1]);
                            double zrot = RadiansToDegrees(eulers[2]);

                            Console.WriteLine("\t\tori ({0:N3}, {1:N3}, {2:N3})", xrot, yrot, zrot);
                        }
                        else
                        {
                            Console.WriteLine("\t{0} is not tracked in current frame", rb.Name);
                        }
                    }
                }

            }

            /* Parsing Skeleton Frame Data  */
            for (int i = 0; i < mSkeletons.Count; i++)      // Fetching skeleton IDs from the saved descriptions
            {
                int sklID = mSkeletons[i].ID;

                for (int j = 0; j < data.nSkeletons; j++)
                {
                    if (sklID == data.Skeletons[j].ID)      // When skeleton ID of the description matches skeleton ID of the frame data.
                    {
                        NatNetML.Skeleton skl = mSkeletons[i];              // Saved skeleton descriptions
                        NatNetML.SkeletonData sklData = data.Skeletons[j];  // Received skeleton frame data

                        Console.WriteLine("\tSkeleton ({0}):", skl.Name);
                        Console.WriteLine("\t\tSegment count: {0}", sklData.nRigidBodies);

                        /*  Now, for each of the skeleton segments  */
                        for (int k = 0; k < sklData.nRigidBodies; k++)
                        {
                            NatNetML.RigidBodyData boneData = sklData.RigidBodies[k];

                            /*  Decoding skeleton bone ID   */
                            int skeletonID = HighWord(boneData.ID);
                            int rigidBodyID = LowWord(boneData.ID);
                            int uniqueID = skeletonID * 1000 + rigidBodyID;
                            int key = uniqueID.GetHashCode();

                            NatNetML.RigidBody bone = (RigidBody)mHtSkelRBs[key];   //Fetching saved skeleton bone descriptions

                            //Outputting only the hip segment data for the purpose of this sample.
                            if (k == 0)
                                Console.WriteLine("\t\t{0:N3}: pos({1:N3}, {2:N3}, {3:N3})", bone.Name, boneData.x, boneData.y, boneData.z);
                        }
                    }
                }
            }

            /*  Parsing Force Plate Frame Data  */
            for (int i = 0; i < mForcePlates.Count; i++)
            {
                int fpID = mForcePlates[i].ID;                  // Fetching force plate IDs from the saved descriptions

                for (int j = 0; j < data.nForcePlates; j++)
                {
                    if (fpID == data.ForcePlates[j].ID)         // When force plate ID of the descriptions matches force plate ID of the frame data.
                    {
                        NatNetML.ForcePlate fp = mForcePlates[i];                // Saved force plate descriptions
                        NatNetML.ForcePlateData fpData = data.ForcePlates[i];    // Received forceplate frame data

                        Console.WriteLine("\tForce Plate ({0}):", fp.Serial);

                        // Here we will be printing out only the first force plate "subsample" (index 0) that was collected with the mocap frame.
                        for (int k = 0; k < fpData.nChannels; k++)
                        {
                            Console.WriteLine("\t\tChannel {0}: {1}", fp.ChannelNames[k], fpData.ChannelData[k].Values[0]);
                        }
                    }
                }
            }
            Console.WriteLine("\n");
        }
        void ProcessFrameOfData(ref NatNetML.FrameOfMocapData data)
        {

            TelemetryData telemetry = new TelemetryData();
            bool bMotiveHardwareLatenciesAvailable = data.CameraMidExposureTimestamp != 0;
            if (bMotiveHardwareLatenciesAvailable)
            {
                telemetry.TotalLatency = mNatNet.SecondsSinceHostTimestamp(data.CameraMidExposureTimestamp) * 1000.0;
                telemetry.MotiveTotalLatency = (data.TransmitTimestamp - data.CameraMidExposureTimestamp) / (double)desc.HighResClockFrequency * 1000.0;
            }
            bool bMotiveLatenciesAvailable = data.CameraDataReceivedTimestamp != 0;
            if (bMotiveLatenciesAvailable)
            {
            }
            telemetry.TransmitLatency = mNatNet.SecondsSinceHostTimestamp(data.TransmitTimestamp) * 1000.0;


            // detect and reported any 'reported' frame drop (as reported by server)
            if (m_fLastFrameTimestamp != 0.0f)
            {
                double framePeriod = 1.0f / m_ServerFramerate;
                double thisPeriod = data.fTimestamp - m_fLastFrameTimestamp;
                double delta = thisPeriod - framePeriod;
                double fudgeFactor = 0.002f; // 2 ms
                if (delta > fudgeFactor)
                {
                    //OutputMessage("Frame Drop: ( ThisTS: " + data.fTimestamp.ToString("F3") + "  LastTS: " + m_fLastFrameTimestamp.ToString("F3") + " )");
                    double missingPeriod = delta / framePeriod;
                    int nMissing = (int)(missingPeriod + 0.5);
                    mDroppedFrames += nMissing;
                    telemetry.DroppedFrames = nMissing;
                    droppedFrameIndicator = 10; // for graphing only
                }
                else
                {
                    droppedFrameIndicator = 0;
                }
            }

            // check and report frame drop (frame id based)
            if (mLastFrame != 0)
            {
                if ((data.iFrame - mLastFrame) != 1)
                {
                    //OutputMessage("Frame Drop: ( ThisFrame: " + data.iFrame.ToString() + "  LastFrame: " + mLastFrame.ToString() + " )");
                    //mDroppedFrames++;
                }
            }
            /*
            if (data.bTrackingModelsChanged)
                mNeedTrackingListUpdate = true;
                */
            // NatNet manages the incoming frame of mocap data, so if we want to keep it, we must make a copy of it
            FrameOfMocapData deepCopy = new FrameOfMocapData(data);

            // Add frame to a background queue for access by other threads
            // Note: this lock should always succeed immediately, unless connecting/disconnecting, when the queue gets reset
            lock (BackQueueLock)
            {
                m_BackQueue.Enqueue(deepCopy);

                // limit background queue size to 10 frames
                while (m_BackQueue.Count > 3)
                {
                    m_BackQueue.Dequeue();
                }
            }

            // Update the shared UI queue, only if the UI thread is not updating (we don't want to wait here as we're in the data update thread)
            bool lockAcquired = false;
            try
            {
                Monitor.TryEnter(FrontQueueLock, ref lockAcquired);
                if (lockAcquired)
                {
                    // [optional] clear the frame queue before adding a new frame (UI only wants most recent frame)
                    m_FrontQueue.Clear();
                    m_FrontQueue.Enqueue(deepCopy);

                    m_FrameTransitLatencies.Clear();
                    m_FrameTransitLatencies.Enqueue(telemetry.TransmitLatency);

                    m_TotalLatencies.Clear();
                    m_TotalLatencies.Enqueue(telemetry.TotalLatency);
                }
                else
                {
                    mUIBusyCount++;
                }
            }
            finally
            {
                if (lockAcquired)
                    Monitor.Exit(FrontQueueLock);
            }

            // recording : write packet to data file
            if (mRecording)
            {
                WriteFrame(deepCopy, telemetry);
            }

            mLastFrame = data.iFrame;
            m_fLastFrameTimestamp = data.fTimestamp;

        }
        public void start_recording_to_file() {
            try
            {
                mWriter = File.CreateText("recordingData.txt");
                mRecording = true;
                Console.WriteLine("Recording started...");
            }
            catch (System.Exception ex)
            {
                Console.WriteLine("Record Error : " + ex.Message);
            }

        }
        public void stop_recording_to_file() {
            if (mRecording)
            {
                mRecording = false;
                mWriter.Close();
                Console.WriteLine("Recording stoppped");

            }

        }
        private void WriteFrame(FrameOfMocapData data, TelemetryData telemetry)
        {
            String str = "";
            bool recordMarkerData = true;
            //bool recordForcerData = false;
            //bool recordRBData = false;

            str += data.fTimestamp.ToString("F3") + "\t";
            str += telemetry.TransmitLatency.ToString("F3") + "\t";
            str += telemetry.TotalLatency.ToString("F3") + "\t";
            str += telemetry.DroppedFrames.ToString() + "\t";

            // 'all' markerset data
            if (recordMarkerData)
            {
                for (int i = 0; i < m_FrameOfData.nMarkerSets; i++)
                {
                    NatNetML.MarkerSetData ms = m_FrameOfData.MarkerSets[i];
                    if (ms.MarkerSetName == "all")
                    {
                        for (int j = 0; j < ms.nMarkers; j++)
                        {
                            str += ms.Markers[j].x.ToString("F3") + "\t";
                            str += ms.Markers[j].y.ToString("F3") + "\t";
                            str += ms.Markers[j].z.ToString("F3") + "\t";
                        }
                    }
                }
            }

            mWriter.WriteLine(str);
        }
        static double RadiansToDegrees(double dRads)
        {
            return dRads * (180.0f / Math.PI);
        }
        static int LowWord(int number)
        {
            return number & 0xFFFF;
        }
        static int HighWord(int number)
        {
            return ((number >> 16) & 0xFFFF);
        }
        private void OutputMessage(string strMessage)
        {

            if (!mApplicationRunning)
                return;
            Console.WriteLine(strMessage);

        }

        public class QueryPerfCounter
        {
            [DllImport("KERNEL32")]
            private static extern bool QueryPerformanceCounter(out long lpPerformanceCount);

            [DllImport("Kernel32.dll")]
            private static extern bool QueryPerformanceFrequency(out long lpFrequency);

            private long start;
            private long stop;
            private long frequency;
            Decimal multiplier = new Decimal(1.0e9);

            public QueryPerfCounter()
            {
                if (QueryPerformanceFrequency(out frequency) == false)
                {
                    // Frequency not supported
                    throw new Win32Exception();
                }
            }

            public void Start()
            {
                QueryPerformanceCounter(out start);
            }

            public void Stop()
            {
                QueryPerformanceCounter(out stop);
            }

            // return elapsed time between start and stop, in milliseconds.
            public double Duration()
            {
                double val = ((double)(stop - start) * (double)multiplier) / (double)frequency;
                val = val / 1000000.0f;   // convert to ms
                return val;
            }
        }

        public class TelemetryData
        {
            public double MotiveTotalLatency = -1.0;
            public double TransmitLatency = -1.0;
            public double TotalLatency = -1.0;
            public int DroppedFrames = 0;
        }
    }
}

