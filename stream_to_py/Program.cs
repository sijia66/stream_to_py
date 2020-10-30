using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Diagnostics;



namespace stream_to_py
{
    class Program
    {
        //socket for talking to BMI3D
        private static TcpListener server_socket = null;
        private static TcpClient client = null;
        private static readonly int port = 1230;
        //private static readonly string server_addr = "10.155.205.164";//need to be BMI3D's server;
        private static readonly int HEADERSIZE = 10; // the same as it is on BMI3D
        private static readonly int byte_len = 128; //how many bytes to read from the stream

        private static string full_msg;
        private static int byte_count;
        private static NetworkStream stream;
        private static byte[] send_data;

        private static readonly int byte_buffer_len = 32;

        private static bool debug = true;


        static void Main(string[] args) {
            

            //Connect to NatNet interface and SDK. 
            NatStreamer natStreamer = new NatStreamer();
            //connect to motive and retrieve server and data descriptions
            Console.WriteLine(natStreamer.connect_to_motive());

            Stopwatch stopwatch = new Stopwatch();

            //connect to BMI3D using TCP/IP
            connect_to_BMI3D();
            

            //always check for connection 
            while (true) {
                bool exit = false;
                Console.WriteLine();
                if (client != null) {
                    server_socket.Stop();
                    server_socket.Start();
                    Console.WriteLine("Restarted the server socket!");
   
                }

                Console.WriteLine("Waiting for BMI3D connection {0}", port);
                client = server_socket.AcceptTcpClient();
                Console.WriteLine("Connected at port {0}", port);
                stream = client.GetStream();


                string command = null;
                string stream_type = null;

                while (exit == false) {

                    //wait for BMI3D to issue command 
                    // read one frame
                    byte[] byte_frame = new byte[byte_buffer_len];
                    int curr_buffer_length = byte_buffer_len;

                    //allocate 10 frames of byte_buffer_lenth for streaming comamnds
                    byte[] byte_msg_frames = new byte[byte_buffer_len * 10];

                    stream.Read(byte_frame, 0, byte_buffer_len);
                    byte[] header_bytes = byte_frame.Take(HEADERSIZE).ToArray();
                    int.TryParse(Encoding.Default.GetString(header_bytes), out int num_expected_bytes);

                    //nnly proceed if the expected num_bytes is greater than zero
                    if (num_expected_bytes == 0) {
                        continue;

                    }

                    //copy over the array
                    Array.Copy(byte_frame, byte_msg_frames, byte_buffer_len);

                    if (debug) {
                        Console.WriteLine("expected bytes:" + num_expected_bytes.ToString());
                        Console.WriteLine(curr_buffer_length.ToString());
                        stopwatch.Start();
                    }

                    //decode the command
                    byte[] command_bytes = new byte[num_expected_bytes];
                    Array.Copy(byte_msg_frames, HEADERSIZE, command_bytes, 0, num_expected_bytes);

                    if (debug) {
                        Console.WriteLine("received bytes:" + System.Text.Encoding.Default.GetString(command_bytes));

                    }

                    command = Encoding.Default.GetString(command_bytes);

                    if (debug){
                        Console.WriteLine(command);

                    }

                    //serve the command
                    int byte_size;
                    switch (command)
                    {
                        case "start":
                            Console.WriteLine("received:" + command);
                            break;
                        case "stop":
                            Console.WriteLine("BMI3D disconnected...");
                            natStreamer.stop_recording();
                            exit = true;
                            break;
                        case "send_rigid_bodies":
                            stream_type = "rb";
                            if (debug) { Console.WriteLine("stream type set to {0}", stream_type); }
                            break;
                        case "send_markers":
                            stream_type = "markers";
                            break;
                 
                        case "get":
                            //check if the string type is none
                            if (stream_type == null)
                            {
                                Console.WriteLine(stream_type);
                                throw new System.ArgumentException("stream_type cannot be null", "original");
                            }
                                

                            double[] frame_of_data = natStreamer.get_last_frame(stream_type, 0);

                            if (debug) {
                                foreach (double i in frame_of_data) {
                                    Console.WriteLine(i.ToString());
                                }
                            }
              
                            //send back the value to BMI3D
                            //convert to string
                            string data_string = string.Join(",", frame_of_data);

                            //convert the data string  and send back to bmi3d
                            byte_size = Encoding.ASCII.GetByteCount(data_string);
                            stream.Write(Encoding.ASCII.GetBytes(data_string), 0,byte_size);
                            break;

                        case "gfr": //get framerate
                            double fr = natStreamer.get_frame_rate();
                            byte_size = Encoding.ASCII.GetByteCount(fr.ToString());
                            stream.Write(Encoding.ASCII.GetBytes(fr.ToString()), 0, byte_size);
  
                            break;
                        case "start_rec":
                            natStreamer.start_recording();
                            break;
                        case "set":
                            Console.WriteLine("received:" + command);
                            break;
                        default:
                            //return -1
                            Console.WriteLine("Unsupported commands " + command);
                            break;

                    }
                    //exit = true;
                    if (debug) {
                        stopwatch.Stop();
                        Console.WriteLine("Elapsed Time is {0} ms", stopwatch.ElapsedMilliseconds);
                    }

                }
            }


            /*

            
            //send a quick message
 
            */

        }

        private static void connect_to_BMI3D() {

            try
            {
                //IPAddress localAddr = IPAddress.Parse("127.0.0.1");
                server_socket = new TcpListener(IPAddress.Any, port);
                //start the server
                server_socket.Start();
                //Console.WriteLine("Waiting for BMI3D connection {0}", port);
                //client = server_socket.AcceptTcpClient();

                //Console.WriteLine("Connected at port {0}", port);
            }
            catch (System.Net.Sockets.SocketException a)
            {
                Console.WriteLine("Connection failed!");
                Console.WriteLine(a.Message);
            }


        }
    } 
}
