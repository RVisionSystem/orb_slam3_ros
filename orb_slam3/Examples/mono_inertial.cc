#include<iostream>
#include<chrono>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<System.h>
#include "ImuTypes.h"
#include <thread>
#include <serial/serial.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

using namespace std;
using namespace cv;

serial::Serial ser;

struct IMUData {
    float axf, ayf, azf; // Accelerometer data
    float gxf, gyf, gzf; // Gyroscope data
    double t; // Timestamp
};

vector<ORB_SLAM3::IMU::Point> vImuMeas;
double time_offset_in_seconds;
std::string input;
int data_packet_start;
uint8_t last_received_message_number;
bool received_message = false;
queue<IMUData> imuBuffer; // IMU data buffer
mutex bufferMutex; 

double ttrack_tot = 0;

void getImuData()
{   
    while(true){
        try
        {
            if (ser.isOpen())
            {   
                if (ser.available())
                {   
                    std::string read = ser.read(ser.available());
                    input += read;
                    while (input.length() >= 28) // while there might be a complete package in input
                    {
                        // Parse for data packets
                        data_packet_start = input.find("$\x03");
                        if (data_packet_start != std::string::npos)
                        {   
                            if ((input.length() >= data_packet_start + 28) && (input.compare(data_packet_start + 26, 2, "\r\n") == 0))
                            {   
                                // get gyro values
                                int16_t gx = (((0xff &(char)input[data_packet_start + 10]) << 8) | 0xff &(char)input[data_packet_start + 11]);
                                int16_t gy = (((0xff &(char)input[data_packet_start + 12]) << 8) | 0xff &(char)input[data_packet_start + 13]);
                                int16_t gz = (((0xff &(char)input[data_packet_start + 14]) << 8) | 0xff &(char)input[data_packet_start + 15]);
                                // calculate rotational velocities in rad/s
                                float gxf = gx * (4000.0 / 65536.0) * (M_PI / 180.0) * 25.0;
                                float gyf = gy * (4000.0 / 65536.0) * (M_PI / 180.0) * 25.0;
                                float gzf = gz * (4000.0 / 65536.0) * (M_PI / 180.0) * 25.0;

                                // get accelerometer values
                                int16_t ax = (((0xff &(char)input[data_packet_start + 16]) << 8) | 0xff &(char)input[data_packet_start + 17]);
                                int16_t ay = (((0xff &(char)input[data_packet_start + 18]) << 8) | 0xff &(char)input[data_packet_start + 19]);
                                int16_t az = (((0xff &(char)input[data_packet_start + 20]) << 8) | 0xff &(char)input[data_packet_start + 21]);
                                // calculate accelerations in m/s²
                                float axf = ax * (8.0 / 65536.0) * 9.81;
                                float ayf = ay * (8.0 / 65536.0) * 9.81;
                                float azf = az * (8.0 / 65536.0) * 9.81;

                                // Obtain current timestamp
                                float t = std::chrono::system_clock::now().time_since_epoch().count() / 1e9;
                                
                                bufferMutex.lock();
                                imuBuffer.push({axf, ayf, azf, gxf, gyf, gzf, t});
                                bufferMutex.unlock();
                                
                                uint8_t received_message_number = input[data_packet_start + 25];

                                if (received_message) // can only check for continuous numbers if already received at least one packet
                                {
                                    uint8_t message_distance = received_message_number - last_received_message_number;
                                    if (message_distance > 1)
                                    {
                                        cerr << "Missed " << message_distance - 1 << " MPU6050 data packets from arduino." << endl;
                                    }
                                }
                                else
                                {
                                    received_message = true;
                                }
                                last_received_message_number = received_message_number;

                                // Erase processed packet from input
                                input.erase(0, data_packet_start + 28);
                            }
                                else
                            {
                                if (input.length() >= data_packet_start + 28)
                                {
                                    input.erase(0, data_packet_start + 1); // delete up to false data_packet_start character so it is not found again
                                }
                                else
                                {
                                    // do not delete start character, maybe complete package has not arrived yet
                                    input.erase(0, data_packet_start);
                                }
                            }
                            
                        }
                        else
                        {
                            // no start character found in input, so delete everything
                            input.clear();
                        }
                    }
                }
            }
            else
            {
                // Try to open the serial port
                ser.setPort("/dev/ttyUSB0");
                ser.setBaudrate(115200);
                serial::Timeout to = serial::Timeout::simpleTimeout(1000);
                ser.setTimeout(to);
                ser.open();
                
                if (ser.isOpen())
                {
                    std::cout << "Serial port initialized and opened." << std::endl;
                }
                else
                {
                    std::cerr << "Failed to open serial port. Retrying in 5 seconds." << std::endl;
                    std::this_thread::sleep_for(std::chrono::seconds(5));
                }
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error: " << e.what() << std::endl;
            // Handle the error gracefully or retry
        }
        // Delay to achieve desired update rate (100 Hz)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
}
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
{
    ifstream fImu;
    fImu.open(strImuPath.c_str());
    vTimeStamps.reserve(5000);
    vAcc.reserve(5000);
    vGyro.reserve(5000);

    while(!fImu.eof())
    {
        string s;
        getline(fImu,s);
        if (s[0] == '#')
            continue;

        if(!s.empty())
        {
            string item;
            size_t pos = 0;
            double data[7];
            int count = 0;
            while ((pos = s.find(',')) != string::npos) {
                item = s.substr(0, pos);
                data[count++] = stod(item);
                s.erase(0, pos + 1);
            }
            item = s.substr(0, pos);
            data[6] = stod(item);

            vTimeStamps.push_back(data[0]/1e9);
            vAcc.push_back(cv::Point3f(data[4],data[5],data[6]));
            vGyro.push_back(cv::Point3f(data[1],data[2],data[3]));
        }
    }
}


int main(int argc, char *argv[]) {
    if (argc != 4 && argc < 6) {
        cerr << endl << "Usage: ./monoi mode path_to_vocabulary path_to_settings [path_to_sequence_folder path_to_times_file]" << endl;
        return 1;
    }
    string mode = argv[1];

    cv::Mat im;
    cv::VideoCapture cap; 

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[2], argv[3], ORB_SLAM3::System::IMU_MONOCULAR, true);

    if(mode == "dataset") {
        const int num_seq = (argc-4)/2;
        cout << "num_seq = " << num_seq << endl;
        bool bFileName= (((argc-4) % 2) == 1);
        string file_name;
        if (bFileName)
        {
            file_name = string(argv[argc-1]);
            cout << "file name: " << file_name << endl;
        }

        vector< vector<string> > vstrImageFilenames;
        vector< vector<double> > vTimestampsCam;
        vector< vector<cv::Point3f> > vAcc, vGyro;
        vector< vector<double> > vTimestampsImu;
        vector<int> nImages;
        vector<int> nImu;
        vector<int> first_imu(num_seq, 0);

        int seq;
        vstrImageFilenames.resize(num_seq);
        vTimestampsCam.resize(num_seq);
        vAcc.resize(num_seq);
        vGyro.resize(num_seq);
        vTimestampsImu.resize(num_seq);
        nImages.resize(num_seq);
        nImu.resize(num_seq);

        int tot_images = 0;
        for(int seq = 0; seq < num_seq; seq++) {
            cout << "Loading images for sequence " << seq << "...";

            string pathSeq(argv[(2 * seq) + 4]);
            string pathTimeStamps(argv[(2 * seq) + 5]);

            string pathCam0 = pathSeq + "/mav0/cam0/data";
            string pathImu = pathSeq + "/mav0/imu0/data.csv";

            LoadImages(pathCam0, pathTimeStamps, vstrImageFilenames[seq], vTimestampsCam[seq]);
            cout << "LOADED!" << endl;

            cout << "Loading IMU for sequence " << seq << "...";
            LoadIMU(pathImu, vTimestampsImu[seq], vAcc[seq], vGyro[seq]);
            cout << "LOADED!" << endl;

            nImages[seq] = vstrImageFilenames[seq].size();
            tot_images += nImages[seq];
            nImu[seq] = vTimestampsImu[seq].size();

            if((nImages[seq] <= 0) || (nImu[seq] <= 0)) {
                cerr << "ERROR: Failed to load images or IMU for sequence" << seq << endl;
                return 1;
            }
            // Find first imu to be considered, supposing imu measurements start first
            while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][0])
                first_imu[seq]++;
            first_imu[seq]--; // first imu measurement to be considered
        }

        // Vector for tracking time statistics
        vector<float> vTimesTrack;
        vTimesTrack.resize(tot_images);

        //cout << endl << "-------" << endl;
        cout.precision(17);

        float imageScale = SLAM.GetImageScale();

        double t_resize = 0.f;
        double t_track = 0.f;

        int proccIm=0;
        for (seq = 0; seq<num_seq; seq++)
        {

            // Main loop
            cv::Mat im;
            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            proccIm = 0;
            for(int ni=0; ni<nImages[seq]; ni++, proccIm++)
            {
                // Read image from file
                im = cv::imread(vstrImageFilenames[seq][ni],cv::IMREAD_UNCHANGED); //CV_LOAD_IMAGE_UNCHANGED);

                double tframe = vTimestampsCam[seq][ni];

                if(im.empty())
                {
                    cerr << endl << "Failed to load image at: "
                        <<  vstrImageFilenames[seq][ni] << endl;
                    return 1;
                }

                if(imageScale != 1.f)
                {
        #ifdef REGISTER_TIMES
            #ifdef COMPILEDWITHC11
                        std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
            #else
                        std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
            #endif
        #endif
                        int width = im.cols * imageScale;
                        int height = im.rows * imageScale;
                        cv::resize(im, im, cv::Size(width, height));
        #ifdef REGISTER_TIMES
            #ifdef COMPILEDWITHC11
                        std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
            #else
                        std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
            #endif
                        t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
                        SLAM.InsertResizeTime(t_resize);
        #endif
                    }

                    // Load imu measurements from previous frame
                    vImuMeas.clear();

                    if(ni>0)
                    {
                        // cout << "t_cam " << tframe << endl;

                        while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][ni])
                        {
                            vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[seq][first_imu[seq]].x,vAcc[seq][first_imu[seq]].y,vAcc[seq][first_imu[seq]].z,
                                                                    vGyro[seq][first_imu[seq]].x,vGyro[seq][first_imu[seq]].y,vGyro[seq][first_imu[seq]].z,
                                                                    vTimestampsImu[seq][first_imu[seq]]));
                            first_imu[seq]++;
                        }
                    }

            #ifdef COMPILEDWITHC11
                    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            #else
                    std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
            #endif

                    // Pass the image to the SLAM system
                    // cout << "tframe = " << tframe << endl;
                    SLAM.TrackMonocular(im,tframe,vImuMeas); // TODO change to monocular_inertial

            #ifdef COMPILEDWITHC11
                    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            #else
                    std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
            #endif

            #ifdef REGISTER_TIMES
                        t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
                        SLAM.InsertTrackTime(t_track);
            #endif

                        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
                        ttrack_tot += ttrack;
                        // std::cout << "ttrack: " << ttrack << std::endl;

                        vTimesTrack[ni]=ttrack;

                        // Wait to load the next frame
                        double T=0;
                        if(ni<nImages[seq]-1)
                            T = vTimestampsCam[seq][ni+1]-tframe;
                        else if(ni>0)
                            T = tframe-vTimestampsCam[seq][ni-1];

                        if(ttrack<T)
                            usleep((T-ttrack)*1e6); // 1e6
                    }
                    if(seq < num_seq - 1)
                    {
                        cout << "Changing the dataset" << endl;

                        SLAM.ChangeDataset();
                    }
                }

                // Save camera trajectory
                if (bFileName)
                {
                    const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
                    const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
                    SLAM.SaveTrajectoryEuRoC(f_file);
                    SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
                }
                else
                {
                    SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
                    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
                }
    }
                    
    
    else if(mode == "realtime") {
        // Initialize camera
        cap = cv::VideoCapture(4);

        cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

        double timestamp = 0;
        if(!cap.isOpened()){
            std::cout << "No video stream detected" << std::endl;
            return -1;
        }

        std::thread imu(getImuData);

        while(1){//!SLAM.isShutDown()){

            if(!imuBuffer.empty()){
                cap >> im;
                timestamp = std::chrono::system_clock::now().time_since_epoch().count()/1e9;

                // Start the IMU data collection thread
                bufferMutex.lock(); 
                    while(!imuBuffer.empty() && imuBuffer.front().t <= timestamp)
                    {
                    cv::Point3f acc(imuBuffer.front().axf, imuBuffer.front().ayf, imuBuffer.front().azf);
                    cv::Point3f gyr(imuBuffer.front().gxf, imuBuffer.front().gyf, imuBuffer.front().gzf);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, imuBuffer.front().t));
                    imuBuffer.pop();
                    }
                bufferMutex.unlock();

                // Pass the image to the SLAM system
                SLAM.TrackMonocular(im, timestamp, vImuMeas);
                vImuMeas.clear();
            }

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
        }
    else {
        cerr << "Invalid mode. Available modes are 'realtime' and 'dataset'." << endl;
        return 1;
    }

   if (mode == "dataset") {
        // Release resources for mode 1
        SLAM.Shutdown();
    } else if (mode == "realtime") {
        // Release resources for mode 2
        cap.release();
        SLAM.Shutdown();
}
}
