#include <iostream>
#include <chrono>

#include "opencv2/opencv.hpp"

#include "System.h"


using namespace std;
using namespace cv;

int main(int argc, char **argv) {
    if (argc < 3) {
        cerr << endl << "Usage: ./slam_on_vid <path_to_vocab> <path_to_settings> <path_to_video or webcam if left empty>" << endl;
        return 1;
    }

    VideoCapture cap;

    if (argc == 3) {
      cap.open(0);
    }
    else {
      cap.open(argv[3]);
    }

    if (!cap.isOpened()) {
        cerr << "Error opening video file" << endl;
        return 1;
    }

    int width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);

    cout << "VIDEO DIMENSIONS: " << width << " x " << height << endl;

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

    cout << "\n\n SLAM INITIALIZED \n\n";

    float imageScale = SLAM.GetImageScale();

    int ni = 0;

    vector<Sophus::SE3f> traj; // To store the traj and plot later

    while (true) {
        Mat frame;
        cap >> frame;

        if (frame.empty()) 
            break;
        
        // For some reason, vertical videos don't work
        if (width < height)
            rotate(frame, frame, ROTATE_90_COUNTERCLOCKWISE);

        auto timestamp = std::chrono::high_resolution_clock::now();
        auto timestamp_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(timestamp.time_since_epoch()).count();

        float imageScale = 0.333;

        int new_width = frame.cols * imageScale;
        int new_height = frame.rows * imageScale;

        //resize(frame, frame, Size(new_width, new_height));
      
        Mat gray;
        cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        imshow("FRAME GIVEN TO ORB SLAM", gray);
      
        Sophus::SE3f Tcw_SE3 = SLAM.TrackMonocular(gray,timestamp_seconds);

        if (!Tcw_SE3.matrix().isZero()) {
            Eigen::Vector3f tcw = Tcw_SE3.translation();
            Sophus::SE3f Twc_SE3 = Tcw_SE3.inverse();
            Eigen::Vector3f twc = Twc_SE3.translation();

            std::cout << "Frame " << ni << " | Timestamp: " << timestamp_seconds << " | Translation (tcw): ";
            std::cout << "[" << twc.x() << ", " << twc.y() << ", " << twc.z() << "]\n";

            traj.push_back(Twc_SE3);
        }

        ni += 1;
      
        //waitKey(1); // 1ms delay
    }

    SLAM.Shutdown();

    ofstream trajectoryFile("../trajectory.txt");

    for (size_t i = 0; i < traj.size(); i++) {
        Eigen::Vector3f pos = traj[i].translation();
        Eigen::Quaternionf q = traj[i].unit_quaternion();
        
        // Format: timestamp tx ty tz qx qy qz qw
        trajectoryFile << i << " " 
                      << pos.x() << " " << pos.y() << " " << pos.z() << " "
                      << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
    }

    trajectoryFile.close();

    cout << "Trajectory data saved to 'trajectory.txt'" << endl;

    return 0;
}
