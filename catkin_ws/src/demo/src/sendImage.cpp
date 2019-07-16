#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "opencv2/opencv.hpp"
// #include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>


using namespace std;
using namespace cv;

// init camera parameters 
static void initCameraPara(map<cv::String, cv::Mat> & ip_mtx,
                      map<cv::String, cv::Mat> & ip_dist)
{
    //camera 1
    cv::String ip = "192.168.100.1";
    ip_mtx[ip] = (Mat_<double>(3,3) << 2.6182e+03, 0, 973.1464, 
                                                0, 2.6228e+03, 571.7691,
                                                0, 0, 1);
    ip_dist[ip] = (Mat_<double>(1,5) << -0.5222, -0.2738, 0, 0, 0);  

    //camera 2
    ip = "192.168.100.2";
    ip_mtx[ip] = (Mat_<double>(3,3) << 2.6182e+03, 0, 973.1464, 
                                              0, 2.6228e+03, 571.7691,
                                            0, 0, 1);
    ip_dist[ip] = (Mat_<double>(1,5) << -0.5222, -0.2738, 0, 0, 0);  

    //camera 3
    ip = "192.168.100.3";
    ip_mtx[ip] = (Mat_<double>(3,3) << 2.6182e+03, 0, 973.1464, 
                                       0, 2.6228e+03, 571.7691,
                                       0, 0, 1);
    ip_dist[ip] = (Mat_<double>(1,5) << 0.5222, 0.2738, 0, 0, 0);  

    //camera 4
    ip = "192.168.100.4";
    ip_mtx[ip] = (Mat_<double>(3,3) << 2.6182e+03, 0, 973.1464, 
                                       0, 2.6228e+03, 571.7691,
                                       0, 0, 1);
    ip_dist[ip] = (Mat_<double>(1,5) << -0.5222, -0.2738, 0, 0, 0);
}

static void findUndistortRectifyMap(cv::Mat &map1, cv::Mat &map2, 
                                    cv::Size image_size, 
                                    cv::Mat & mtx,
                                    cv::Mat & dist)
{
    cv::Mat R;
    cv::initUndistortRectifyMap(mtx, dist, R, mtx, image_size, CV_32FC1, map1, map2);
}

// undistort image
static void undistortImage(const cv::Mat & src, cv::Mat & dst,
                           const cv::Mat & map1, const cv::Mat & map2) 
{       
    cv::remap(src, dst, map1, map2, INTER_LINEAR);
}

// read images with pattern
static void readImagesWithPattern(cv::String pattern, vector<cv::Mat> &images)
{
    std::vector<cv::String> fn;
    cv::glob(pattern, fn, false);

    for (const auto filename:fn) {
        images.push_back(cv::imread(filename));
    }
}

static void help() { 
	std::cout 	<< "\n<Usage>"
			    << "\n>> ./example 0 topic  data/*.jpg     10"
			    << "\n>> ./example 1 topic 192.168.1.0     10\n"
                << "\n argv[1]: command id, 0, 1, ..."
                << "\n argv[2]: receive topic"
                << "\n argv[3]: image folder or camera ip address"
                << "\n argv[4]: send how many images per second"
			    << "\n" 
			    << endl;
}

int main(int argc, char** argv)
{
    /* parse argc */
    if (argc != 5) {
        help();
        return 1;
    }

    int cmd = atoi(argv[1]);
    cv::String topic = argv[2];
    cv::String address = argv[3];
    int fps = atoi(argv[4]);

    /* read image */
    cv::Mat image;
    vector<cv::Mat> images;
    cv::VideoCapture cap;

    map<cv::String, cv::Mat> ip_mtx;
    map<cv::String, cv::Mat> ip_dist;
    cv::Mat map1, map2;

    if (cmd == 0) {
        /* read image form dir */
        readImagesWithPattern(address, images);
        if(images.empty()) {
            cout << "open image error!\n";
            return 1;
        }
    } else if (cmd == 1)  {
        /* use rtsp  protocol to read HIKVISION camera */
        cv::String url=cv::String("rtsp://admin:aaron20127@") + address + "//Streaming/Channels/1";
        cap = cv::VideoCapture(url);

        cout << "undistorting ..." << endl;
        while (!image.empty()) {
            cap >> image;
        }
        initCameraPara(ip_mtx, ip_dist);
        findUndistortRectifyMap(map1, map2, image.size(), ip_mtx[address], ip_dist[address]);
        cout << "undistorting successfully ." << endl;
    } else {
        help();
        return 1;
    }

    /* init ros */
    ros::init(argc, argv, "node_images");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise(topic, 1);
    ros::Rate loop_rate(fps);// 30fps

    /* send */
    int i = 0;
    while (true) 
    {
        if (cmd == 0) {
            image = images[i];
            i++;
            if(i == images.size()){
                i = 0;
            }
        }
        else if (cmd == 1) {
            cap >> image;
            if(image.empty()) {
                cout << "open image error!\n";
            }
            undistortImage(image, image, map1, map2);
        } else {

        }

        /* add a header */
        std_msgs::Header header;
        header.frame_id = "new_camera";
        header.stamp = ros::Time::now();

        /* add a image */
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();

        /* publish */
        pub.publish(msg);

        cout << "[ " << header.stamp << " ] " << "send a image ok" << endl;
        // ros::spinOnce();
        // ROS_INFO("Send a image OK!");
        loop_rate.sleep();

    }

}
