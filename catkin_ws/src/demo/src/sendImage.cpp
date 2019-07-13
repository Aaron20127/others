#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "opencv2/opencv.hpp"
// #include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>

using namespace std;

static void help() { 
	std::cout 	<< "\n<Usage>"
			    << "\n>> ./example topic 192.168.1.0 \n"
                << "\n argv[1]: receive topic"
                << "\n argv[2]: camera ip address"
			    << "\n" 
			    << endl;
}

int main(int argc, char** argv)
{
    /* parse argc */
    if (argc != 3) {
        help();
        return 1;
    }
    cv::String ip = argv[2];
    cv::String topic = argv[1];

    /* init ros */
    ros::init(argc, argv, "node_images");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise(topic, 1);

    /* read camera */
    cv::String url=cv::String("rtsp://admin:aaron20127@") + ip + "//Streaming/Channels/1";
    cv::VideoCapture cap(url);
    ros::Rate loop_rate(30);// 30fps
    cv::Mat image;

    /* send */
    while (true) 
    {
        cap >> image;
        if(image.empty()) printf("\nopen image error!\n");

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        pub.publish(msg);
        // ros::spinOnce();
        ROS_INFO("Send a image OK!");
        loop_rate.sleep();

    }

}
