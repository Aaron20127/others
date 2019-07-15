#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "opencv2/opencv.hpp"
// #include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>

using namespace std;

static void help() { 
	std::cout 	<< "\n<Usage>"
			    << "\n>> ./example 0 topic data/test.png   10"
			    << "\n>> ./example 1 topic 192.168.1.0     10\n"
                << "\n argv[1]: command id, 0, 1, ..."
                << "\n argv[2]: receive topic"
                << "\n argv[3]: camera ip address"
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
    cv::VideoCapture cap;
    if (cmd == 0) {
        /* read image form path */
        image = cv::imread(address);
        if(image.empty()) {
            cout << "open image error!\n";
            return 1;
        }
    } else if (cmd == 1)  {
        /* use rtsp  protocol to read HIKVISION camera */
        cv::String url=cv::String("rtsp://admin:aaron20127@") + address + "//Streaming/Channels/1";
        cap = cv::VideoCapture(url);
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
    while (true) 
    {
        if (cmd == 1) {
            cap >> image;
            if(image.empty()) {
                cout << "open image error!\n";
                return 1;
            }
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
