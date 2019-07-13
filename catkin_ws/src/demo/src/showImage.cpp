#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::namedWindow("image",0);
        cv::imshow("image", cv_bridge::toCvShare(msg, "bgr8")->image);
        cv::waitKey(1);
        ROS_INFO("Show a image OK!");
    }    
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from %s to 'bgr8'.", msg->encoding.c_str());
    }
}

static void help() { 
	std::cout 	<< "\n<Usage>"
			    << "\n>> ./example topic \n"
                << "\n argv[1]: receive topic"
			    << "\n" 
			    << endl;
}

int main(int argc, char** argv)
{
    /* parse argc */
    if (argc != 2) {
        help();
        return 1;
    }
    cv::String topic = argv[1];     

    /* init */
    ros::init(argc, argv, "node_showImage");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(topic, 1, imageCallback);
    ros::spin();

}

