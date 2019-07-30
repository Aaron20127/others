#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "opencv2/opencv.hpp"
// #include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>


using namespace std;
using namespace cv;


// read images with pattern
static void readImagesWithPattern(cv::String pattern, vector<cv::Mat> &images)
{
    std::vector<cv::String> fn;
    cv::glob(pattern, fn, false);

    for ( int i=0; i < fn.size(); i++ ) {
        images.push_back(cv::imread(fn[i]));
    }
}

static void help() { 
	std::cout 	<< "\n<Usage>"
			    << "\n>> ./example topic 10 folder1/*.jpg folder2/*.jpg ..."
                << "\n argv[1]: topic name"
                << "\n argv[2]: send how many images per second"
                << "\n argv[3]: image folder 1"
                << "\n argv[4]: image folder 2"
                << "\n ..."
                << "\n argv[n]: image folder n"
			    << "\n" 
			    << endl;
}

int main(int argc, char** argv)
{
    /* parse argc */
    if (argc < 4) {
        help();
        return 1;
    }

    cv::String topic = argv[1];
    int fps = atoi(argv[2]);

    /* get image form each folder */
    vector<vector<cv::Mat>> image_vector;
    for (int i = 3; i < argc; i++) {
        vector<cv::Mat> image;
        readImagesWithPattern(argv[i], image);
        if(image.empty()) {
            cout << "open folder \"" << argv[i] << "\" error!\n";
            return 1;
        }
        image_vector.push_back(image);
    }

    /* init ros */
    ros::init(argc, argv, "node_images");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise(topic, 1);
    ros::Rate loop_rate(fps);// 30fps

    /* package and send */
    int rows = image_vector[0][0].rows;
    int cols = image_vector[0][0].cols;
    int size = image_vector.size();
    cv::Mat image_pair(rows, cols * size, CV_8UC3, Scalar(0,0,0));
    int i = 0;

    while (true) 
    {
        /* package multiple image to one image */

        for (int j = 0; j < size; j++) {
            cv::Mat image = image_vector[j][i];
            image.copyTo(image_pair.colRange(j*cols, (j+1)*cols));
        }

        i++;
        if(i == image_vector[0].size()){
            i = 0;
        }
        

        /* add a header */
        std_msgs::Header header;
        header.frame_id = "new_camera";
        header.stamp = ros::Time::now();

        /* add a image */
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", image_pair).toImageMsg();

        /* publish */
        pub.publish(msg);

        cout << "[ " << header.stamp << " ] " << "send a image ok" << endl;
        // ros::spinOnce();
        // ROS_INFO("Send a image OK!");
        loop_rate.sleep();

    }

}
