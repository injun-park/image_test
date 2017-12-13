//
// Created by kobuki on 17. 12. 14.
//

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/core/core.hpp"



using namespace cv;

/// Global variables

Mat src, src_gray;
Mat dst, detected_edges;

int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
char* window_name = "Edge Map";
bool dst_created = false;

image_transport::Subscriber sub;
image_transport::Publisher pub;

/**
 * @function CannyThreshold
 * @brief Trackbar callback - Canny thresholds input with a ratio 1:3
 */
void CannyThreshold(int, void*)
{
    /// Reduce noise with a kernel 3x3
    blur( src_gray, detected_edges, Size(3,3) );

    /// Canny detector
    Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

    /// Using Canny's output as a mask, we display our result
    dst = Scalar::all(0);

    src.copyTo( dst, detected_edges);
    imshow( window_name, dst );

}




void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        src = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::imshow("view", src);
        /// Create a matrix of the same type and size as src (for dst)
        dst.create( src.size(), src.type() );

        /// Convert the image to grayscale
        cvtColor( src, src_gray, CV_BGR2GRAY );
        /// Create a Trackbar for user to enter threshold
        createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );

        /// Show the image
        CannyThreshold(0, 0);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg();
        pub.publish(msg);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


/** @function main */
int main( int argc, char** argv )
{

    ros::init(argc, argv, "canny_edge_node");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    sub = it.subscribe("image", 1, imageCallback);
    pub = it.advertise("canny_edge", 1);
    ros::spin();
    return 0;
}