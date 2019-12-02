#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <iostream>
#include "std_msgs/msg/string.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/imgproc/imgproc.hpp>     //make sure to include the relevant headerfiles
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "sensor_msgs/msg/image.hpp"
#include <opencv/cvaux.h>
#include <math.h>
#include <vector>
#include <opencv/cxcore.h>
#include <geometry_msgs/msg/twist.hpp>
#include "sensor_msgs/msg/range.hpp"
//#include "boost/thread/mutex.hpp"
//#include "boost/thread/thread.hpp"


using namespace std::chrono_literals;

class ImageConverter : public rclcpp::Node {
    public:
    ImageConverter();
    ~ImageConverter();
    void imagecallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void chattercallback(const sensor_msgs::msg::Range::SharedPtr rmsg);
    
   // ~ImageConverter();
    
    private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr  img_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub;
    float min_range;
    geometry_msgs::msg::Twist VelMsg;
    double linear_, angular_;
	double l_scale_, a_scale_;
	double left_threshold, right_threshold;
    float prevVelocity_angular, prevVelocity_linear, newVelocity_angular,newVelocity_linear;
    float derive_angular, derive_linear, dt = 0.5;
    float horizontalcount;
    cv::Mat image;
    cv_bridge::CvImagePtr bridge;
    double img_size;
    double img_center;
    bool left_flag = false;  // Tells us if there's left boundary of lane detected
    bool right_flag = false;  // Tells us if there's right boundary of lane detected
    cv::Point right_b;  // Members of both line equations of the lane boundaries:
    double right_m;  // y = m*x + b
    cv::Point left_b;  //
    double left_m;  //
    cv::Mat src;


};