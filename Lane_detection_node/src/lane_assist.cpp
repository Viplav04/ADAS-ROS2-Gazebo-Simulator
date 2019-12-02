#include "lane_assist.h"

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;
static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW2 = "Image window2";
static const std::string OPENCV_WINDOW3 = "Image window3";

ImageConverter::ImageConverter() : Node("lane_assist"){
    // call that from the baseclass "Node" and not from image transport 
    img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("robocam_out");
    img_sub_=this->create_subscription<sensor_msgs::msg::Image>("/custom_ns/depth_camera/image_raw",std::bind(&ImageConverter::imagecallback,this,std::placeholders::_1));
    vel_pub=this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel");
	  range_sub=this->create_subscription<sensor_msgs::msg::Range>("laser/range",std::bind(&ImageConverter::chattercallback,this,std::placeholders::_1));
    
   // get_parameter_or("")
   		get_parameter_or<double>("scale_angular", a_scale_,1.0);
		get_parameter_or<double>("scale_linear", l_scale_,1.0);
		get_parameter_or<double>("angular", angular_,0.05);
		get_parameter_or<double>("linear", linear_,0.05);
		get_parameter_or<double>("left_threshold", left_threshold,0);
		get_parameter_or<double>("right_threshold", right_threshold,768);
    
    cv::namedWindow(OPENCV_WINDOW);
	  cv::namedWindow(OPENCV_WINDOW2);
		cv::namedWindow(OPENCV_WINDOW3);
		
}
ImageConverter::~ImageConverter()
     {
      cv::destroyWindow(OPENCV_WINDOW);
      cv::destroyWindow(OPENCV_WINDOW2);
	    cv::destroyWindow(OPENCV_WINDOW3);
     }


void ImageConverter::imagecallback(const sensor_msgs::msg::Image::SharedPtr msg) {
  
     
        bridge= cv_bridge::toCvCopy(msg,"bgr8") ; 
		bridge->image;

        
        
        
    	CvMemStorage* storage = cvCreateMemStorage(0);
		CvSeq* lines;
		int i, c, d;
		float c1[50];
		float m, angle;
		float buf;
		float m1;
		float dis;
		int k = 0, k1 = 0;
		int count = 0;

		float xv;
		float yv;
		int vc = 0;
		float xvan = 0, yvan = 0;
		static float xvan1 = 0, yvan1 = 0;
		float num = 0;
		static float count1 = 0;
		float dv;
		float vxx, vyy;

        Mat src_gray;
		Mat dst, detected_edges;

		int edgeThresh = 1;
		int lowThreshold=50;
		int const max_lowThreshold = 100;
		int ratio = 4;
		int kernel_size = 3;

		src=bridge->image;
		  /// Create a matrix of the same type and size as src (for dst)


  		/// Convert the image to grayscale
  		cvtColor( src, src_gray, CV_BGR2GRAY );
		  /// Reduce noise with a kernel 3x3
  		blur( src_gray, detected_edges, Size(3,3) );
 		 dst.create( src.size(), src.type() );
 		 /// Canny detector
 		 Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

  		/// Using Canny's output as a mask, we display our result
  		dst = Scalar::all(0);

		src.copyTo( dst, detected_edges);
  	
        //cv::imshow(OPENCV_WINDOW,bridge->image);
		Mat mask;
		Mat img_mask;
		 mask = Mat::zeros(dst.size(),dst.type() ); //create mask image
		 cv::Point pts[4] = {
          	cv::Point(-250, 768),
          	cv::Point(450, 450),
       	  	cv::Point(650, 450),
      	 	cv::Point(1280, 768)};
		cv::fillConvexPoly(mask, pts, 4, cv::Scalar(0, 0, 255)); //fill image with whites
		imshow( OPENCV_WINDOW, mask);
		cv::bitwise_and(dst, mask, img_mask);
		cvtColor(img_mask,img_mask,CV_BGR2GRAY);
		imshow(OPENCV_WINDOW2,src_gray);	
		std::vector<cv::Vec4i> lines;
		HoughLinesP(img_mask, lines, 1,
				CV_PI /180, 50,20, 20);   
		
	
// SORT RIGHT AND LEFT LINES
/**
 *@brief Sort all the detected Hough lines by slope.
 *@brief The lines are classified into right or left depending
 *@brief on the sign of their slope and their approximate location
 *@param lines is the vector that contains all the detected lines
 *@param img_edges is used for determining the image center
 *@return The output is a vector(2) that contains all the classified lines
*/
    std::vector<std::vector<cv::Vec4i> > output(2);
  size_t j = 0;
  cv::Point ini;
  cv::Point fini;
  double slope_thresh = 0.3;
  std::vector<double> slopes;
  std::vector<cv::Vec4i> selected_lines;
  std::vector<cv::Vec4i> right_lines, left_lines;

  // Calculate the slope of all the detected lines
  for (auto i : lines) {
    ini = cv::Point(i[0], i[1]);
    fini = cv::Point(i[2], i[3]);

    // Basic algebra: slope = (y1 - y0)/(x1 - x0)
    double slope = (static_cast<double>(fini.y) - static_cast<double>(ini.y))/(static_cast<double>(fini.x) - static_cast<double>(ini.x) + 0.00001);

    // If the slope is too horizontal, discard the line
    // If not, save them  and their respective slope
    if (std::abs(slope) > slope_thresh) {
      slopes.push_back(slope);
      selected_lines.push_back(i);
    }
  }
    // Split the lines into right and left lines
    img_center = static_cast<double>((dst.cols / 2));
  while (j < selected_lines.size()) {
    ini = cv::Point(selected_lines[j][0], selected_lines[j][1]);
    fini = cv::Point(selected_lines[j][2], selected_lines[j][3]);

    // Condition to classify line as left side or right side
    if (slopes[j] > 0 && fini.x > img_center && ini.x > img_center) {
      right_lines.push_back(selected_lines[j]);
      right_flag = true;
    } else if (slopes[j] < 0 && fini.x < img_center && ini.x < img_center) {
        left_lines.push_back(selected_lines[j]);
        left_flag = true;
    }
    j++;
  }

  output[0] = right_lines;
  output[1] = left_lines;

  // REGRESSION FOR LEFT AND RIGHT LINES
/**
 *@brief Regression takes all the classified line segments initial and final points and fits a new lines out of them using the method of least squares.
 *@brief This is done for both sides, left and right.
 *@param left_right_lines is the output of the lineSeparation function
 *@param inputImage is used to select where do the lines will end
 *@return output contains the initial and final points of both lane boundary lines
 */
 std::vector<cv::Point> outputl(4);
  cv::Point ini1;
  cv::Point fini1;
  cv::Point ini2;
  cv::Point fini2;
  cv::Vec4d right_line;
  cv::Vec4d left_line;
  std::vector<cv::Point> right_pts;
  std::vector<cv::Point> left_pts;
  std::vector<std::vector<cv::Vec4i> > left_right_lines;
  left_right_lines = {right_lines,left_lines};
  // If right lines are being detected, fit a line using all the init and final points of the lines
  if (right_flag == true) {
    for (auto i : left_right_lines[0]) {
      ini1 = cv::Point(i[0], i[1]);
      fini1 = cv::Point(i[2], i[3]);

      right_pts.push_back(ini1);
      right_pts.push_back(fini1);
    }

    if (right_pts.size() > 0) {
      // The right line is formed here
      cv::fitLine(right_pts, right_line, CV_DIST_L2, 0, 0.01, 0.01);
      right_m = right_line[1] / right_line[0];
      right_b = cv::Point(right_line[2], right_line[3]);
    }
  }

  // If left lines are being detected, fit a line using all the init and final points of the lines
  if (left_flag == true) {
    for (auto j : left_right_lines[1]) {
      ini2 = cv::Point(j[0], j[1]);
      fini2 = cv::Point(j[2], j[3]);

      left_pts.push_back(ini2);
      left_pts.push_back(fini2);
    }

    if (left_pts.size() > 0) {
      // The left line is formed here
      cv::fitLine(left_pts, left_line, CV_DIST_L2, 0, 0.01, 0.01);
      left_m = left_line[1] / left_line[0];
      left_b = cv::Point(left_line[2], left_line[3]);
    }
  }

  // One the slope and offset points have been obtained, apply the line equation to obtain the line points
  int ini_y = src.rows;
  int fin_y = 470;

  double right_ini_x = ((ini_y - right_b.y) / right_m) + right_b.x;
  double right_fin_x = ((fin_y - right_b.y) / right_m) + right_b.x;

  double left_ini_x = ((ini_y - left_b.y) / left_m) + left_b.x;
  double left_fin_x = ((fin_y - left_b.y) / left_m) + left_b.x;
  
  outputl[0] = cv::Point(right_ini_x, ini_y);
  outputl[1] = cv::Point(right_fin_x, fin_y);
  outputl[2] = cv::Point(left_ini_x, ini_y);
  outputl[3] = cv::Point(left_fin_x, fin_y);

// TURN PREDICTION
/**
 *@brief Predict if the lane is turning left, right or if it is going straight
 *@brief It is done by seeing where the vanishing point is with respect to the center of the image
 *@return String that says if there is left or right turn or if the road is straight
 */

  std::string outputs;
  double vanish_x;
  double thr_vp = 10;

  // The vanishing point is the point where both lane boundary lines intersect
  vanish_x = static_cast<double>(((right_m*right_b.x) - (left_m*left_b.x) - right_b.y + left_b.y) / (right_m - left_m));

  // The vanishing points location determines where is the road turning
  if (vanish_x < (img_center - thr_vp)) {
    outputs = "LEFT TURN";
//	VelMsg.linear.x=1.0;VelMsg.angular.z=0.5;
  }
  else if (vanish_x > (img_center + thr_vp)){
    outputs = "RIGHT TURN";
//	VelMsg.linear.x=1.0;VelMsg.angular.z=-0.5;
}
  else if (vanish_x >= (img_center - thr_vp) && vanish_x <= (img_center + thr_vp)) {
    outputs = "STRAIGHT";}

 //vel_pub->publish(VelMsg);



// PLOT RESULTS
/**
 *@brief This function plots both sides of the lane, the turn prediction message and a transparent polygon that covers the area inside the lane boundaries
 *@param inputImage is the original captured frame
 *@param lane is the vector containing the information of both lines
 *@param turn is the output string containing the turn information
 *@return The function returns a 0
 */
    std::vector<cv::Point> poly_points;
//	std::vector<cv::Point> lane;
    cv::Mat outputf;

  // Create the transparent polygon for a better visualization of the lane
  src.copyTo(outputf);
  poly_points.push_back(outputl[2]);
  poly_points.push_back(outputl[0]);
  poly_points.push_back(outputl[1]);
  poly_points.push_back(outputl[3]);
  cv::fillConvexPoly(outputf, poly_points, cv::Scalar(255, 0, 0), CV_AA, 0);
  cv::addWeighted(outputf, 0.3, src, 1.0 - 0.3, 0, src);

  // Plot both lines of the lane boundary
 	//	 for( size_t i = 0; i < lines.size(); i++ )
 //{
 //   Vec4i l = lines[i];
   line( src, outputl[0],outputl[1], Scalar(0,255,255), 3, CV_AA);
 // }	
 
  	//	 for( size_t k = 0; k < left_lines.size(); k++ )
  //{
    //Vec4i m = left_lines[k];
    line( src, outputl[2], outputl[3], Scalar(0,255,255), 3, CV_AA);
  //}	
   cv::putText(src, outputs, cv::Point(400, 50), cv::FONT_HERSHEY_COMPLEX_SMALL, 2, cvScalar(255, 0, 0), 1, CV_AA);
      imshow(OPENCV_WINDOW3,img_mask);
        cv::waitKey(3);
	img_pub_->publish(msg);
}

void ImageConverter::chattercallback(const sensor_msgs::msg::Range::SharedPtr rmsg) {           
	std::string outputs2;
	
	min_range = rmsg->range;

	if(min_range<=5)  {
	
         outputs2 = "STOP, Reverse or Change Direction"   ;
	  	 cv::putText(src, outputs2, cv::Point(300, 350), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(0, 0, 255), 1, CV_AA);   
	}
	  
}