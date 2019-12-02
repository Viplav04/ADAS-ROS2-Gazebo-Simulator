#include "lane_assist.h"

int main(int argc,char * argv[])
{
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<ImageConverter>());
	rclcpp::shutdown();
	return 0;
}