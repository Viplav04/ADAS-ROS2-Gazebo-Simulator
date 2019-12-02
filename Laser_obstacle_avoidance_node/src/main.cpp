#include "obstacle_avoidance.h"

int main(int argc,char * argv[])
{
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<Obstacle>());
	rclcpp::shutdown();
	return 0;
}