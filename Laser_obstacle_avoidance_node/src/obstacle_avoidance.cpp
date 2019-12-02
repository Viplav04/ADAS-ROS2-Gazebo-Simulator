
#include "obstacle_avoidance.h"

using namespace std::chrono_literals;

Obstacle::Obstacle() : Node("ObstacleAvoidance"){
	vel_pub=this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel");	   
	sub=this->create_subscription<sensor_msgs::msg::Range>("laser/range",std::bind(&Obstacle::chattercallback,this,std::placeholders::_1)); 
}


void Obstacle::chattercallback(const sensor_msgs::msg::Range::SharedPtr msg) {           
	min_range = msg->range;
	if(min_range<=4)  {
		cmdvel.linear.x <=0;
		printf("STOP or Change direction or Reverse");
                vel_pub->publish(cmdvel);
                
	}
		
}

	




// create one class (meaining .h & .cpp)
// create one file with main function
// instantiate the class
