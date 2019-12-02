
#include <iostream>
#include <memory>
#include <cstdio>
#include <vector>
#include <chrono>
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/range.hpp"

using namespace std::chrono_literals;




// class declaration 
class Obstacle : public rclcpp::Node {

public: 
        Obstacle();

private:

        //functions
		void chattercallback(const sensor_msgs::msg::Range::SharedPtr msg);

        //members
	    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
    	rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub;
        float min_range;
        geometry_msgs::msg::Twist cmdvel;
	
};

// create one class (meaining .h & .cpp)
// create one file with main function
// instantiate the class
