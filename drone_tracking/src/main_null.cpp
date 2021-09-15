#include <iostream>
#include <ros/ros.h>
#include <null_test.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_null");
    ros::NodeHandle _nh; // create a node handle; need to pass this to the class constructor
    ros::Rate rate(20.0);

    TestNull testnull(&_nh, 0.0, 0.0);
    
    while(ros::ok()){
        testnull.publishVal();
        rate.sleep();
    }

    ros::spin();

    return 0; 
}