#include "ros/ros.h"
#include <std_msgs/Int8.h>

int user_input;

void usercontrol_cb(const std_msgs::Int8::ConstPtr& msg)
{   
    user_input = msg->data;
    std::cout<<"user_input: "<<user_input<<std::endl;
    if (user_input == NULL)
    {
        ROS_INFO("i hear nothing");
    }
}

int main(int argc, char **argv)
{   
    ros::init(argc,argv,"test_user");
    ros::NodeHandle nh;
        
    ros::Subscriber user_input_sub = nh.subscribe<std_msgs::Int8>
                    ("user_control", 10, &usercontrol_cb);
    
    ros::Rate rate(20.0);
    
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;   
}



