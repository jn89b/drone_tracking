#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <moving_avg.h>

float kf_x;
float kf_y;
float tol;
bool target_found;

std_msgs::Bool stabilize;


void init_vals()
{
    kf_x = 0.0;
    kf_y = 0.0;
    tol = 0.10; 
    target_found = false;
}

void kftag_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
   kf_x = msg->pose.position.x;
   kf_y = msg->pose.position.y;
}


void targetfound_cb(const std_msgs::Bool::ConstPtr& msg)
{
    target_found = msg->data;
}

int main(int argc, char **argv)
{   
    ros::init(argc,argv,"moving_avg_main");
    ros::NodeHandle nh;
     
    init_vals();    

    ros::Subscriber rtag_ekf_sub = nh.subscribe<geometry_msgs::PoseStamped>
                    ("kf_tag/pose", 10, &kftag_cb);
    
    ros::Publisher moving_avg_pub = nh.advertise<std_msgs::Bool>
                    ("stabilize_tag", 10);

    ros::Subscriber target_found_sub = nh.subscribe<std_msgs::Bool>
                    ("target_found", 10, &targetfound_cb);
                    
    
    ros::Rate rate(20.0);
    
    while (ros::ok()){

        if (target_found == true){ 
            MovingAverage moving_avg(kf_x, kf_y);
            float kf_avg_x = moving_avg.compute_avg(kf_x);
            float kf_avg_y = moving_avg.compute_avg(kf_y);
            float _avg_mag = sqrt(pow(kf_avg_x,2) + pow(kf_avg_y,2));
            std::cout<<"mag: " << _avg_mag << std::endl;

            if ((_avg_mag < tol)){
                stabilize.data = true;
                moving_avg_pub.publish(stabilize);
            } 
            else{
                stabilize.data = false;
                moving_avg_pub.publish(stabilize);
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;   
}



