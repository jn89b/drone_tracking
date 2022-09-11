#include <iostream>
#include <ros/ros.h>
#include <uas_px4.h>
#include <stdlib.h>    
#include <PID.h>
#include <vector>
#include <tf/tf.h>
#include <string>
#include <math.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace Eigen;

std::vector<float> get_offset_pos(ros::NodeHandle* nh)
{
    float offset_x;
    float offset_y; 

    std::vector<float> offset_pos;    
    nh->getParam("uav0/offboard_test/offset_x", offset_x);
    nh->getParam("uav0/offboard_test/offset_y", offset_y);
    offset_pos.push_back(0);
    offset_pos.push_back(-10.0);

    return offset_pos;
}

double calc_heading(std::vector<float> some_vec)
{   
    tf::Quaternion q(
    some_vec[3],
    some_vec[4],
    some_vec[5],
    some_vec[6]);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    //for wrapping of angles
    if( yaw <= 0) yaw += 2*M_PI;
    //return atan2(some_vec[1],some_vec[0]);
    return yaw;
}

// void websling_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
// {
//     websling[0] = msg->pose.position.x;
//     websling[1] = msg->pose.position.y;
    
// }

int main(int argc, char **argv)
{

    const float kp = 0.30; //0.35
    const float ki = 0.00001;
    const float kd = 0.00001; //0.00001
    const float dt = 0.1;
    const float gain_tol = 0.075;
    const float heading_bound = M_PI/6; //45 degrees
    const float rate_val = 20.0;    

    ros::init(argc, argv, "test_px4_header");
    ros::NodeHandle _nh; // create a node handle; need to pass this to the class constructor
    ros::Rate rate(rate_val);

    std::vector<float> offset_pos = get_offset_pos(&_nh);
    
    PX4Drone px4drone(&_nh, offset_pos);
    double drone_yaw = 0.0;
    double at_yaw = 0.0;

    //this should be from parameters
    std::vector<float> init_pos = {5.0, 5.0, 30.0};
    px4drone.send_init_cmds(init_pos, rate);
    px4drone.set_mode.request.custom_mode = "AUTO.LAND";
    px4drone.arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        //px4drone.setmode_arm(last_request, px4drone.set_mode.request.custom_mode , px4drone.arm_cmd);
        //px4drone.send_global_waypoints(init_pos);

        PID pid_x(kp, ki, kd, dt, px4drone.kf_tag[0], px4drone.odom[0]);
        PID pid_y(kp, ki, kd, dt, px4drone.kf_tag[1], px4drone.odom[1]);

        PID pid_yaw(kp, ki, kd, dt, at_yaw, drone_yaw);

        switch(px4drone.user_cmd)
        {
            case 0:
            {   
                if (px4drone.current_state.mode == "OFFBOARD"){
                    px4drone.send_global_waypoints(init_pos);
                }
                else{
                    px4drone.set_offboard(init_pos, rate);
                }
                
                break;
            }
            case 1: //TRACKING WITH LQR for now
            {
                px4drone.lqr_track();
                break;
            }
            case 2: // Precision Land with LQR
            {
                px4drone.lqr_land(-0.5, rate);
                break;
            }
            // case 2: // TRACKING WITH pid
            // {   
            //     float p_x = pid_x.getPID();
            //     float p_y = pid_y.getPID();

            //     float p_yaw = pid_yaw.getPID();
            //     Eigen::Vector2d gain(p_x, p_y);
            //     Eigen::Vector2d no_gain(0, 0);
 
            //     {
            //         if ((abs(gain[0])<= gain_tol) && (abs(gain[1]) <= gain_tol)){
            //             std::cout<<"good enough"<<gain<<std::endl;
            //             px4drone.send_yaw_cmd(no_gain, init_pos[2], drone_yaw);
            //             // px4drone.send_velocity_cmd(no_gain);    
            //         } 
                    
            //         else if((abs(gain[0])>= gain_tol) && (abs(gain[1]) <= gain_tol)){
            //             Eigen::Vector2d x_gain(p_x, 0);
            //             px4drone.send_yaw_cmd(x_gain, init_pos[2], drone_yaw);
            //             // px4drone.send_velocity_cmd(no_gain);    
            //         } 
                    
            //         else if((abs(gain[1])>= gain_tol) && (abs(gain[0]) <= gain_tol)){
            //             Eigen::Vector2d y_gain(0, p_x);
            //             px4drone.send_yaw_cmd(y_gain, init_pos[2], drone_yaw);
            //             // px4drone.send_velocity_cmd(no_gain);    
            //         } 
                    
            //         else{
            //             px4drone.send_yaw_cmd(gain, init_pos[2], drone_yaw);
            //             // px4drone.send_velocity_cmd(vel_gain);
            //         }
            //     }   
            //     break;
            // }
            // case 2: //precland
            // {
            //     float p_x = pid_x.getPID();
            //     float p_y = pid_y.getPID();
            //     float p_yaw = pid_yaw.getPID();
            //     Eigen::Vector2d gain(p_x, p_y);
            //     px4drone.begin_land_protocol(gain, rate);
            //     //ROS_INFO("returning from function");
            //     break;
            // }
            default:
            {
                if (px4drone.current_state.mode == "OFFBOARD"){
                    px4drone.send_global_waypoints(init_pos);
                }
                else{
                    px4drone.set_offboard(init_pos, rate);
                }
            }
        }
        //std::cout<<"outside"<<std::endl;
        ros::spinOnce();
        rate.sleep();
    }

    //ros::spin();

    return 0; 
}