#include <iostream>
#include <ros/ros.h>
#include <px4_offboard.h>
#include <stdlib.h>    
#include <PID.h>
#include <vector>
#include <tf/tf.h>
#include <string>
#include <math.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace Eigen;


void set_offboard(class PX4Offboard)
{

    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_offboard_main");

    //map these to a yaml or config file or something
    //PID parameters 
    float kp;
    float ki;
    float kd;

    ros::NodeHandle _nh; // create a node handle; need to pass this to the class constructor    
    const float rate_val = 20.0;    
    
    _nh.getParam("kp", kp);
    _nh.getParam("ki", ki);
    _nh.getParam("kd", kd);

    const float dt = 1/rate_val;
    const float gain_tol = 0.05; //meters threshhold to stay in middle of tag
    const float drop_height = 1.0;
    const float drop_rate = 0.25;


    ros::Rate rate(rate_val);

    PX4Offboard px4drone(&_nh);

    std::vector<float> init_pos = {1.0, 1.0, 3.0};

    px4drone.send_init_cmds(init_pos, rate);
    px4drone.set_mode.request.custom_mode = "OFFBOARD";
    px4drone.arm_cmd.request.value = true;
    // px4drone.set_offboard(init_pos, rate);

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){

        PID pid_x(kp, ki, kd, dt, px4drone.kf_tag[0], px4drone.odom[0]);
        PID pid_y(kp, ki, kd, dt, px4drone.kf_tag[1], px4drone.odom[1]);
        
        //set initial offboard need to set this as case 
        if( px4drone.current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( px4drone.set_mode_client.call(px4drone.set_mode) &&
                px4drone.set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !px4drone.current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( px4drone.arming_client.call(px4drone.arm_cmd) &&
                    px4drone.arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        switch(px4drone.user_cmd)
        {   
            case 0: // normal position command
            {
                if (px4drone.current_state.mode == "OFFBOARD"){
                    px4drone.send_global_waypoints(init_pos, 1);
                }
                else{
                    px4drone.set_offboard(init_pos, rate);
                }
                
            }   
            break;

            case 1: //begin pid tracking
            {
                   
                float p_x = pid_x.getPID();
                float p_y = pid_y.getPID();
                
                std::cout<<"px" <<p_x << "py" <<p_y <<std::endl;

                Eigen::Vector2f gain(p_x, p_y);
                Eigen::Vector2f no_gain(0.0f, 0.0f);
                std::vector<float> adjusted_pos{0.0,0.0,0.0};
                
                float odom_z = 0.0;//px4drone.odom[2];

                {
                    //check gain tolerances on x and y
                    if ((abs(gain[0])<= gain_tol) && (abs(gain[1]) <= gain_tol)){
                        
                        ROS_INFO("good enough");
                        adjusted_pos[0] = no_gain[0];
                        adjusted_pos[1] = no_gain[1];
                        adjusted_pos[2] = odom_z; 
                        
                        // adjusted_pos= {no_gain[0], no_gain[1], odom_z};
                        px4drone.track(adjusted_pos, 8);
                        // px4drone.send_velocity_cmd(no_gain);    
                    } 
                    
                    else if((abs(gain[0])>= gain_tol) && (abs(gain[1]) <= gain_tol)){
                        
                        ROS_INFO("X");
                        adjusted_pos[0] = gain[0];
                        adjusted_pos[1] = no_gain[1];
                        adjusted_pos[2] = odom_z;
                        
                        // adjusted_pos= {p_x+gain[0], no_gain[1], odom_z};
                        px4drone.track(adjusted_pos, 8);
                        // px4drone.send_velocity_cmd(no_gain);    
                    } 
                    
                    else if((abs(gain[1])>= gain_tol) && (abs(gain[0]) <= gain_tol)){
                        
                        ROS_INFO("Y");
                        adjusted_pos[0] = no_gain[0];
                        adjusted_pos[1] = gain[1];
                        adjusted_pos[2] = odom_z;
                        // adjusted_pos = {no_gain[0], p_y+gain[1], odom_z};
                        px4drone.track(adjusted_pos, 8);
                        // px4drone.send_velocity_cmd(no_gain);    
                    } 
                    
                    else{
                        adjusted_pos[0] = gain[0];
                        adjusted_pos[1] = gain[1];
                        adjusted_pos[2] = odom_z;
                        std::cout<<adjusted_pos[0]<<std::endl;
                        px4drone.track(adjusted_pos, 8);
                    }
                }   
                break;
            }         
            
            case 2: //precland
            {
                float p_x = pid_x.getPID();
                float p_y = pid_y.getPID();
                // float p_yaw = pid_yaw.getPID();
                Eigen::Vector2d gain(p_x, p_y);
                px4drone.begin_land_protocol(gain, rate, drop_height, drop_rate);
                //ROS_INFO("returning from function");
                break;
            }

            case 3: 
            {
                px4drone.lqr_track();
                break;   
            }

            case 4:
            {
                px4drone.lqr_land(drop_height, drop_rate, rate);
                break;   
            }

            default: 
            {
                px4drone.send_global_waypoints(init_pos, 1);
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0; 
}