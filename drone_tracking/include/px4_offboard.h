#ifndef NULL_PX4_OFFBOARD_H
#define PX4_OFFBOARD_H

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <math.h>  
#include <tf/tf.h>
#include <drone_tracking/LQRGain.h>


class PX4Offboard
{
    private:
        ros::NodeHandle nh;
        ros::Publisher local_pos_pub, vel_pub, cmd_raw, att_pub;

        ros::Subscriber state_sub, quad_odom_sub;
        ros::Subscriber rtag_quad_sub, rtag_ekf_sub;
        ros::Subscriber service_input_sub;


    public:

        //constructor
        PX4Offboard(ros::NodeHandle* nh);

        //attributes, need to see which ones should be private
        ros::ServiceClient arming_client, set_mode_client;
        geometry_msgs::PoseStamped pose;
        mavros_msgs::SetMode set_mode;
        mavros_msgs::CommandBool arm_cmd;
        mavros_msgs::State current_state;

        int user_cmd;   

        std::vector<float> odom;
        std::vector<float> kf_tag;

        void init_vals();

        //callbacks
        void state_cb(const mavros_msgs::State::ConstPtr& msg);
        void quad_odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
        void kftag_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void user_cmd_cb(const std_msgs::Int8::ConstPtr& msg);

        //commands
        void setmode_arm(ros::Time last_request,const std::string& mode_input, 
                            mavros_msgs::CommandBool arm_cmd);

        void send_init_cmds(std::vector<float> position, ros::Rate rate);
        void send_global_waypoints(std::vector<float> wp_vector);
        void set_offboard(std::vector<float> pos_cmd,  ros::Rate rate);
        void begin_land_protocol(Eigen::Vector2d gain, ros::Rate rate,
                             float land_height, float dropping); 


};



#endif
