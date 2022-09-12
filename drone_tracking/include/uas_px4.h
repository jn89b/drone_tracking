#ifndef NULL_UAS_PX4_H
#define UAS_PX4_H

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

class PX4Drone
{
    private:
        ros::NodeHandle nh;

        ros::Publisher local_pos_pub, vel_pub, cmd_raw, att_pub;

        ros::Subscriber state_sub, target_found_sub, 
            rtag_ekf_sub,rtag_quad_sub, quad_odom_sub;

        ros::Subscriber rtag_ekf_vel_sub, land_permit_sub,
             true_quad_odom_sub,service_input_sub, lqr_gain_sub;
        
        //mavros service clients for arming and setting modes
        ros::ServiceClient arming_client, set_mode_client;

        geometry_msgs::PoseStamped pose;
        geometry_msgs::TwistStamped cmd_vel;   
                  
        
        Eigen::Vector4d lqr_gain_x;
        Eigen::Vector4d lqr_gain_y;

        float true_odom_z; // this is the true odometry 
    
        float pre_error_x;
        float pre_error_y;

        float pre_ierror_x;
        float pre_ierror_y;

        //initial pose commands
        std::vector<float> init_pos;

        //offsets
        std::vector<float> _offset_pos;

        void lqr_precland(float z_val);

    public:
    //Odometry of quad with offset because Airsim does not like to play nice
        std::vector<float> odom;
        std::vector<float> true_pos;
        std::vector<float> kf_tag;
        std::vector<float> rtag;

        std::vector<float> vel;
        std::vector<float> kf_vel;
        
        mavros_msgs::SetMode set_mode;
        mavros_msgs::CommandBool arm_cmd;
        mavros_msgs::State current_state;
        
        PX4Drone(ros::NodeHandle* nh, std::vector<float> offset_pos);
        void init_vals(std::vector<float> offset_pos);
        
        //call back functions
        void state_cb(const mavros_msgs::State::ConstPtr& msg);
        void quad_odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
        void true_odom_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
        
        //drone stuff 
        void setmode_arm(ros::Time last_request,
            const std::string& mode_input, mavros_msgs::CommandBool arm_cmd);
        void send_init_cmds(std::vector<float> position, ros::Rate rate);
        void send_global_waypoints(std::vector<float> wp_vector);
        void get_true_pos();

        //tracking with PID
        void go_follow(Eigen::Vector2d gain, float z_cmd);
        void rtagquad_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void kftag_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void kftag_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);

        //lqr tracking
        void lqr_cb(const drone_tracking::LQRGain::ConstPtr& msg);
        void lqr_track();

        //lqr landing
        void lqr_land(float z_drop, ros::Rate rate);

        void user_cmd_cb(const std_msgs::Int8::ConstPtr& msg);
        void send_yaw_cmd(Eigen::Vector2d gain, float z_cmd, float yaw);
        
        void begin_land_protocol(Eigen::Vector2d gain, ros::Rate rate);
        void set_offboard(std::vector<float> pos_cmd,  ros::Rate rate);

        void send_velocity_cmd(Eigen::Vector2d gain);

        void send_att_cmd(float pitch, float roll, float yaw);

};

#endif