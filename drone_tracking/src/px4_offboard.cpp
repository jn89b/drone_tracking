#include <px4_offboard.h>


PX4Offboard::PX4Offboard(ros::NodeHandle* nh)
{
//publishers
    local_pos_pub = nh->advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 30);
    
    vel_pub = nh->advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 30);

    cmd_raw = nh->advertise<mavros_msgs::AttitudeTarget>
            ("mavros/setpoint_raw/attitude", 30);

    att_pub = nh->advertise<mavros_msgs::AttitudeTarget>
            ("mavros/setpoint_raw/attitude", 30);

    //subscribers

    state_sub = nh->subscribe<mavros_msgs::State>
            ("mavros/state", 30, &PX4Offboard::state_cb, this);

    quad_odom_sub = nh->subscribe<nav_msgs::Odometry>
        ("mavros/odometry/in",30, &PX4Offboard::quad_odom_cb, this);

    rtag_ekf_sub = nh->subscribe<geometry_msgs::PoseStamped>
            ("kf_tag/pose", 30, &PX4Offboard::kftag_cb,this);
    
    //service input
    service_input_sub = nh->subscribe<std_msgs::Int8>
                    ("utm_control", 30, &PX4Offboard::user_cmd_cb,this);
    

    //services
    arming_client = nh->serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");
    set_mode_client = nh->serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode");

    init_vals();
}

void PX4Offboard::init_vals()
{   
    odom = {0,0,0,0,0,0,0};
    kf_tag = {0,0,0,0,0,0,0};
}

void PX4Offboard::send_init_cmds(std::vector<float> position, ros::Rate rate)
{
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    } 

    send_global_waypoints(position);
    
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
}

void PX4Offboard::send_global_waypoints(std::vector<float> wp_vector){
     //send initial points subtract the offset position from spawn
    pose.pose.position.x = wp_vector[0]; 
    pose.pose.position.y = wp_vector[1];
    pose.pose.position.z = wp_vector[2];
    local_pos_pub.publish(pose);   
}

void PX4Offboard::set_offboard(std::vector<float> pos_cmd,  ros::Rate rate)
{
    while ((ros::ok()) && (current_state.mode != "OFFBOARD"))
    {   
        ros::Time last_request = ros::Time::now();
        
        if (current_state.mode == "OFFBOARD")
        {
            std::cout<<"i am in offboard"<<std::endl;
            return;
        }        
        
        // send_init_cmds(pos_cmd, rate);
        set_mode.request.custom_mode = "OFFBOARD";
        arm_cmd.request.value = true;
        setmode_arm(last_request, set_mode.request.custom_mode , arm_cmd);
        send_global_waypoints(pos_cmd);
        ros::spinOnce();
        rate.sleep();
    }
}


void PX4Offboard::setmode_arm(ros::Time last_request,
const std::string& mode_input, mavros_msgs::CommandBool arm_cmd)
{

    if( current_state.mode != mode_input &&
        (ros::Time::now() - last_request > ros::Duration(7.5))){
        if( set_mode_client.call(set_mode) &&
            set_mode.response.mode_sent){
            ROS_INFO("Mode enabled");
        }
        last_request = ros::Time::now();
    } else {
        if( !current_state.armed &&
            (ros::Time::now() - last_request > ros::Duration(7.5))){
            if( arming_client.call(arm_cmd) &&
                arm_cmd.response.success){
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }
    }
}


//CALLBACKS
void PX4Offboard::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void PX4Offboard::quad_odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom[0] = msg->pose.pose.position.x;
    odom[1] = msg->pose.pose.position.y;
    odom[2] = msg->pose.pose.position.z;

    odom[3] = msg->pose.pose.orientation.x;
    odom[4] = msg->pose.pose.orientation.y;
    odom[5] = msg->pose.pose.orientation.z;
    odom[6] = msg->pose.pose.orientation.w;
}

void PX4Offboard::kftag_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    kf_tag[0] = msg->pose.position.x;
    kf_tag[1] = msg->pose.position.y;
    kf_tag[2] = msg->pose.position.z;
    kf_tag[3] = msg->pose.orientation.x;
    kf_tag[4] = msg->pose.orientation.y;
    kf_tag[5] = msg->pose.orientation.z;
    kf_tag[6] = msg->pose.orientation.w;
}

void PX4Offboard::user_cmd_cb(const std_msgs::Int8::ConstPtr& msg)
{   
    user_cmd = msg->data;
}

void PX4Offboard::begin_land_protocol(Eigen::Vector2d gain, ros::Rate rate,
                                    float land_height, float dropping)
{   
    // const float land_height = 1.5;
    // const float dropping = 0.5;
    std::vector<float> adjusted_pos{0.0,0.0,0.0};


    adjusted_pos[0] = odom[0] + gain[0];
    adjusted_pos[1] = odom[1] + gain[1];

    if (abs(odom[2])>= land_height){
        //std::cout<<"starting to land"<<std::endl;
        adjusted_pos[2] = odom[2]-dropping;
        send_global_waypoints(adjusted_pos);

    }
    else{
        ros::Time last_request = ros::Time::now();  
        set_mode.request.custom_mode = "AUTO.LAND";
        arm_cmd.request.value = false;
        while(ros::ok() && (current_state.mode != "AUTO.LAND")){
            adjusted_pos[2] = odom[2];
            send_global_waypoints(adjusted_pos);
            setmode_arm(last_request, set_mode.request.custom_mode , arm_cmd);
            ros::spinOnce();
            rate.sleep();
            if (user_cmd != 2) 
            {
                ROS_INFO("I hear a different command");
                return;
            }
        }
    }
}