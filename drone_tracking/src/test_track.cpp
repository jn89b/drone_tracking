#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <PID.h>

using std::string;

class TestTrack
{
   private:
        ros::NodeHandle nh;
        ros::Subscriber rtag_ekf_sub, rtag_quad_sub, quad_odom_sub;
        
        //Position vectors 
        float r_x; // x tag wrt quad 
        float r_y; // y tag wrt quad
        float r_z; // z tag wrt quad -> Don't really need this can use actual quad to true ground
        // Quaternion orientation vectors
        float r_qx;
        float r_qy;                 
        float r_qz;
        float r_qw;

        //kalman filter estimate
        float kf_x;
        float kf_y;

        //Odometry of quad with offset because Airsim does not like to play nice
        float odom_x;
        float odom_y;
        float odom_z;

    public:
        TestTrack()
        {
            rtag_quad_sub = nh.subscribe<geometry_msgs::PoseStamped>
                ("tag/pose", 10, &TestTrack::rtagquad_cb,this);
            rtag_ekf_sub = nh.subscribe<geometry_msgs::PoseStamped>
                    ("kf_tag/pose", 10, &TestTrack::kftag_cb,this);     
            quad_odom_sub = nh.subscribe<geometry_msgs::PoseStamped>
                ("mavros/offset_local_position/pose",15, &TestTrack::quad_odom_callback, this);

            ros::Rate rate(20);
            init_vals();

            while(ros::ok())
            {
                request_gains();
                ros::spinOnce();
                rate.sleep();
            }
        }

    void init_vals()
    {
        r_x = 0.0;
        r_y = 0.0;
        r_z = 0.0;
        r_qx = 0.0;
        r_qy = 0.0;
        r_qz = 0.0;
        r_qw = 0.0;  

        //kalman filter estimate
        kf_x = 0.0;
        kf_y = 0.0;

        //Odometry of quad with offset because Airsim does not like to play nice
        odom_x = 0.0;     
    }

    void request_gains()
    {   
        float kp = 0.8;
        float ki = 0.0;
        float kd = 0.0;
        float dt = 0.1;
        std::cout<<"kf_x: "<< kf_x << std::endl;
        PID pid_x(kp, ki, kd, dt, kf_x, odom_x);
        std::cout<<"PID: "<< pid_x.getPID() << std::endl;
    }

    void rtagquad_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        r_z = msg->pose.position.x;
        r_y = msg->pose.position.y;
        r_z = msg->pose.position.z;

        r_qx = msg->pose.orientation.x;
        r_qy = msg->pose.orientation.y;
        r_qz = msg->pose.orientation.z;
        r_qw = msg->pose.orientation.w;
    }

    void kftag_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        kf_x = msg->pose.position.x;
        kf_y = msg->pose.position.y;

    }

    void quad_odom_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        odom_x = msg->pose.position.x;
        odom_y = msg->pose.position.y;
        odom_z = msg->pose.position.z;
        //ROS_INFO("odom_x: %f, odom_y:", odom_x, odom_y); 
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_track");
    ROS_INFO("starting");
    TestTrack testtrack;


    return 0; 
}

