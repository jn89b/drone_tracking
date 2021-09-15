#ifndef NULL_TEST_H
#define TIME_H

//ros stuff
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

class TestNull
{
    private:
        //vals from subscribers
        float _kf_x; 
        float _kf_y;

        //vals to remember this is for testing
        float _val_to_remember;

        ros::NodeHandle _nh;
        ros::Subscriber _rtag_ekf_sub;
        ros::Publisher _test_pub;

    public:
        TestNull(ros::NodeHandle* nodehandle, float kf_x, float kf_y);

        //callback function
        void kftagCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
        
        //member methods
        void publishVal();

        //return values
        float getKFx(){return _kf_x;}
        float getKFy(){return _kf_y;}
};

#endif
