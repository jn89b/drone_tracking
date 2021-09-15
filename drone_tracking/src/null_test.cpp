#include <iostream>
#include <null_test.h>

TestNull::TestNull(ros::NodeHandle* nodehandle, float kf_x, float kf_y)
{
    _rtag_ekf_sub = _nh.subscribe<geometry_msgs::PoseStamped>
            ("kf_tag/pose", 10, &TestNull::kftagCb,this);

    _test_pub = _nh.advertise<geometry_msgs::PoseStamped>
                ("test_pub", 10);

    publishVal();

    //initiate any vaariables here
    _val_to_remember= 0.0;
}


void TestNull::publishVal()
{
    geometry_msgs::PoseStamped pose; 
    pose.pose.position.x = _kf_x;
    _test_pub.publish(pose);
    _val_to_remember += _kf_x;
    
    //float actual_val = *some_pointer;=
    std::cout<<"test_val: "<< _val_to_remember << std::endl;
    ros::spinOnce();
}


void TestNull::kftagCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    _kf_x = msg->pose.position.x;
    _kf_y = msg->pose.position.y;
    //std::cout << "val:" << _val_to_remember << std::endl;
}
