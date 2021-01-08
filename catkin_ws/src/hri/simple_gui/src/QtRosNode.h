#pragma once
#include <iostream>
#include <cmath>
#include <QThread>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"

class QtRosNode : public QThread
{
Q_OBJECT
public:
    QtRosNode();
    ~QtRosNode();

    ros::NodeHandle* n;
    ros::Publisher        pubCmdVel;
    ros::Publisher        pubFlipperFL;
    ros::Publisher        pubFlipperFR;
    ros::Publisher        pubFlipperBL;
    ros::Publisher        pubFlipperBR;
    tf::TransformListener tf_listener;
    
    geometry_msgs::Twist cmd_vel;
    bool publishing_cmd_vel;
    bool gui_closed;
    
    void run();
    void setNodeHandle(ros::NodeHandle* nh);

    void publish_cmd_vel(float linear_frontal, float linear_lateral, float angular);
    void start_publishing_cmd_vel(float linear_frontal, float linear_lateral, float angular);
    void stop_publishing_cmd_vel();

    void get_robot_pose(float& robot_x, float& robot_y, float& robot_a);

    void publish_flipper_fl(double d);
    void publish_flipper_fr(double d);
    void publish_flipper_bl(double d);
    void publish_flipper_br(double d);
    void publish_flipper_predef_position(std::string position);
    
signals:
    void updateGraphics();
    void onRosNodeFinished();
    
};
