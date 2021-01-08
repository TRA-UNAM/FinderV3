#include "QtRosNode.h"

QtRosNode::QtRosNode()
{
    this->gui_closed = false;
    publishing_cmd_vel = false;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.x = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;
}

QtRosNode::~QtRosNode()
{
}

void QtRosNode::run()
{    
    ros::Rate loop(30);
    pubCmdVel    = n->advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    pubFlipperFL = n->advertise<std_msgs::Float64>("/finder_base/left_front_flipper_joint_position_controller/command" , 10);
    pubFlipperFR = n->advertise<std_msgs::Float64>("/finder_base/right_front_flipper_joint_position_controller/command", 10);
    pubFlipperBL = n->advertise<std_msgs::Float64>("/finder_base/left_back_flipper_joint_position_controller/command"  , 10);
    pubFlipperBR = n->advertise<std_msgs::Float64>("/finder_base/right_back_flipper_joint_position_controller/command" , 10);
    int pub_zero_counter = 5;
    while(ros::ok() && !this->gui_closed)
    {
        if(publishing_cmd_vel)
        {
            pubCmdVel.publish(cmd_vel);
            pub_zero_counter = 5;
        }
        else if(--pub_zero_counter > 0)
        {
            if(pub_zero_counter <= 0)
                pub_zero_counter = 0;
            pubCmdVel.publish(cmd_vel);
        }
        ros::spinOnce();
        emit updateGraphics();
        loop.sleep();
    }
    emit onRosNodeFinished();
}

void QtRosNode::setNodeHandle(ros::NodeHandle* nh)
{
    this->n = nh;
}

void QtRosNode::publish_cmd_vel(float linear_frontal, float linear_lateral, float angular)
{
    cmd_vel.linear.x = linear_frontal;
    cmd_vel.linear.y = linear_lateral;
    cmd_vel.angular.z = angular;
    pubCmdVel.publish(cmd_vel);
}

void QtRosNode::start_publishing_cmd_vel(float linear_frontal, float linear_lateral, float angular)
{
    cmd_vel.linear.x = linear_frontal;
    cmd_vel.linear.y = linear_lateral;
    cmd_vel.angular.z = angular;
    publishing_cmd_vel = true;
}

void QtRosNode::stop_publishing_cmd_vel()
{
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0;
    publishing_cmd_vel = false;
}

void QtRosNode::get_robot_pose(float& robot_x, float& robot_y, float& robot_a)
{
    tf::StampedTransform t;
    tf::Quaternion q;
    tf_listener.waitForTransform("map", "base_link", ros::Time::now(), ros::Duration(0.5));
    tf_listener.lookupTransform("map", "base_link", ros::Time(0), t);
    robot_x = t.getOrigin().x();
    robot_y = t.getOrigin().y();
    q = t.getRotation();
    robot_a = atan2(q.z(), q.w())*2;
}

void QtRosNode::publish_flipper_fl(double d)
{
    std_msgs::Float64 msg;
    msg.data = d;
    pubFlipperFL.publish(msg);
}

void QtRosNode::publish_flipper_fr(double d)
{
    std_msgs::Float64 msg;
    msg.data = d;
    pubFlipperFR.publish(msg);
}

void QtRosNode::publish_flipper_bl(double d)
{
    std_msgs::Float64 msg;
    msg.data = d;
    pubFlipperBL.publish(msg);
}

void QtRosNode::publish_flipper_br(double d)
{
    std_msgs::Float64 msg;
    msg.data = d;
    pubFlipperBR.publish(msg);
}

void QtRosNode::publish_flipper_predef_position(std::string position)
{
    std_msgs::Float64 msg;
    if(position == "Up")
        msg.data = -1.5708;
    else if(position == "Down")
        msg.data = 1.2;
    else if(position == "Home")
        msg.data = 0;
    pubFlipperFL.publish(msg);
    pubFlipperFR.publish(msg);
    pubFlipperBL.publish(msg);
    pubFlipperBR.publish(msg);
}
