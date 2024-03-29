#include<ros/ros.h>
#include<tf/transform_listener.h>
#include<geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

std::string child_frame_id, frame_id;
int main(int argc, char *argv[])
{
    //初始化节点
    ros::init(argc, argv, "k_tf_listener");
    
    ros::NodeHandle nh;
    nh.getParam("/child_frame_id", child_frame_id);
    nh.getParam("/frame_id", frame_id);

    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_out", 10);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom_out", 10);

    tf::TransformListener listener;

    ros::Rate rate(30.0);
    
    while (nh.ok())
    {

       tf::StampedTransform transform;
       try
       {
           // 查找turtle2与turtle1的坐标变换
            listener.waitForTransform(frame_id, child_frame_id, ros::Time(0), ros::Duration(1.0));
            listener.lookupTransform(frame_id, child_frame_id, ros::Time(0), transform);
       }
       catch (tf::TransformException &ex)
       {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
       }

        // Create a PoseStamped message
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.frame_id = child_frame_id;
        pose_msg.header.stamp = ros::Time::now();
        // Set the position and orientation from the transform
        pose_msg.pose.position.x = transform.getOrigin().x();
        pose_msg.pose.position.y = transform.getOrigin().y();
        pose_msg.pose.position.z = transform.getOrigin().z();

        // pose_msg.pose.position = transform_stamped.transform.translation;
        pose_msg.pose.orientation.w = transform.getRotation().getW();
        pose_msg.pose.orientation.x = transform.getRotation().getX();
        pose_msg.pose.orientation.y = transform.getRotation().getY();
        pose_msg.pose.orientation.z = transform.getRotation().getZ();
        
        // Publish the PoseStamped message
        pose_pub.publish(pose_msg);

        nav_msgs::Odometry odom_msg;
        odom_msg.header.frame_id = child_frame_id;
        odom_msg.header.stamp = ros::Time::now();
        // Set the position and orientation from the transform
        odom_msg.pose.pose.position.x = transform.getOrigin().x();
        odom_msg.pose.pose.position.y = transform.getOrigin().y();
        odom_msg.pose.pose.position.z = transform.getOrigin().z();

        // pose_msg.pose.position = transform_stamped.transform.translation;
        odom_msg.pose.pose.orientation.w = transform.getRotation().getW();
        odom_msg.pose.pose.orientation.x = transform.getRotation().getX();
        odom_msg.pose.pose.orientation.y = transform.getRotation().getY();
        odom_msg.pose.pose.orientation.z = transform.getRotation().getZ();
        
        // Publish the PoseStamped message
        odom_pub.publish(odom_msg);

        rate.sleep();
       
    }
    
    

    return 0;
}
