#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


// 有问题... 
std::string child_frame_id, frame_id;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_listener_node");
    ros::NodeHandle nh;

    nh.getParam("/child_frame_id", child_frame_id);
    nh.getParam("/frame_id", frame_id);

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_out", 10);
    
    ros::Rate rate(25);  // Adjust the publishing rate as per your requirement
    
    while (ros::ok())
    {   
        // Get the latest transform from "map" to "base_link"
        geometry_msgs::TransformStamped transform_stamped;
        try
        {
            // transform_stamped = tf_buffer.lookupTransform(child_frame_id, frame_id, ros::Time(0)); 
            transform_stamped = tf_buffer.lookupTransform("map0", "base_link0", ros::Time(0)); 
        }
        catch (tf2::TransformException& e)
        {
            ROS_ERROR("Failed to get TF transformation: %s", e.what());
            // continue;
        }
        
        // Create a PoseStamped message
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header = transform_stamped.header;
        
        // Set the position and orientation from the transform
        pose_msg.pose.position.x = transform_stamped.transform.translation.x ;
        pose_msg.pose.position.y = transform_stamped.transform.translation.y ;
        pose_msg.pose.position.z = transform_stamped.transform.translation.z ;

        // pose_msg.pose.position = transform_stamped.transform.translation;
        pose_msg.pose.orientation = transform_stamped.transform.rotation;
        
        // Publish the PoseStamped message
        pose_pub.publish(pose_msg);

        rate.sleep();
    }
    
    return 0;
}
