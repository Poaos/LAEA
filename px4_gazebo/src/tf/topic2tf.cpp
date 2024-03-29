#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


std::string child_frame_id, frame_id;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // Create a TransformStamped message
    geometry_msgs::TransformStamped transform_msg;
    
    // Set the header frame IDs and timestamp
    transform_msg.header = msg->header;
    transform_msg.child_frame_id = child_frame_id;  // The child frame
    transform_msg.header.frame_id = frame_id;  // The parent frame
    // transform_msg.child_frame_id = "base_link";  // The child frame
    // transform_msg.header.frame_id = "map";  // The parent frame
    
    // Set the transformation translation
    transform_msg.transform.translation.x = msg->pose.position.x;
    transform_msg.transform.translation.y = msg->pose.position.y;
    transform_msg.transform.translation.z = msg->pose.position.z;
    
    // Set the transformation rotation (quaternion)
    transform_msg.transform.rotation = msg->pose.orientation;
    
    // Create a TransformBroadcaster and publish the transformation
    static tf2_ros::TransformBroadcaster tf_broadcaster;
    tf_broadcaster.sendTransform(transform_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_publisher_node");
    ros::NodeHandle nh("~");

    nh.getParam("/child_frame_id", child_frame_id);
    nh.getParam("/frame_id", frame_id);

    
    ros::Rate rate(40);  // Adjust the publishing rate as per your requirement

    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("pose_in", 10, poseCallback);
    
    ros::spin();
    
    return 0;
}
