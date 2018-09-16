/* Compute the odometry using images from a ROS topic */

#include "VisualOdometry/VisualOdometry.h"

#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>


VisualOdometry odom;

Mat rgb_frame;
bool first_frame = true;

clock_t c_start = clock();
clock_t c_end = clock();

ros::Publisher odom_pub;

void distance_callback(const std_msgs::Float64::ConstPtr& msg)
{
   double wall_distance = 1000 * msg->data;
   cout << "Wall distance: " << wall_distance << endl;

   odom.set_wall_distance(wall_distance);
}


//void publish_odom(const ros::TimerEvent&)
void publish_odom()
{
    double x = odom.get_x() / (3.5 * 100);
    double y = odom.get_y() / (3.5 * 100);
    double z = odom.get_z() / (3.5 * 100);
    double yaw = odom.get_yaw();

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    // Set position
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;

    // Set orientation (in quaternion...)
    tf::Quaternion q;
    q.setRPY(0, 0, yaw);
    tf::quaternionTFToMsg(q, pose.pose.orientation);

    // Publish msg
    odom_pub.publish(pose);
}

void camera_callback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& left_info)
{
    rgb_frame = cv_bridge::toCvShare(image_msg, "bgr8")->image;
    //cout << "rgb size: " << frame.size() << endl;
    // cout << "frame type: " << rgb_frame.type() << endl;
    // cout << "frame dims: " << rgb_frame.dims << endl;
    if (!rgb_frame.data)
    {
        cout << "Empty frame" << endl;
        return;
    }
    resize(rgb_frame, rgb_frame, Size(960, 540));
    c_end = clock();
    if (first_frame)
    {
        odom.init_frame(rgb_frame);
        first_frame = false;
    }
    else
    {
        odom.compute_next(rgb_frame);
    }
    
    double elapsed_secs = double(c_end - c_start) / CLOCKS_PER_SEC;
    // cout << "Frame time: " << elapsed_secs << endl;
    c_start = clock();
    // imshow ("RGB", rgb_frame);

    publish_odom();
}



int main(int argc, char **argv)
{
    ros::init (argc, argv, "drone_visual_odometry");
    ros::NodeHandle public_nh, private_nh("~");
    image_transport::ImageTransport it(public_nh);

    // namedWindow("RGB", CV_WINDOW_AUTOSIZE);
    // startWindowThread();

    // Parameters
    string camera_topic;
    private_nh.param<string>("camera_topic", camera_topic, "webcam/image");
    string altitude_topic;
    private_nh.param<string>("altitude_topic", altitude_topic, "mavros/global_position/rel_alt");
    string pose_topic;
    private_nh.param<string>("pose_topic", pose_topic, "localization/visual_pose");

    // Publishers
    odom_pub = public_nh.advertise<geometry_msgs::PoseStamped>(pose_topic, 3);

    // Subscribers
    image_transport::CameraSubscriber cam_sub = it.subscribeCamera(camera_topic, 5, camera_callback);
    ros::Subscriber sub_distance = public_nh.subscribe("wall_distance", 2, distance_callback);

    ros::spin();
    destroyAllWindows();
    return 0;
}