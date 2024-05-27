#include <csignal>

#include <sensor_msgs/PointCloud2.h> 
#include <geometry_msgs/PoseStamped.h> 
#include <std_msgs/Empty.h> 

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

class TestSendPoseCloud
{
public:

  void init(ros::NodeHandle &nh, ros::NodeHandle &pnh)
  {
    pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/grid_map/pose", 10);
    cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/grid_map/cloud", 10);
    pub_pose_timer_ = nh.createTimer(ros::Duration(0.025), &TestSendPoseCloud::pubPoseTimerCB, this);
    pub_cloud_timer_ = nh.createTimer(ros::Duration(0.025), &TestSendPoseCloud::pubCloudTimerCB, this);
  }

  void pubPoseTimerCB(const ros::TimerEvent & /*event*/)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
    pose.pose.position.x = 0.0; 
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.0;

    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    pose_pub_.publish(pose);
  }

  void pubCloudTimerCB(const ros::TimerEvent & /*event*/)
  {
    sensor_msgs::PointCloud2 cloud_msg;


    // create point cloud object
    pcl::PointCloud<pcl::PointXYZ> myCloud;

    // fill cloud with random points
    for (int v=0; v<100; ++v)
    {
        pcl::PointXYZ newPoint;
        newPoint.x = (rand() * 1.0) / RAND_MAX;
        newPoint.y = (rand() * 1.0) / RAND_MAX;
        newPoint.z = (rand() * 1.0) / RAND_MAX;
        myCloud.points.push_back(newPoint);
    }

    pcl::toROSMsg(myCloud, cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "map";

    cloud_pub_.publish(cloud_msg);
  }

private:
  std::string node_name_{"test_send_pose_cloud"};

  /* Subscribers */
  ros::Publisher pose_pub_;
  ros::Publisher cloud_pub_;

  /* Timer */
  ros::Timer pub_pose_timer_;
  ros::Timer pub_cloud_timer_;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "testsendposecloud");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  TestSendPoseCloud testsendposecloud;
  testsendposecloud.init(nh, pnh);

  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();

  return 0;
}
