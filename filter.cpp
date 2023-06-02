#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <queue>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

std::queue<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_queue;
int frames_to_merge;
float grid_length;
std::string input_topic, output_topic;

ros::Publisher pub;
tf2_ros::Buffer tf_buffer;
tf2_ros::TransformListener* tf_listener;

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  clouds_queue.push(cloud);

  // If the queue is too big, remove the oldest element
  while(clouds_queue.size() > frames_to_merge)
  {
    clouds_queue.pop();
  }

  // Merge all clouds in the queue
  pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::queue<pcl::PointCloud<pcl::PointXYZ>::Ptr> temp_queue = clouds_queue;  // Create a copy of the queue to iterate over
  while(!temp_queue.empty())
  {
    *merged_cloud += *temp_queue.front();
    temp_queue.pop();
  }

  // Get the latest transform--Directly copy subscribed tf information 
  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tf_buffer.lookupTransform(cloud_msg->header.frame_id, cloud_msg->header.frame_id,
                                          cloud_msg->header.stamp, ros::Duration(1.0));
  }
  catch(const tf2::TransformException& ex)
  {
    ROS_WARN_STREAM("Failed to lookup transform: " << ex.what());
    return;
  }

  // Data Filter by VoxelGridFilter
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud(merged_cloud);
  vg.setLeafSize(grid_length, grid_length, grid_length);
  vg.filter(*cloud_filtered);

  // Publish merged point cloud with original tf
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud_filtered, output);
  output.header = cloud_msg->header;
  pub.publish(output);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "node_filter");
  ros::NodeHandle nh;

  // Get the parameters from the parameter server
  nh.param("frames_to_merge", frames_to_merge, 10);
  nh.param("grid_length", grid_length, 0.01f);
  nh.param<std::string>("input_topic", input_topic, "livox/lidar");
  nh.param<std::string>("output_topic", output_topic, "cloud_in");

  tf_listener = new tf2_ros::TransformListener(tf_buffer);

  ros::Subscriber sub = nh.subscribe(input_topic, frames_to_merge, cloud_callback);
  pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 1);
  ROS_INFO("Change Frequency is %d", frames_to_merge);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  delete tf_listener;

  return 0;
}
