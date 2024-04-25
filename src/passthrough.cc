#include <iostream>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_z(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_voxel(new pcl::PointCloud<pcl::PointXYZ>);
ros::Publisher pub;
double z_filter_min=-0.5;
double z_filter_max=0.5;
std::deque<sensor_msgs::PointCloud2ConstPtr> point_cloud_buffer;
double t_buffer=1.0;

void processPointCloudBuffer()
{
    if (!point_cloud_buffer.empty())
    {
        // Get the latest point cloud message from the buffer
        sensor_msgs::PointCloud2ConstPtr latest_msg = point_cloud_buffer.back();

        // Convert ROS point cloud message to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*latest_msg, *cloud);

        // Create the filtering object
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(z_filter_min, z_filter_max);
        pass.filter(*cloud_filtered_z);

        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(cloud_filtered_z);
        voxel_grid.setLeafSize(0.05, 0.05, 0.05); // Adjust leaf size according to your density requirement
        voxel_grid.filter(*cloud_filtered_voxel);

        // Convert filtered point cloud back to ROS message
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud_filtered_voxel, output);

        // Publish filtered point cloud
        pub.publish(output);
    }
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Store the incoming point cloud message in the buffer
    point_cloud_buffer.push_back(input);

    // Process the point cloud buffer after the specified buffer time
    ros::Duration(t_buffer).sleep(); // Sleep for t_buffer duration
    processPointCloudBuffer();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "passthrough_node");
    ros::NodeHandle nh;

    // Subscribe to ORB-SLAM3 point cloud topic
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/orb_slam3/all_points", 1, cloudCallback);

    // Advertise a topic to publish the filtered point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_point_cloud", 1);

    ros::spin();

    return 0;
}
