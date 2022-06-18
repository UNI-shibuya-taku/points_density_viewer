#ifndef __POINTS_DENSITY_VIEWER
#define __POINTS_DENSITY_VIEWER

#include <ros/ros.h>
#include <sensor_msgs/point_cloud_conversion.h>
// PCL
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <vector>
#include <iostream>
#include <fstream>
class PointsDensityViewer
{
public:
    typedef pcl::PointXYZI PointXYZI;
    typedef pcl::PointCloud<PointXYZI> CloudXYZI;
    typedef pcl::PointCloud<PointXYZI>::Ptr CloudXYZIPtr;

    typedef pcl::PointXYZINormal PointXYZIN;
    typedef pcl::PointCloud<PointXYZIN> CloudXYZIN;
    typedef pcl::PointCloud<PointXYZIN>::Ptr CloudXYZINPtr;
    
    typedef pcl::PointXYZRGB PointXYZRGB;
    typedef pcl::PointCloud<PointXYZRGB> CloudXYZRGB;
    typedef pcl::PointCloud<PointXYZRGB>::Ptr CloudXYZRGBPtr;

    typedef pcl::PointXYZHSV PointXYZHSV;
    typedef pcl::PointCloud<PointXYZHSV> CloudXYZHSV;
    typedef pcl::PointCloud<PointXYZHSV>::Ptr CloudXYZHSVPtr; 
    PointsDensityViewer(void);
    void process(void);
    // callback
    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr&);
    double get_distance(PointXYZRGB pt)
    {
        double distance = sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
        return distance;
    }
private:
    class GridPoint{
        public:
            GridPoint(void);
            PointXYZI stored_point;
            // CloudXYZPtr dense_cloud{new CloudXYZ};
            std::vector<int> point_indices;
            double density;
        private:
    };
    int voxel_num_x, voxel_num_y, voxel_num_z;
    int resolution_x, resolution_y, resolution_z;
    int max;
    int min;
    double width;
    double height;
    double min_height;
    double voxel_size_x, voxel_size_y, voxel_size_z;
    double width_x_2;
    double width_y_2;
    double grid_volume;
    ros::NodeHandle nh;
    ros::NodeHandle local_nh;
    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub;
    ros::Publisher check_pub;
    CloudXYZINPtr cloud_ptr{new CloudXYZIN}; 
};
#endif