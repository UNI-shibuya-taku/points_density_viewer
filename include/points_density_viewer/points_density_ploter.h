#ifndef __POINTS_DENSITY_PLOTER
#define __POINTS_DENSITY_PLOTER

#include <ros/ros.h>
#include <sensor_msgs/point_cloud_conversion.h>
// PCL
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <vector>
#include <iostream>
#include <fstream>
class PointsDensityPloter
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
        PointsDensityPloter(void);
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
        int max;
        int min;
        int all_count = 0;
        int resolution_x, resolution_y, resolution_z;
        double x_left, x_right;
		double y_left, y_right;
		double z_upper, z_lower;
        double width;
        double height;
        double min_height;
        double width_x_2;
        double width_y_2;
        double density_r;
        double density_g;
        double density_b;
        double volume_of_target_area;
        std::ofstream ofs;
		std::string csv_dir;
        ros::NodeHandle nh;
        ros::NodeHandle local_nh;
        ros::Subscriber cloud_sub;
        ros::Subscriber cloud_compare_sub;
        ros::Publisher cloud_pub;
        ros::Publisher check_pub;
        CloudXYZINPtr cloud_ptr{new CloudXYZIN}; 
};
#endif