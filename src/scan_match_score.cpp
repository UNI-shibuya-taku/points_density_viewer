#include <ros/ros.h>
#include <sensor_msgs/point_cloud_conversion.h>
// PCL
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <vector>
#include <iostream>
#include <fstream>

#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
// #include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_representation.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
class ScanMatchScore
{
public:
    typedef pcl::PointXYZI PointXYZI;
    typedef pcl::PointCloud<PointXYZI> CloudXYZI;
    typedef pcl::PointCloud<PointXYZI>::Ptr CloudXYZIPtr;

    typedef pcl::PointXYZINormal PointXYZIN;
    typedef pcl::PointCloud<PointXYZIN> CloudXYZIN;
    typedef pcl::PointCloud<PointXYZIN>::Ptr CloudXYZINPtr;

    ScanMatchScore(void);
    void process(void);
    // callback
    void cloud_source_callback(const sensor_msgs::PointCloud2ConstPtr&);
    void cloud_target_callback(const sensor_msgs::PointCloud2ConstPtr&);
    geometry_msgs::Quaternion quat_eigen_to_msg(Eigen::Quaternionf q)
    {
        geometry_msgs::Quaternion msg;
        msg.x = (double)q.x();
        msg.y = (double)q.y();
        msg.z = (double)q.z();
        msg.w = (double)q.w();

        return msg;
    }
    Eigen::Quaternionf quat_msg_to_eigen(geometry_msgs::Quaternion q_msg)
    {
        Eigen::Quaternionf q_eigen(
            (float)q_msg.w,
            (float)q_msg.x,
            (float)q_msg.y,
            (float)q_msg.z
        );
        q_eigen.normalize();
        return q_eigen;
    }
private:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    ros::Subscriber cloud_source_sub;
    ros::Subscriber cloud_target_sub;
    ros::Publisher cloud_pub;
    CloudXYZIPtr cloud_source_ptr{new CloudXYZI}; 
    CloudXYZIPtr cloud_target_ptr{new CloudXYZI}; 
    CloudXYZIPtr cloud_target_last_ptr{new CloudXYZI}; 
	CloudXYZIPtr pc_trans{new CloudXYZI};
    int max_iterations;
    bool first_callback_flag = false;
    bool source_cloud_flag = false;
    bool target_cloud_flag = false;
    double pc_range;
    double leafsize_source;
    double leafsize_target;
    double leafsize;
    double trans_epsilon;
    double matching_score_th;
    double correspondence_distance;
	geometry_msgs::PoseStamped pose_ekf;
    std::ofstream ofs;
    std::string csv_dir;
};

ScanMatchScore::ScanMatchScore(void) : private_nh("~")
{
	private_nh.param("leafsize_source", leafsize_source, 0.1);
	private_nh.param("pc_range", pc_range, 100.0);
	private_nh.param("trans_epsilon", trans_epsilon, 0.001);
	private_nh.param("max_iterations", max_iterations, 100);
	private_nh.param("correspondence_distance", correspondence_distance, 0.1);
	private_nh.param("matching_score_th", matching_score_th, 0.1);
    private_nh.param("csv_dir", csv_dir, {"/home/amsl/catkin_ws/src/points_density_viewer/test.csv"});
    // cloud_source_sub = nh.subscribe("/cloud/source", 1, &ScanMatchScore::cloud_source_callback, this);
    cloud_target_sub = nh.subscribe("/cloud/target", 1, &ScanMatchScore::cloud_target_callback, this);
    
    std::cout << "========set param========" << std::endl;
    ofs.open(csv_dir);
}
// 蓄積点群
void ScanMatchScore::cloud_source_callback(const sensor_msgs::PointCloud2ConstPtr& msg_source_cloud)
{
    pcl::fromROSMsg(*msg_source_cloud, *cloud_source_ptr);
    source_cloud_flag = true;
}

// 最新点群
void ScanMatchScore::cloud_target_callback(const sensor_msgs::PointCloud2ConstPtr& msg_target_cloud)
{
    std::cout << "========ScanMatchScore========" << std::endl;
    pcl::fromROSMsg(*msg_target_cloud, *cloud_target_ptr);
    int cloud_size = cloud_target_ptr->points.size();
    std::cout << "subscribe cloud size: " << cloud_size << std::endl;
    if(first_callback_flag){
        /*initialize*/
        pcl::IterativeClosestPointNonLinear<PointXYZI, PointXYZI> icp;
        /*drop out*/
        // if(cloud_target_ptr->points.empty()) return false;

        /*set parameters*/
        icp.setTransformationEpsilon(trans_epsilon);
        icp.setMaximumIterations(max_iterations); // 繰り返し最大数
        icp.setMaxCorrespondenceDistance(correspondence_distance); // matching距離閾値
        // icp.setPointRepresentation(pcl::make_shared<const MyPointRepresentation>(point_representation));
        
        /*set cloud*/
        icp.setInputSource(cloud_target_ptr);
        icp.setInputTarget(cloud_target_last_ptr);

        /*initial guess*/
        Eigen::Translation3f init_translation(
            (float)pose_ekf.pose.position.x,
            (float)pose_ekf.pose.position.y,
            (float)pose_ekf.pose.position.z
        );
        Eigen::AngleAxisf init_rotation(
            quat_msg_to_eigen(pose_ekf.pose.orientation)
        );
        std::cout << "init_translation = (" << init_translation.x() << ", " << init_translation.y() << ", " << init_translation.z() << ")" << std::endl; 
        std::cout << "init_rotation : (" << init_rotation.axis()(0) << ", " << init_rotation.axis()(1) << ", " << init_rotation.axis()(2) << "), " << init_rotation.angle() << " [rad]" << std::endl; 
        Eigen::Matrix4f init_guess = (init_translation*init_rotation).matrix();

        /*drop out*/
        // if(pc_now_filtered->points.size() > pc_map_filtered->points.size()){
        // 	pcl::transformPointCloud (*pc_now_filtered, *pc_now_filtered, init_guess);
        // 	*pc_map += *pc_now_filtered;
        // 	return false;
        // }

        /*align*/
        std::cout << "aligning ..." << std::endl;
        icp.align(*pc_trans, init_guess);
        std::cout << "DONE" << std::endl;

        /*drop out*/
        if(!icp.hasConverged())	{
            std::cout << "has not converged!!!" << std::endl;
            // return false;
        }
        /*print*/
        std::cout << "ICP has converged:" << icp.hasConverged ()  << std::endl << " score: " << icp.getFitnessScore () << std::endl;
        std::cout << "icp.getFinalTransformation()" << std::endl << icp.getFinalTransformation() << std::endl;
        std::cout << "init_guess" << std::endl << init_guess << std::endl;
        ofs << icp.hasConverged() << "," << icp.getFitnessScore() << std::endl;

        // if(icp.getFitnessScore() <= matching_score_th){
            /*input*/
            // Eigen::Matrix4f T = icp.getFinalTransformation();
            // Eigen::Matrix3f R = T.block(0, 0, 3, 3);
            // Eigen::Quaternionf q_rot(R);
            // q_rot.normalize();
            // pose_ndt.pose.position.x = T(0, 3);
            // pose_ndt.pose.position.y = T(1, 3);
            // pose_ndt.pose.position.z = T(2, 3);
            // pose_ndt.pose.orientation = quat_eigen_to_msg(q_rot);
            // publication();
        // }
        // std::cout << "transformation time [s] = " << ros::Time::now().toSec() - time_start << std::endl;
        *cloud_target_last_ptr = *cloud_target_ptr;
    }else{
        *cloud_target_last_ptr = *cloud_target_ptr;
        first_callback_flag = true;
    }
}
void ScanMatchScore::process()
{
    ros::spin();
    ofs.close();
    std::cout << "finish scan_match_score" << std::endl;
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan_match_score");
    ScanMatchScore scan_match_score;
    scan_match_score.process();
    return 0;
}