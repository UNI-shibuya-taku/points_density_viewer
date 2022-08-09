#include "points_density_viewer/points_density_ploter.h"
PointsDensityPloter::PointsDensityPloter(void) : local_nh("~")
{
    local_nh.param("width", width, {50.0}); // 測定範囲
    // infant: 3.5~-2.6 くらい SQ2 : 13~-0.8くらい
    local_nh.param("height", height, {20.0}); // 測定高さ範囲
    local_nh.param("min_height", min_height, {10.0}); // 測定高さ範囲
   	local_nh.param("csv_dir", csv_dir, {"/home/amsl/catkin_ws/src/points_density_viewer/csv/test.csv"}); 
    // local_nh.param("x_left", x_left, {40.0}); // 左下座標
    // local_nh.param("x_right", x_right, {40.0}); // 右下
    // local_nh.param("y_left", y_left, {40.0}); // 左上座標
    // local_nh.param("y_right", y_right, {40.0}); // 右上
    // local_nh.param("z_upper", z_upper, {40.0}); // 高さ
    // local_nh.param("z_lower", z_lower, {1.0}); // 下座標
  
    cloud_sub = nh.subscribe("/cloud/lcl", 1, &PointsDensityPloter::cloud_callback, this); // sq_lidar 1scan
    // cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/density/color_cloud", 1);
    // check_pub = nh.advertise<sensor_msgs::PointCloud2>("/density/check_cloud", 1);
    
    width_x_2 = width/2.0;
    width_y_2 = width/2.0;

    volume_of_target_area = width*width*(height + min_height);
    
    std::cout << "========set param========" << std::endl;
    std::cout << "width: " << width << std::endl;
    std::cout << "height: " << height << std::endl;

    std::cout << "========calculater param========" << std::endl;
	std::cout << "csv_dir: " << csv_dir << std::endl;

    ofs.open(csv_dir);
	ofs << "count" << "," << "density" << std::endl;
	// ofs << "Time(density)" << "," << "1scan" << "," << "after_move"  << std::endl;
}

// 指定領域内の点群の点密度をプロットしていきたい
void PointsDensityPloter::cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg_cloud)
{
    std::cout << "========points density ploter========" << std::endl;
    pcl::fromROSMsg(*msg_cloud, *cloud_ptr);
    int cloud_size = cloud_ptr->points.size();
    std::cout << "subscribe cloud size: " << cloud_size << std::endl;
    int temp_count = 0;
    CloudXYZINPtr check_cloud(new CloudXYZIN);
    for(int i = 0; i < cloud_size; i++){
        PointXYZIN temp_p = cloud_ptr->points[i];
        double p_x = cloud_ptr->points[i].x;
        double p_y = cloud_ptr->points[i].y;
        double p_z = cloud_ptr->points[i].z;
        // 範囲を絞る
        if(p_x < width_x_2 && p_x > -width_x_2 && 
            p_y < width_y_2 && p_y > -width_y_2 && 
            p_z < height && p_z > -min_height){
            check_cloud->push_back(temp_p);
        }
    } 
    double density_of_points = check_cloud->points.size() / volume_of_target_area;
    ofs << all_count << "," << density_of_points << std::endl; 
    all_count ++;
}
void PointsDensityPloter::process()
{
    ros::spin();
    ofs.close();
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "points_density_ploter");
    PointsDensityPloter points_density_ploter;
    points_density_ploter.process();
    return 0;
}
PointsDensityPloter::GridPoint::GridPoint(void)
{}