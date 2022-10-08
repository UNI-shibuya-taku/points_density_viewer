#include "points_density_viewer/compare_points_density_viewer.h"
ComparePointsDensityViewer::ComparePointsDensityViewer(void) : local_nh("~")
{
    local_nh.param("width", width, {50.0}); // 測定範囲
    // infant: 3.5~-2.6 くらい SQ2 : 13~-0.8くらい
    local_nh.param("height", height, {20.0}); // 測定高さ範囲
    local_nh.param("min_height", min_height, {10.0}); // 測定高さ範囲
    local_nh.param("voxel_num_x", voxel_num_x, {500}); // 区切りたい数
    local_nh.param("voxel_num_y", voxel_num_y, {500});
    local_nh.param("voxel_num_z", voxel_num_z, {500});
    local_nh.param("z_thre", z_thre, {3.0});
    local_nh.param("is_ceiling", is_ceiling, {false});
    
    cloud_sub = nh.subscribe("/velodyne_points", 1, &ComparePointsDensityViewer::cloud_callback, this); // sq_lidar 1scan
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/density/compare/color_cloud", 1);
    check_pub = nh.advertise<sensor_msgs::PointCloud2>("/density/compare/check_cloud", 1);
    
    voxel_size_x = width/(double)voxel_num_x; // 一つのボクセルのサイズ
    voxel_size_y = width/(double)voxel_num_y;
    voxel_size_z = (height+min_height)/(double)voxel_num_z;

    width_x_2 = width/2.0;
    width_y_2 = width/2.0;

    grid_volume = voxel_size_x*voxel_size_y*voxel_size_z; // 一つのボクセルの体積
    std::cout << "voxel size x: " << voxel_size_x << std::endl;
    std::cout << "voxel size y: " << voxel_size_y << std::endl;
    std::cout << "voxel size z: " << voxel_size_z << std::endl;
    std::cout << "grid volume: " << grid_volume << std::endl;
    
    std::cout << "========set param========" << std::endl;
    std::cout << "width: " << width << std::endl;
    std::cout << "height: " << height << std::endl;
    std::cout << "voxel_num_x: " << voxel_num_x << std::endl;
    std::cout << "voxel_num_y: " << voxel_num_y << std::endl;
    std::cout << "voxel_num_z: " << voxel_num_z << std::endl;
}

// ちゃんとindexが合っているか
// 色の変動について正しくできているか
// 点密度計算合ってる？
void ComparePointsDensityViewer::cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg_cloud)
{
    std::cout << "========points density viewer========" << std::endl;
    pcl::fromROSMsg(*msg_cloud, *cloud_ptr);
    int cloud_size = cloud_ptr->points.size();
    std::cout << "subscribe cloud size: " << cloud_size << std::endl;
    std::vector<std::vector<std::vector<GridPoint>>> voxel_grid(voxel_num_x, std::vector<std::vector<GridPoint>>(voxel_num_y, std::vector<GridPoint>(voxel_num_z)));
    int temp_count = 0;
    int max_x = 0;
    int min_x = 0;
    int max_y = 0;
    int min_y = 0;
    double max_z = 0;
    double min_z = 0;
    int index_size_max = 0;
    int keep_index_x, keep_index_y, keep_index_z;
    for(int i = 0; i < cloud_size; i++){
        double p_x = cloud_ptr->points[i].x;
        double p_y = cloud_ptr->points[i].y;
        double p_z = cloud_ptr->points[i].z;
        if(p_z > z_thre && is_ceiling) continue;
        // 範囲を絞る
        if(p_x < width_x_2 && p_x > -width_x_2 && p_y < width_y_2 && p_y > -width_y_2 && p_z < height && p_z > -min_height){
            int index_x = (int)(p_x / voxel_size_x);
            int index_y = (int)(p_y / voxel_size_y);
            int index_z = (int)(p_z / voxel_size_z);
            index_x += (voxel_num_x/2 - 1);
            index_y += (voxel_num_y/2 - 1);
            index_z += (voxel_num_z/2 - 1);
            if(p_z > max_z) max_z = p_z;
            if(p_z < min_z) min_z = p_z;
            if(index_x < voxel_num_x && index_y < voxel_num_y && index_z < voxel_num_z){
                // if(index_x > max_x) max_x = index_x;
                // if(index_x < min_x) min_x = index_x;
                // if(index_y > max_y) max_y = index_y;
                // if(index_y < min_y) min_y = index_y;
                // if(index_z > max_z) max_z = index_z;
                // if(index_z < min_z) min_z = index_z;
                voxel_grid[index_x][index_y][index_z].point_indices.push_back(i);
                if(index_size_max < voxel_grid[index_x][index_y][index_z].point_indices.size()){
                    index_size_max = voxel_grid[index_x][index_y][index_z].point_indices.size();
                    keep_index_x = index_x;
                    keep_index_y = index_y;
                    keep_index_z = index_z;
                }
            }
        }
    } 
    // std::cout << "max_x: " << max_x << " min_x: " << min_x << std::endl;
    // std::cout << "max_y: " << max_y << " min_y: " << min_y << std::endl;
    // std::cout << "max_z: " << max_z << " min_z: " << min_z << std::endl;
    std::cout << "max_pz: " << max_z << " min_pz: " << min_z << std::endl;
    std::cout << "index_size_max: " << index_size_max  << std::endl;
    std::cout << "keep_x: " << keep_index_x  << std::endl;
    std::cout << "keep_y: " << keep_index_y  << std::endl;
    std::cout << "keep_z: " << keep_index_z  << std::endl;

    CloudXYZRGBPtr cloud_density_rgb(new CloudXYZRGB);
    cloud_density_rgb->header = cloud_ptr->header;
    CloudXYZHSVPtr cloud_density_hsv(new CloudXYZHSV);
    cloud_density_hsv->header = cloud_ptr->header;
    CloudXYZINPtr check_cloud(new CloudXYZIN);
    check_cloud->header = cloud_ptr->header;
    for(int i = 0; i < voxel_num_x; i++){
        for(int j = 0; j < voxel_num_y; j++){
            for(int k = 0; k < voxel_num_z; k++){
                if(voxel_grid[i][j][k].point_indices.size() > 0){
                    for(int s = 0; s < voxel_grid[i][j][k].point_indices.size(); s++){
                        int select_index = voxel_grid[i][j][k].point_indices[s];
                        double the_voxel_density = (double)voxel_grid[i][j][k].point_indices.size() / grid_volume;
                        PointXYZHSV p_hsv;
                        p_hsv.x = cloud_ptr->points[select_index].x;
                        p_hsv.y = cloud_ptr->points[select_index].y;
                        p_hsv.z = cloud_ptr->points[select_index].z;
                        p_hsv.h = 1/the_voxel_density;
                        // p_hsv.h = the_voxel_density;
                        p_hsv.s = 1.0;
                        p_hsv.v = 1.0;
                        cloud_density_hsv->push_back(p_hsv);

                        // PointXYZRGB p_rgb;
                        // p_rgb.x = cloud_ptr->points[select_index].x;
                        // p_rgb.y = cloud_ptr->points[select_index].y;
                        // p_rgb.z = cloud_ptr->points[select_index].z;
                        // if(the_voxel_density > 0.0 && the_voxel_density <= 0.1 ){
                        //     p_rgb.b = 255;
                        // } // 低密度
                        // else if(the_voxel_density > 0.1 && the_voxel_density <= 10){
                        //     p_rgb.g = 255;
                        // } // 標準密度
                        // else if(the_voxel_density > 10 && the_voxel_density > 50){
                        //     p_rgb.r = 255;
                        // } // 高密度
                        // cloud_density_rgb->push_back(p_rgb);
                    }
                    voxel_grid[i][j][k].point_indices.clear();
                }
            }
        }
    }
    cloud_pub.publish(cloud_density_hsv);
    // cloud_pub.publish(cloud_density_rgb);
    max = 0;
}
void ComparePointsDensityViewer::process()
{
    ros::spin();
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "compare_points_density_viewer");
    ComparePointsDensityViewer compare_points_density_viewer;
    compare_points_density_viewer.process();
    return 0;
}
ComparePointsDensityViewer::GridPoint::GridPoint(void)
{
}