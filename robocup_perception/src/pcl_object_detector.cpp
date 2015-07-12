//Using ros header
#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl_ros/point_cloud.h>
#include<visualization_msgs/Marker.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/PoseArray.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/features/boundary.h>
#include<flann/flann.h>
#include<pcl/kdtree/kdtree_flann.h>


//Using PCL header
#include<pcl/point_types.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/features/normal_3d.h>
#include<pcl/common/centroid.h>
#include<pcl/common/common.h>
#include<pcl/features/normal_3d_omp.h>

//PCL search
#include<pcl/search/search.h>
#include<pcl/search/kdtree.h>
#include<pcl/search/organized.h>

//領域成長法
#include<pcl/segmentation/region_growing.h>

//RANSACセグメント
#include<pcl/sample_consensus/method_types.h>
#include<pcl/segmentation/sac_segmentation.h>
#include<pcl/filters/extract_indices.h>
#include<pcl/ModelCoefficients.h>
#include<pcl/segmentation/extract_clusters.h>

//外れ値除去
#include<pcl/filters/statistical_outlier_removal.h>

//パススルーフィルター
#include<pcl/filters/passthrough.h>

#define TEST
#define SINGLE
//#define MULTI

typedef pcl::PointCloud<pcl::PointXYZRGB> rgbcloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbcloudPtr;

namespace{
    ros::Publisher pub;
    ros::Publisher vis_pub;
    ros::Publisher point_pub;
}

/*
 * @brief Voxel Grid filter
 * @in pcl::PointCloud<pcl::PointXYZRGB> input_cloud
 * @out pcl::PointCloud<pcl::PointXYZRGB> vf_cloud
 *
 */

rgbcloudPtr voxelGrid(rgbcloudPtr input_cloud){
    rgbcloudPtr zpf_cloud(new rgbcloud);
    rgbcloudPtr xpf_cloud(new rgbcloud);
    pcl::PassThrough<pcl::PointXYZRGB> z_pass;
    z_pass.setInputCloud(input_cloud);
    z_pass.setFilterFieldName("z");
    z_pass.setFilterLimits(0.0, 1.5);
    z_pass.filter(*zpf_cloud);

    pcl::PassThrough<pcl::PointXYZRGB> x_pass;
    x_pass.setInputCloud(zpf_cloud);
    x_pass.setFilterFieldName("x");
    x_pass.setFilterLimits(-1.0, 1.0);
    x_pass.filter(*xpf_cloud);


    rgbcloudPtr vf_cloud(new rgbcloud);
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(xpf_cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*vf_cloud);

    return vf_cloud;
}

/* 
 * @brief Plane Extraction
 * @in pcl::PointCloud<pcl::PointXYZRGB> input_cloud
 * @in pcl::PointIndices::Ptr inliers
 * @in pcl::ModelCoefficients::Ptr coefficients
 *
 * @out pcl::PointIndices::Ptr
 * @out pcl::ModelCoefficients::Ptr
 *
 */

void planeSegmentation(rgbcloudPtr input_cloud, const pcl::PointIndices::Ptr &inliers, const pcl::ModelCoefficients::Ptr &coefficients){
    //セグメント
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(input_cloud);
    seg.segment(*inliers, *coefficients);
}

/*
 * @brief extract indices
 * @in pcl::PointCloud<pcl::PointXYZRGB> input_cloud
 * @in pcl::PointIndices::Ptr inliers
 * @in pcl::ModelCoefficients::Ptr coefficients
 * @in bool flag
 *
 * @out pcl::PointCloud<pcl::PointXYZRGB> removal_cloud
 *
 */

rgbcloudPtr extractindices(rgbcloudPtr input_cloud, const pcl::PointIndices::Ptr &inliers, const pcl::ModelCoefficients::Ptr &coefficients, bool flag){
    rgbcloudPtr extract_cloud(new rgbcloud);
    rgbcloudPtr removal_cloud(new rgbcloud);

    //除去
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(input_cloud);
    extract.setIndices(inliers);
    extract.setNegative(flag);
    extract.filter(*extract_cloud);

    //外れ値除去
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(extract_cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*removal_cloud);
    return removal_cloud;
}

/*
 * @brief 
 */

rgbcloudPtr removedCloud(rgbcloudPtr input_cloud, pcl::PointIndices::Ptr &inliers){

}

/*
 * @brief Region Growing Classification
 * @in pcl::PointCloud<pcl::PointXYZRGB> input_cloud
 * 
 * @out std::vector<pcl::PointIndices> clusters
 *
 */

std::vector<pcl::PointIndices> regiongrowing(rgbcloudPtr input_cloud){
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);

    //法線推定
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(input_cloud);
    normal_estimator.setKSearch(10);
    normal_estimator.compute(*normals);

    //領域成長
    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
    reg.setMinClusterSize(50);
    reg.setMaxClusterSize(1000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(10);
    reg.setInputCloud(input_cloud);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(10.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(3.0);
    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

#ifdef DEBUG
    rgbcloudPtr colored_cloud = reg.getColoredCloud();

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*colored_cloud, output);
    output.header.frame_id = input_cloud->header.frame_id;
    pub.publish(output);
#endif

    return clusters;
}

/*
 * @brief calculate Centroid
 * @in pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud
 * @in std::vector<pcl::PointIndices> clusters
 */

Eigen::Vector4f centroidCompute(rgbcloudPtr input_cloud, std::vector<pcl::PointIndices> clusters)
{

    geometry_msgs::PoseStamped objectpoint;
    //std::vector<Eigen::Vector4f> vector_centroid;
    Eigen::Vector4f centroid;
    objectpoint.header.frame_id = input_cloud->header.frame_id;
    objectpoint.header.stamp = ros::Time::now();



    std::cout<<"clusters : "<<clusters.size()<<std::endl;

    //for(std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it){
    //for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
    //for(int i = 0; i < 10; i++){
    pcl::compute3DCentroid(*input_cloud, clusters[0], centroid);

    objectpoint.pose.position.x = centroid[0];
    objectpoint.pose.position.y = centroid[1];
    objectpoint.pose.position.z = centroid[2];

    //         point.pose.position.x = vector_centroid[i].x();
    //         point.pose.position.y = vector_centroid[i].y();
    //         point.pose.position.z = vector_centroid[i].z();
    //
    std::cout<<"[x y z]  = "<<" [ ] "<<"["<<objectpoint.pose.position.x<< " , "<<objectpoint.pose.position.y <<" , "<<objectpoint.pose.position.z <<"]"<<std::endl;
    return centroid;
    point_pub.publish(objectpoint);
    //}
    // }
}

void mark_cluster(rgbcloudPtr input_cloud, Eigen::Vector4f centroid, int id){
    Eigen::Vector4f min;
    Eigen::Vector4f max;

    pcl::getMinMax3D(*input_cloud, min, max);
    uint32_t shape = visualization_msgs::Marker::CUBE;
    visualization_msgs::Marker marker;
    marker.header.frame_id = input_cloud->header.frame_id;
    marker.header.stamp = ros::Time::now();

    std::string object_name;
    object_name = "object_name" + id;

    marker.ns = object_name;
    marker.id = id;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = centroid[0];
    marker.pose.position.y = centroid[1];
    marker.pose.position.z = centroid[2];
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = (max[0] - min[0]);
    marker.scale.y = (max[1] - min[1]);
    marker.scale.z = (max[2] - min[2]);

    if(marker.scale.x == 0) marker.scale.x = 0.1;
    if(marker.scale.y == 0) marker.scale.y = 0.1;
    if(marker.scale.z == 0) marker.scale.z = 0.1;

    marker.color.r = 255;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.color.a = 0.5;

    marker.lifetime = ros::Duration(0.1);
    vis_pub.publish(marker);
}

std::vector<pcl::PointIndices> euclideanCluster(rgbcloudPtr input_cloud){
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new::pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(input_cloud);
    std::vector<pcl::PointIndices> cloud_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.02);
    ec.setMinClusterSize(50);
    ec.setMaxClusterSize(1000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input_cloud);
    ec.extract(cloud_indices);
    return cloud_indices;
}

void pcCallback(const sensor_msgs::PointCloud2::Ptr &msg)
{
    //sensor_msgs to point cloud
    sensor_msgs::PointCloud2::Ptr cloud = msg;
    rgbcloudPtr convert_cloud(new rgbcloud);
    pcl::fromROSMsg(*cloud, *convert_cloud);

    //点群削減
    rgbcloudPtr vf_cloud(new rgbcloud);
    vf_cloud = voxelGrid(convert_cloud);

    rgbcloudPtr bf_cloud(new rgbcloud);

//     if(vf_cloud->empty()){}
//     else{
//         pcl::search::KdTree<pcl::PointXYZRGB>::Ptr flann(new pcl::search::KdTree<pcl::PointXYZRGB>);
//         pcl::PointCloud<pcl::Normal>::Ptr normals_cloud(new pcl::PointCloud<pcl::Normal>);
//         pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
//         norm_est.setSearchMethod(flann);
//         norm_est.setInputCloud(vf_cloud);
//         norm_est.setRadiusSearch(0.02);
//         norm_est.compute(*normals_cloud);
//
//         pcl::BoundaryEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::Boundary> boundary_est;
//         pcl::BoundaryEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::Boundary>::PointCloudOut boundary;
//         boundary_est.setRadiusSearch(0.02);
//         boundary_est.setInputNormals(normals_cloud);
//
//         boundary_est.setSearchMethod(flann);
//         boundary_est.setInputCloud(vf_cloud);
//         boundary_est.setAngleThreshold(M_PI/6);
//         boundary_est.compute(boundary);
//
//         for(int i = 0; i < boundary.size(); i++){
//             if(boundary.points[i].boundary_point == 0){
//                 bf_cloud->push_back(vf_cloud->points.at(i));
//             }else{
//                 pcl::PointXYZRGB pt = vf_cloud->points.at(i);
//                 pt.x = NAN;
//                 pt.y = NAN;
//                 pt.z = NAN;
//                 bf_cloud->push_back(pt);
//             }
//         }
//     }


    //平面検出＋平面除去
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    rgbcloudPtr extract_cloud(new rgbcloud);
    rgbcloudPtr plane_cloud(new rgbcloud);
    extract_cloud = extractindices(vf_cloud, inliers, coefficients, true);

    for(int i= 0; i < 7; i++){
        planeSegmentation(vf_cloud, inliers, coefficients);
        extract_cloud = extractindices(extract_cloud, inliers, coefficients, true);
        //plane_cloud = extractindices(vf_cloud, inliers, coefficients, false);
    }
    //その他の点の除去
    //     rgbcloudPtr removed_outlier_cloud(new rgbcloud);
    //     removed_outlier_cloud = removedCloud(extract_cloud, inliers);

    //クラスタリング
    std::vector<pcl::PointIndices> clusters;
    //clusters = regiongrowing(extract_cloud);
    clusters = euclideanCluster(extract_cloud);

    //std::cout<<"clusters : "<<clusters.size()<<std::endl;
    //std::cout<<"Point Size :"<<extract_cloud->size()<<std::endl;

    rgbcloudPtr tempcloud(new rgbcloud);
    int i = 0;
    for(std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it){
        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
            tempcloud->points.push_back(extract_cloud->points[*pit]);
            if(tempcloud->points.size() > 300){
                pcl::PointXYZRGB pt = extract_cloud->points.at(i);
                pt.x = NAN;
                pt.y = NAN;
                pt.z = NAN;
                tempcloud->push_back(pt);
            }
            tempcloud->width = tempcloud->size();
            tempcloud->height = 1;
            tempcloud->is_dense = true;
        }
    }

    //重心計算
    extract_cloud->header.frame_id = msg->header.frame_id;
    Eigen::Vector4f centroid;
    //centroid = centroidCompute(tempcloud, clusters);

    geometry_msgs::PoseArray objectpoint;
    geometry_msgs::Pose objectpose;
    objectpoint.header.frame_id = msg->header.frame_id;
    objectpoint.header.stamp = ros::Time::now();
    for(int i =0; i < clusters.size(); i++){
        pcl::compute3DCentroid(*extract_cloud, clusters[i].indices, centroid);

        std::cout<<"error no"<<std::endl;

        objectpose.position.x = centroid[0];
        objectpose.position.y = centroid[1];
        objectpose.position.z = centroid[2];

        std::cout<<"[x y z]  = "<<" ["<<i<<" ] "<<"["<<objectpose.position.x<< " , "<<objectpose.position.y <<" , "<<objectpose.position.z <<"]"<<std::endl;

        objectpoint.poses.push_back(objectpose);

        point_pub.publish(objectpoint);
    }

    //ビジュアライズ用
    for(size_t i = 0; i < clusters.size(); i++){
        mark_cluster(extract_cloud, centroid, i);
    }



    std::cout<<"tempcloud"<<tempcloud->width<<" : "<<tempcloud->height<<std::endl;
    std::cout<<"extract_cloud"<<extract_cloud->width<<" :  "<<extract_cloud->height<<std::endl;
#ifdef TEST

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*tempcloud, output);
    std::cout<<"stop"<<std::endl;
    output.header.frame_id = msg->header.frame_id;
    pub.publish(output);
#endif
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_object_detector");
    ros::NodeHandle node;

    ros::Subscriber pc_sub = node.subscribe("/camera/depth_registered/points", 100, pcCallback);
    pub = node.advertise<sensor_msgs::PointCloud2>("/output",100);
    point_pub = node.advertise<geometry_msgs::PoseArray>("/catchpoint",100);
    vis_pub = node.advertise<visualization_msgs::Marker>("bounding_box",0);

    ros::spin();
    return 0;
}
