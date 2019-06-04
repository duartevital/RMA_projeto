#include <iostream>
#include <fstream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/filter.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr table_top_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_above_plane (new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
std::vector<pcl::PointIndices> cluster_indices;
std::stringstream object_str, height_str,  width_str, red_str, green_str, blue_str;
ofstream output_file;

//Alinhamento com os eixos
void alignWithAxis(){
    double d_x=0, d_y=-0.5, d_z=-0.5;
    double rot_x=-M_PI/6, rot_y=0, rot_z=0;
    Eigen::Affine3f t = pcl::getTransformation(d_x,d_y,d_z, rot_x,rot_y,rot_z);
    pcl::transformPointCloud(*cloud, *cloud, t);
}

//Voxel Grid
void setVoxelGrid(){
    pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter(*cloud);
}

//Sub-sample
void setSubSample(){
    pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud);
}

void filterOutliers(){
    //Filtrar pontos fora da mesa usando método de clusters
    pcl::PassThrough<pcl::PointXYZRGBA> pass;
    pass.setInputCloud(cloud);
    //Eixo X
    pass.setFilterFieldName("x");
    pass.setFilterLimitsNegative(false);
    pass.setFilterLimits(-0.4, 0.4);
    pass.filter(*cloud);
    //Eixo Y
    pass.setFilterFieldName("y");
    pass.setFilterLimitsNegative(false);
    pass.setFilterLimits(-0.4, 0.5);
    pass.filter(*cloud);
    //Eixo Z
    pass.setFilterFieldName("z");
    pass.setFilterLimitsNegative(false);
    pass.setFilterLimits(-0.1, 1);
    pass.filter(*cloud);

    //Remoção de outliers
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGBA> sor;
    sor.setInputCloud(cloud);
    sor.setRadiusSearch(0.05);
    sor.setMinNeighborsInRadius(60);
    sor.filter(*cloud);

    /*pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
    ec.setClusterTolerance(0.02);
    ec.setMinClusterSize(10000);
    ec.setMaxClusterSize(100000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(indices);

    int max_size = 0;
    for(int i=0; i<indices.size(); i++){
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
        for(int j=0; j<indices[i].indices.size(); j++){
            tmp_cloud->points.push_back(cloud->points[indices[i].indices[j]]);
        }
        tmp_cloud->width = tmp_cloud->points.size();
        tmp_cloud->height = 1;
        tmp_cloud->is_dense = true;
        if(tmp_cloud->size() > max_size){
            max_size = tmp_cloud->size();
            cloud = tmp_cloud;
        }
    }*/

}

//Identificação do plano da mesa
void getTableTop(){
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.001);
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    extract.setInputCloud(cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*table_top_cloud);
}

//Identificação dos pontos acima do plano da mesa e isolar objetos
void getObjectsOnTable(){
    double a = coefficients->values[0]; double b = coefficients->values[1];
    double c = coefficients->values[2]; double d = coefficients->values[3];
    for(int i = 0; i<cloud->size(); i++){
        pcl::PointXYZRGBA p = cloud->points[i];
        if(a*p.x+b*p.y+c*p.z+d > 0.01){
            cloud_above_plane->push_back(p);
        }
    }

    //Remoção de outliers
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGBA> sor;
    sor.setInputCloud(cloud_above_plane);
    sor.setRadiusSearch(0.05);
    sor.setMinNeighborsInRadius(20);
    sor.filter(*cloud_above_plane);

    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
    tree->setInputCloud(cloud_above_plane);
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
    ec.setClusterTolerance(0.03);
    ec.setMinClusterSize(50);
    ec.setMaxClusterSize(10000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_above_plane);
    ec.extract(cluster_indices);
}

void getObjectsDetails(){
    float maxY, minY, maxX, minX, maxZ, minZ;
    int height, width;
    pcl::PointXYZRGBA max_x_point; pcl::PointXYZRGBA min_x_point; pcl::PointXYZRGBA max_z_point; pcl::PointXYZRGBA min_z_point;
    float highestDistance; float secondHighestDistance;
    std::vector<float> distanceVector;

    for(int i=0; i<cluster_indices.size(); i++){
        std::cout << "Cluster size: " << cluster_indices[i].indices.size() << std::endl;
        distanceVector.clear();
        maxY = 0,  minY = 0,   maxX = 0,   minX = 0,   maxZ = 0,   minZ = 0;
        float red_sum, green_sum, blue_sum;
        highestDistance=0; secondHighestDistance=0;
        pcl::PointXYZRGBA p;
        for(int j=0; j<cluster_indices[i].indices.size(); j++){

            //Vistas as extremidades dos objetos em X e Z
            if(cloud_above_plane->points[cluster_indices[i].indices[j]].y > maxY || maxY == 0)
                maxY = cloud_above_plane->points[cluster_indices[i].indices[j]].y;
            if(cloud_above_plane->points[cluster_indices[i].indices[j]].y < minY || minY == 0)
                minY = cloud_above_plane->points[cluster_indices[i].indices[j]].y;

            if(cloud_above_plane->points[cluster_indices[i].indices[j]].x > maxX || maxX == 0){
                maxX = cloud_above_plane->points[cluster_indices[i].indices[j]].x;
                max_x_point = cloud_above_plane->points[cluster_indices[i].indices[j]];
            }
            if(cloud_above_plane->points[cluster_indices[i].indices[j]].x < minX || minX == 0){
                minX = cloud_above_plane->points[cluster_indices[i].indices[j]].x;
                min_x_point = cloud_above_plane->points[cluster_indices[i].indices[j]];
            }

            if(cloud_above_plane->points[cluster_indices[i].indices[j]].z > maxZ || maxZ == 0){
                maxZ = cloud_above_plane->points[cluster_indices[i].indices[j]].z;
                max_z_point = cloud_above_plane->points[cluster_indices[i].indices[j]];
            }
            if(cloud_above_plane->points[cluster_indices[i].indices[j]].z < minZ || minZ == 0){
                minZ = cloud_above_plane->points[cluster_indices[i].indices[j]].z;
                min_z_point = cloud_above_plane->points[cluster_indices[i].indices[j]];
            }

            //Guardados valores dos canais R,G, e B
            p = cloud_above_plane->points[cluster_indices[i].indices[j]];
            red_sum += p.r;
            green_sum += p.g;
            blue_sum += p.b;
        }
        //Calculadas distâncias entre todas as extremidades em X e Z;
        distanceVector.push_back(sqrt(pow(max_x_point.x-min_x_point.x, 2) + pow(max_x_point.z-min_x_point.z, 2)) * 100);
        distanceVector.push_back(sqrt(pow(max_x_point.x-max_z_point.x, 2) + pow(max_x_point.z-max_z_point.z, 2)) * 100);
        distanceVector.push_back(sqrt(pow(max_x_point.x-min_z_point.x, 2) + pow(max_x_point.z-min_z_point.z, 2)) * 100);
        distanceVector.push_back(sqrt(pow(max_z_point.x-min_x_point.x, 2) + pow(max_z_point.z-min_x_point.z, 2)) * 100);
        distanceVector.push_back(sqrt(pow(max_z_point.x-min_z_point.x, 2) + pow(max_z_point.z-min_z_point.z, 2)) * 100);
        distanceVector.push_back(sqrt(pow(min_z_point.x-min_x_point.x, 2) + pow(min_z_point.z-min_x_point.z, 2)) * 100);

        for(int k=0; k<distanceVector.size(); k++){
            //std::cout << "Distancia = " << distanceVector[k] << std::endl;
            if(distanceVector[k] > highestDistance)
                highestDistance = distanceVector[k];
            else if(distanceVector[k] > secondHighestDistance)
                secondHighestDistance = distanceVector[k];
        }
        height = sqrt(pow(maxY-minY, 2)) * 100;
        width = highestDistance;

        int red_avg = red_sum / cluster_indices[i].indices.size();
        int green_avg = green_sum / cluster_indices[i].indices.size();
        int blue_avg = blue_sum / cluster_indices[i].indices.size();


        object_str << "Object " << i << " ";
        height_str << height << " ";
        width_str << width << " ";
        red_str << red_avg << " ";
        green_str << green_avg << " ";
        blue_str << blue_avg;
        std::stringstream output_str;
        output_str << object_str.str() << height_str.str() << width_str.str() << red_str.str() << green_str.str() << blue_str.str() << "\n";
        std::cout << output_str.str() << std::endl;

        if(output_file.is_open()){
            output_file << output_str.str();
        }

        output_str.str(""); object_str.str(""); height_str.str(""); width_str.str("");
        red_str.str(""); green_str.str(""); blue_str.str("");
    }
}


int main(int argsc, char** argsv){

    //Load file to PointCloud
    if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(argsv[1], *cloud) == -1){
    //if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>("../clouds/treino/objecto_livro_azul.pcd", *cloud) == -1){
        PCL_ERROR ("Couldn't read file. \n");
        return(-1);
    }
    std::cout << "Loaded " << cloud ->width * cloud ->height << " points" <<std::endl;

    output_file.open("../output_file");

    alignWithAxis();
    setSubSample();
    filterOutliers();
    getTableTop();
    getObjectsOnTable();
    getObjectsDetails();


    output_file.close();

    //Setting visualizer
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_red (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_blue (new pcl::PointCloud<pcl::PointXYZRGBA>);

    //cloud_red = table_top_cloud;
    cloud_blue = cloud_above_plane;

    pcl::visualization::PCLVisualizer *viewer = new pcl::visualization::PCLVisualizer("3D Viewer");
    viewer->setBackgroundColor(0,0,0);

    /*viewer->addPointCloud<pcl::PointXYZRGBA> (cloud_red, "red cloud");
    viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "red cloud");
    viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_COLOR, 255,0,0, "red cloud");*/
    viewer->addPointCloud<pcl::PointXYZRGBA> (cloud_blue, "blue cloud");
    viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "blue cloud");
    viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_COLOR, 0,0,255, "blue cloud");

    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));

    }

}
