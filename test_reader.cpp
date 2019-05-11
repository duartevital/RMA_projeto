#include <iostream>
#include <fstream>
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

int main(int argsc, char** argsv){

    //Load file to PointCloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>("../clouds/treino/objecto_caixa_vermelha.pcd", *cloud) == -1){
        PCL_ERROR ("Couldn't read file. \n");
        return(-1);
    }
    std::cout << "Loaded " << cloud ->width * cloud ->height << " points" <<std::endl;

    //Alinhamento com os eixos
    double d_x=0, d_y=-0.5, d_z=-0.5;
    double rot_x=-M_PI/6, rot_y=0, rot_z=0;
    Eigen::Affine3f t = pcl::getTransformation(d_x,d_y,d_z, rot_x,rot_y,rot_z);
    pcl::transformPointCloud(*cloud, *cloud, t);

    //Sub-sample
    pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud);


    //Filtrar pontos fora da mesa
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
    pass.setFilterLimits(-0.3, 0.5);
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
    sor.setMinNeighborsInRadius(50);
    sor.filter(*cloud);


    //Identificação do plano da mesa
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr table_top_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    extract.setInputCloud(cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*table_top_cloud);

    /*seg.setInputCloud (table_top_cloud);
    seg.segment (*inliers, *coefficients);
    extract.setInputCloud(table_top_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*table_top_cloud);*/

    //Identificação dos pontos acima do plano da mesa
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_above_plane (new pcl::PointCloud<pcl::PointXYZRGBA>);
    double a = coefficients->values[0]; double b = coefficients->values[1];
    double c = coefficients->values[2]; double d = coefficients->values[3];
    std::cout << "coeficentes: "<<a<<", "<<b<<", "<<c<<", "<<d<<std::endl;
    for(int i = 0; i<cloud->size(); i++){
        pcl::PointXYZRGBA p = cloud->points[i];
        if(a*p.x+b*p.y+c*p.z+d > 0.01){
            cloud_above_plane->push_back(p);
        }
    }

    //Isolar objetos e descobrir a altura
    float maxY = 0, minY = 0; int height;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr objects_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
    tree->setInputCloud(cloud_above_plane);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
    ec.setClusterTolerance(0.02);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(5000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_above_plane);
    ec.extract(cluster_indices);
    for(int i=0; i<cluster_indices[0].indices.size(); i++){
        objects_cloud->points.push_back(cloud_above_plane->points[cluster_indices[0].indices[i]]);
        if(objects_cloud->points[i].y > maxY)
            maxY = objects_cloud->points[i].y;
        if(objects_cloud->points[i].y < minY)
            minY = objects_cloud->points[i].y;
    }
    height = -(maxY + minY)*100;
    std::cout << "Object height: " << height << std::endl;

    //Write to file
    ofstream output_file;
    output_file.open("../output_file");
    if(output_file.is_open()){
        output_file << "lololol. \n";
        output_file.close();
    }
    else cout << "Unable to open file";


    //Setting visualizer
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_red (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_blue (new pcl::PointCloud<pcl::PointXYZRGBA>);

    //cloud_red = table_top_cloud;
    cloud_blue = objects_cloud;

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

