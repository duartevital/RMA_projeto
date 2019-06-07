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
std::vector<std::string> file_list;
std::vector<std::string> object_list;
double c_a, c_b, c_c, c_d;
std::stringstream object_str, height_str,  width_str, red_str, green_str, blue_str, area_str;
ofstream output_file;
int n=0;

void setFileNames(){
    file_list.push_back("../clouds/treino/objecto_caixa_azul.pcd");
    file_list.push_back("../clouds/treino/objecto_caixa_branca.pcd");
    file_list.push_back("../clouds/treino/objecto_caixa_vermelha.pcd");
    file_list.push_back("../clouds/treino/objecto_cassete_preta.pcd");
    file_list.push_back("../clouds/treino/objecto_furador.pcd");
    file_list.push_back("../clouds/treino/objecto_livro_azul.pcd");
    file_list.push_back("../clouds/treino/objecto_livro_cinzento.pcd");
    file_list.push_back("../clouds/treino/objecto_xicara_branca.pcd");

    object_list.push_back("caixa azul ");
    object_list.push_back("caixa branca ");
    object_list.push_back("caixa vermelha ");
    object_list.push_back("cassete preta ");
    object_list.push_back("furador ");
    object_list.push_back("livro azul ");
    object_list.push_back("livro cinzento ");
    object_list.push_back("xicara branca ");
}

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
    pass.setFilterLimits(-0.3, 0.4);
    pass.filter(*cloud);
    //Eixo Z
    pass.setFilterFieldName("z");
    pass.setFilterLimitsNegative(false);
    pass.setFilterLimits(-0.1, 1.4);
    pass.filter(*cloud);

    /*pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
    ec.setClusterTolerance(0.03);
    ec.setMinClusterSize(1000);
    ec.setMaxClusterSize(20000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(indices);
    int max_size = 0;
    std::cout << "number =  " << n << std::endl;
    for(int i=0; i<indices.size(); i++){
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

        std::cout << "indexcdcec  " << indices[i].indices.size() << std::endl;
        for(int j=0; j<indices[i].indices.size(); j++){
            tmp_cloud->points.push_back(cloud->points[indices[i].indices[j]]);
            n++;
            //std::cout << "valores" << j <<" =  " << indices[i].indices[j] << std::endl;
        }

        tmp_cloud->width = tmp_cloud->points.size();
        tmp_cloud->height = 1;
        tmp_cloud->is_dense = true;
        if(tmp_cloud->size() > max_size){
            max_size = tmp_cloud->size();
            cloud = tmp_cloud;
        }
    }*/

    //Remoção de outliers
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGBA> sor;
    sor.setInputCloud(cloud);
    sor.setRadiusSearch(0.04);
    sor.setMinNeighborsInRadius(60);
    sor.filter(*cloud);

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

void alignPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ex_cloud){
    double pitch, roll, height;
    double norm = sqrt(c_a*c_a+c_b*c_b+c_c*c_c);
    c_a = c_a/norm;
    c_b = c_b/norm;
    c_c = c_c/norm;

    if (c_c<0){
        c_a*=-1; c_b*=-1; c_c*=-1;
    }

    pitch = asin(c_c);
    roll = -acos(c_b/cos(pitch));
    height = fabs(c_d);

    if (c_a<0)
        (roll) *= -1;

    Eigen::Affine3f t1 = pcl::getTransformation (0.0, -height, 0.0, 0.0, 0.0, 0);
    Eigen::Affine3f t2 = pcl::getTransformation (0.0, 0.0, 0.0, -pitch, 0.0, 0.0);
    Eigen::Affine3f t3 = pcl::getTransformation (0.0, 0.0, 0.0, 0.0, 0.0, -roll);
    pcl::transformPointCloud(*ex_cloud, *ex_cloud, t1*t2*t3);
}

void getObjectsOnTable(){
    c_a = coefficients->values[0]; c_b = coefficients->values[1];
    c_c = coefficients->values[2]; c_d = coefficients->values[3];
    for(int i = 0; i<cloud->size(); i++){
        pcl::PointXYZRGBA p = cloud->points[i];
        if(c_a*p.x+c_b*p.y+c_c*p.z+c_d > 0.01){
            cloud_above_plane->push_back(p);
        }
    }

    //Remoção de outliers
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGBA> sor;
    sor.setInputCloud(cloud_above_plane);
    sor.setRadiusSearch(0.04);
    sor.setMinNeighborsInRadius(18);
    sor.filter(*cloud_above_plane);
}

void getObjectsDetails(int i){
    float maxY=0, minY=0, maxX=0, minX=0, maxZ=0, minZ=0;
    int height, width;
    pcl::PointXYZRGBA max_x_point; pcl::PointXYZRGBA min_x_point; pcl::PointXYZRGBA max_z_point; pcl::PointXYZRGBA min_z_point;
    float highestDistance; float secondHighestDistance;
    std::vector<float> distanceVector;

        float red_sum=0, green_sum=0, blue_sum=0;
        highestDistance=0; secondHighestDistance=0;
        pcl::PointXYZRGBA p;
        for(int j=0; j<cloud_above_plane->size(); j++){

            //Vistas as extremidades dos objetos em X e Z
            if(cloud_above_plane->points[j].y > maxY || maxY == 0)
                maxY = cloud_above_plane->points[j].y;
            if(cloud_above_plane->points[j].y < minY || minY == 0)
                minY = cloud_above_plane->points[j].y;

            if(cloud_above_plane->points[j].x > maxX || maxX == 0){
                maxX = cloud_above_plane->points[j].x;
                max_x_point = cloud_above_plane->points[j];
            }
            if(cloud_above_plane->points[j].x < minX || minX == 0){
                minX = cloud_above_plane->points[j].x;
                min_x_point = cloud_above_plane->points[j];
            }

            if(cloud_above_plane->points[j].z > maxZ || maxZ == 0){
                maxZ = cloud_above_plane->points[j].z;
                max_z_point = cloud_above_plane->points[j];
            }
            if(cloud_above_plane->points[j].z < minZ || minZ == 0){
                minZ = cloud_above_plane->points[j].z;
                min_z_point = cloud_above_plane->points[j];
            }

            //Guardados valores dos canais R,G, e B
            p = cloud_above_plane->points[j];
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
            if(distanceVector[k] > highestDistance)
                highestDistance = distanceVector[k];
            else if(distanceVector[k] > secondHighestDistance)
                secondHighestDistance = distanceVector[k];
        }
        height = sqrt(pow(maxY-minY, 2)) * 100;
        width = highestDistance;

        int red_avg = red_sum / cloud_above_plane->size();
        int green_avg = green_sum / cloud_above_plane->size();
        int blue_avg = blue_sum / cloud_above_plane->size();
        int area_sum = cloud_above_plane->size();


        //getObjectType(height, width, red_avg, green_avg, blue_avg, area_sum);
        object_str << object_list[i];
        height_str << height << " ";
        width_str << width << " ";
        red_str << red_avg << " ";
        green_str << green_avg << " ";
        blue_str << blue_avg << " ";
        area_str << area_sum;
        std::stringstream output_str;
        output_str << object_str.str() << height_str.str() << width_str.str() << red_str.str() << green_str.str() << blue_str.str() << area_str.str() << "\n";
        std::cout << output_str.str() << std::endl;

        if(output_file.is_open()){
            output_file << output_str.str();
        }

        output_str.str(""); object_str.str(""); height_str.str(""); width_str.str("");
        red_str.str(""); green_str.str(""); blue_str.str(""); area_str.str("");
}

int main(){
    setFileNames();
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

    output_file.open("../output_file");

    for(int i=0; i<file_list.size(); i++){
        std::cout << "file name =  " << file_list[i] << std::endl;
        cloud->clear(); table_top_cloud->clear(); cloud_above_plane->clear();

        if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(file_list[i], *cloud) == -1){
                PCL_ERROR ("Couldn't read file. \n");
                return(-1);
        }
        setSubSample();
        filterOutliers();
        getTableTop();
        getObjectsOnTable();
        alignPointCloud(cloud); alignPointCloud(cloud_above_plane); alignPointCloud(table_top_cloud);
        getObjectsDetails(i);
    }


    output_file.close();
}
