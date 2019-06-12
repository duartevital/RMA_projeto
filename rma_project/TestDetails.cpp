#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/LightSource>
#include <osgShadow/ShadowedScene>
#include <osgShadow/ShadowTexture>
#include <osgShadow/ShadowMap>
#include <osg/Texture2D>
#include <osg/AnimationPath>
#include <osg/Fog>
#include <osg/BlendFunc>
#include <osg/Point>
#include <osg/PointSprite>
#include <osgParticle/ParticleSystem>
#include <osgParticle/ParticleSystemUpdater>
#include <osgParticle/ModularEmitter>
#include <osgParticle/ModularProgram>
#include <osgParticle/AccelOperator>
#include <osgParticle/FluidFrictionOperator>
#include <osgGA/TrackballManipulator>
#include <osgText/Font>
#include <osgText/Text>

#include <string>
#include <stdlib.h>
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
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_voxelized (new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr table_top_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_above_plane (new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
double c_a, c_b, c_c, c_d;
std::vector<pcl::PointIndices> cluster_indices;
ofstream output_file;
std::vector<Object> objects_in_file;
std::vector<Object> objects_in_cloud;
int max_area = 0, max_height = 0, max_width = 0;

//Guardar em objects_on_file os objectos no txt
void getObjectsInFile(){
    std::string line("");
    std::string delimiter(" ");
    ifstream file("../output_file");
    if(file.is_open()){
        while( getline(file, line) && !line.empty()){
            int pos=0;
            std::vector<std::string> token_list;
            while((pos = line.find(delimiter)) != std::string::npos){
                token_list.push_back(line.substr(0, pos));
                line.erase(0, pos+delimiter.length());
            }
            std::string type = token_list[0]; std::string color = token_list[1];
            int height = atoi(token_list[2].c_str()); int width = atoi(token_list[3].c_str());
            int red = atoi(token_list[4].c_str()); int green = atoi(token_list[5].c_str()); int blue = atoi(token_list[6].c_str());
            int area = atoi(token_list[7].c_str());

            Object obj(type, color, height, width, red, green, blue, area);
            objects_in_file.push_back(obj);
        }
        file.close();
    }else
        std::cout << "Unable to open file" << std::endl;
}

float getEuclideanDistance(pcl::PointXYZRGBA p1, pcl::PointXYZRGBA p2){
    return sqrt(pow(p1.x-p2.x, 2) + pow(p1.z-p2.z, 2)) * 100;
}

void setObjectType(){
    for(int i=0; i<objects_in_cloud.size(); i++){
        //Variables in cloud
        int height1=objects_in_cloud[i].getHeight(), width1=objects_in_cloud[i].getWidth();
        int red1=objects_in_cloud[i].getRed(), green1=objects_in_cloud[i].getGreen(), blue1=objects_in_cloud[i].getBlue();
        int area1=objects_in_cloud[i].getArea();

        //Variables in txt file
        int height2=0, width2=0;
        int red2=0, green2=0, blue2=0;
        int area2=0;

        float distance = 10000;

        for(int j=0; j<objects_in_file.size(); j++){
            height2 = objects_in_file[j].getHeight(); width2 = objects_in_file[j].getWidth();
            red2 = objects_in_file[j].getRed(); green2 = objects_in_file[j].getGreen(); blue2 = objects_in_file[j].getBlue();
            area2 = objects_in_file[j].getArea();

            int height_count = pow(height1-height2, 2)/max_height, width_count = pow(width1-width2, 2)/max_width;
            float red_count = pow(red1-red2, 2)/255, green_count = pow(green1-green2, 2)/255, blue_count = pow(blue1-blue2, 2)/255;
            int area_count = pow(area1-area2, 2)/max_area;
            float count_sum = height_count + width_count + red_count + green_count + blue_count + area_count;
            float tmp_distance = sqrt(count_sum);

            if(distance > tmp_distance){
                distance = tmp_distance;
                objects_in_cloud[i].setType(objects_in_file[j].getType());
                objects_in_cloud[i].setColor(objects_in_file[j].getColor());
            }
        }
    }
}

Object isPointInObject(pcl::PointXYZRGBA p){
    Object obj;
    int indice = -1;
    for(int i=0; i<cluster_indices.size(); i++){
        float d = 1000;
        for(int j=0; j<cluster_indices[i].indices.size(); j++){
            int x = cloud_above_plane->points[cluster_indices[i].indices[j]].x;
            int y = cloud_above_plane->points[cluster_indices[i].indices[j]].y;
            int z = cloud_above_plane->points[cluster_indices[i].indices[j]].z;

            if( getEuclideanDistance(p, cloud_above_plane->points[cluster_indices[i].indices[j]]) < 1){
                for(int k = 0; k < objects_in_cloud.size(); k++){
                    if(i == objects_in_cloud[k].getClusterIndice())
                        obj = objects_in_cloud[k];
                }
            }
        }
    }

    return obj;
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

void filterOutliers(){
    //Filtrar pontos fora da mesa usando método de clusters
    pcl::PassThrough<pcl::PointXYZRGBA> pass;
    pass.setInputCloud(cloud);
    //Eixo X
    pass.setFilterFieldName("x");
    pass.setFilterLimitsNegative(false);
    pass.setFilterLimits(-1, 0.5);
    //pass.filter(*cloud);
    //Eixo Y
    pass.setFilterFieldName("y");
    pass.setFilterLimitsNegative(false);
    pass.setFilterLimits(-1, 0.2);
    pass.filter(*cloud);
    //Eixo Z
    pass.setFilterFieldName("z");
    pass.setFilterLimitsNegative(false);
    pass.setFilterLimits(-1, 1.5);
    pass.filter(*cloud);

    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
    ec.setClusterTolerance(0.01);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(100000000);
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
    }

    //Remoção de outliers
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGBA> sor;
    sor.setInputCloud(cloud);
    sor.setRadiusSearch(0.05);
    sor.setMinNeighborsInRadius(50);
    sor.filter(*cloud);
}

//Sub-sample
void setSubSample(){
    pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_voxelized);
}

//Identificação do plano da mesa
void getTableTop(){
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (cloud_voxelized);
    seg.segment (*inliers, *coefficients);
    extract.setInputCloud(cloud_voxelized);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*table_top_cloud);
}

//Identificação dos pontos acima do plano da mesa e isolar objetos
void getObjectsOnTable(){
    c_a = coefficients->values[0]; c_b = coefficients->values[1];
    c_c = coefficients->values[2]; c_d = coefficients->values[3];
    for(int i = 0; i<cloud_voxelized->size(); i++){
        pcl::PointXYZRGBA p = cloud_voxelized->points[i];
        if(c_a*p.x+c_b*p.y+c_c*p.z+c_d > 0.01){
            cloud_above_plane->push_back(p);
        }
    }

    //Remoção de outliers
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGBA> sor;
    sor.setInputCloud(cloud_above_plane);
    sor.setRadiusSearch(0.03);
    sor.setMinNeighborsInRadius(18);
    sor.filter(*cloud_above_plane);

    //Separar clusters
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
    tree->setInputCloud(cloud_above_plane);
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
    ec.setClusterTolerance(0.015);
    ec.setMinClusterSize(30);
    ec.setMaxClusterSize(10000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_above_plane);
    ec.extract(cluster_indices);
}

void getObjectsDetails(){
    float maxY, minY, maxX, minX, maxZ, minZ;
    int height, width;
    pcl::PointXYZRGBA max_x_point; pcl::PointXYZRGBA min_x_point; pcl::PointXYZRGBA max_z_point; pcl::PointXYZRGBA min_z_point;
    pcl::PointXYZRGBA centroid;
    float highestDistance; float secondHighestDistance;
    std::vector<float> distanceVector;

    for(int i=0; i<cluster_indices.size(); i++){
        distanceVector.clear();
        maxY = 0,  minY = 0,   maxX = 0,   minX = 0,   maxZ = 0,   minZ = 0;
        float red_sum=0, green_sum=0, blue_sum=0;
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
        //Calculado centroid do cluster
        centroid.x = (maxX + minX)/2;
        centroid.y = (maxY + minY)/2;
        centroid.z = (maxZ + minZ)/2;

        //Calculadas distâncias entre todas as extremidades em X e Z;
        distanceVector.push_back(getEuclideanDistance(max_x_point, min_x_point));
        distanceVector.push_back(getEuclideanDistance(max_x_point, max_z_point));
        distanceVector.push_back(getEuclideanDistance(max_x_point, min_z_point));
        distanceVector.push_back(getEuclideanDistance(max_z_point, min_x_point));
        distanceVector.push_back(getEuclideanDistance(max_z_point, min_z_point));
        distanceVector.push_back(getEuclideanDistance(min_z_point, min_x_point));

        for(int k=0; k<distanceVector.size(); k++){
            if(distanceVector[k] > highestDistance)
                highestDistance = distanceVector[k];
            else if(distanceVector[k] > secondHighestDistance)
                secondHighestDistance = distanceVector[k];
        }
        height = sqrt(pow(maxY-minY, 2)) * 100;
        width = secondHighestDistance;

        int red_avg = red_sum / cluster_indices[i].indices.size();
        int green_avg = green_sum / cluster_indices[i].indices.size();
        int blue_avg = blue_sum / cluster_indices[i].indices.size();
        int area_sum = cluster_indices[i].indices.size();

        if(area_sum > max_area)
            max_area = area_sum;
        if(height > max_height)
            max_height = height;
        if(width > max_width)
            max_width = width;

        Object obj("", "", height, width, red_avg, green_avg, blue_avg, area_sum);
        obj.setCentroid(centroid);
        obj.setClusterIndice(i);
        objects_in_cloud.push_back(obj);
    }
}


void analyze(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr arg_cloud){
    cloud = arg_cloud;

    getObjectsInFile();

    filterOutliers();
    setSubSample();
    getTableTop();
    getObjectsOnTable();
    alignPointCloud(arg_cloud); alignPointCloud(cloud); alignPointCloud(cloud_voxelized); alignPointCloud(cloud_above_plane); alignPointCloud(table_top_cloud);
    getObjectsDetails();

    setObjectType();

}
