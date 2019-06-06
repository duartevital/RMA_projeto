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
double c_a, c_b, c_c, c_d;
std::vector<pcl::PointIndices> cluster_indices;
std::stringstream object_str, height_str,  width_str, red_str, green_str, blue_str, area_str;
ofstream output_file;


//Voxel Grid
void setVoxelGrid(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp){
    pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
    vg.setInputCloud(tmp);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter(*tmp);
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
    pass.setFilterLimits(-1, 0.5);
    //pass.filter(*cloud);
    //Eixo Y
    pass.setFilterFieldName("y");
    pass.setFilterLimitsNegative(false);
    pass.setFilterLimits(-1, 0.5);
    //pass.filter(*cloud);
    //Eixo Z
    pass.setFilterFieldName("z");
    pass.setFilterLimitsNegative(false);
    pass.setFilterLimits(-1, 1.5);
    pass.filter(*cloud);

    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
    ec.setClusterTolerance(0.03);
    ec.setMinClusterSize(1000);
    ec.setMaxClusterSize(20000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(indices);

    std::cout << "indices size = " << indices.size() << std::endl;

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

//Identificação dos pontos acima do plano da mesa e isolar objetos
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
    sor.setRadiusSearch(0.05);
    sor.setMinNeighborsInRadius(20);
    sor.filter(*cloud_above_plane);
    setVoxelGrid(cloud_above_plane);

    //Separar clusters
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

        int red_avg = red_sum / cluster_indices[i].indices.size();
        int green_avg = green_sum / cluster_indices[i].indices.size();
        int blue_avg = blue_sum / cluster_indices[i].indices.size();
        int area_sum = cluster_indices[i].indices.size();


        object_str << "Object " << i << " ";
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
}

osg::Camera* createHUDCamera( double left, double right, double bottom, double top){
    osg::ref_ptr<osg::Camera> camera = new osg::Camera;
    camera->setReferenceFrame( osg::Camera::ABSOLUTE_RF );
    camera->setClearMask( GL_DEPTH_BUFFER_BIT );
    camera->setRenderOrder( osg::Camera::POST_RENDER, 2);
    camera->setProjectionMatrix( osg::Matrix::ortho2D(left, right, bottom, top));
    return camera.release();
}

osgText::Text* createText(osg::Vec3 pos, std::string content, float size){
    //osg::ref_ptr<osgText::Font> g_font = osgText::readFontFile("fonts/arial.ttf");
    osg::ref_ptr<osgText::Text> text = new osgText::Text;
    //text->setFont( g_font.get() );
    text->setCharacterSize(size);
    text->setPosition(pos);
    text->setText(content);
    text->setDataVariance(osg::Object::DYNAMIC);
    return text.release();
}

std::string getRandomQuestion(){
    std::vector<std::string> question_vec;

    std::string q1 = "Selecciona um objeto à direita/esquerda do objeto pré-seleccionado.";
    std::string q2 = "Selecciona o objeto mais próximo/afastado do objeto pré-seleccionado.";
    std::string q3 = "Selecciona o objeto com maior/menor superfície";
    std::string q4 = "Selecciona o objeto mais alto/baixo";
    std::string q5 = "Selecciona o objeto mais largo/estreito";
    std::string q6 = "Seleciona um objeto mais escuro/claro do que o objeto pré-seleccionado";
    std::string q7 = "Selecciona o objeto do tipo X";
    question_vec.push_back(q1); question_vec.push_back(q2); question_vec.push_back(q3);
    question_vec.push_back(q4); question_vec.push_back(q5); question_vec.push_back(q6);
    question_vec.push_back(q7);

    int r = rand() % 7;
    return question_vec[r];
}

void analyze(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr arg_cloud){
    cloud = arg_cloud;

    //Load file to PointCloud
    /*if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(argsv[1], *cloud) == -1){
    //if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>("../clouds/treino/objecto_livro_azul.pcd", *cloud) == -1){
        PCL_ERROR ("Couldn't read file. \n");
        return(-1);
    }
    std::cout << "Loaded " << cloud ->width * cloud ->height << " points" <<std::endl;*/

    output_file.open("../output_file");

    setSubSample();
    filterOutliers();
    getTableTop();
    getObjectsOnTable();
    alignPointCloud(cloud); alignPointCloud(cloud_above_plane); alignPointCloud(table_top_cloud);
    getObjectsDetails();


    output_file.close();

    //Setting clouds
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_red (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_blue (new pcl::PointCloud<pcl::PointXYZRGBA>);

    cloud_red = cloud;
    cloud_blue = cloud_above_plane;


    //Setting PCL Visualizer
    pcl::visualization::PCLVisualizer *viewerPCL = new pcl::visualization::PCLVisualizer("3D Viewer");
    viewerPCL->setBackgroundColor(0,0,0);

    viewerPCL->addPointCloud<pcl::PointXYZRGBA> (cloud_red, "red cloud");
    viewerPCL->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "red cloud");
    viewerPCL->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_COLOR, 255,0,0, "red cloud");
    /*viewer->addPointCloud<pcl::PointXYZRGBA> (cloud_blue, "blue cloud");
    viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "blue cloud");
    viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_COLOR, 0,0,255, "blue cloud");*/

    viewerPCL->addCoordinateSystem(1.0);
    viewerPCL->initCameraParameters();

    //Setting HUD text and camera
    osg::ref_ptr<osg::Geode> textGeode = new osg::Geode;
    osgText::Text* question_txt = createText(osg::Vec3(50.0f, 650.0f, 0.0f), getRandomQuestion(), 20.0f);
    textGeode->addDrawable(question_txt);

    osg::Camera* hud_camera = createHUDCamera(0, 1024, 0, 768);
    hud_camera->addChild(textGeode.get());
    hud_camera->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    //Setting OSG Visualizer
    osg::ref_ptr<osg::Group> root = new osg::Group;
    root->addChild( hud_camera);

    osgViewer::Viewer viewerOSG;
    viewerOSG.setUpViewInWindow(0, 0, 640, 480);
    viewerOSG.setSceneData( root.get() );

    while (!viewerPCL->wasStopped ())
    {
        viewerPCL->spinOnce (100);
        viewerOSG.frame();
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));

    }

}