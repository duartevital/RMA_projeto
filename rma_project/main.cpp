using namespace std;
#include "Object.cpp"
#include "TestDetails.cpp"
#include "Question.cpp"
#include "Game.cpp"

int main(int argsc, char** argsv)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    //Load file to PointCloud
    if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(argsv[1], *cloud) == -1){
    //if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>("../clouds/treino/objecto_livro_azul.pcd", *cloud) == -1){
        PCL_ERROR ("Couldn't read file. \n");
        return(-1);
    }
    std::cout << "Loaded " << cloud ->width * cloud ->height << " points" <<std::endl;


    //getObjectsInFile();
    analyze(cloud);


    //Setting clouds
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_red (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_blue (new pcl::PointCloud<pcl::PointXYZRGBA>);

    cloud_red = table_top_cloud;
    cloud_blue = cloud_above_plane;

    //Setting PCL Visualizer
    pcl::visualization::PCLVisualizer *viewerPCL = new pcl::visualization::PCLVisualizer("3D Viewer");
    viewerPCL->setBackgroundColor(0,0,0);

    viewerPCL->addPointCloud<pcl::PointXYZRGBA> (cloud_red, "red cloud");
    viewerPCL->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "red cloud");
    viewerPCL->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_COLOR, 255,0,0, "red cloud");
    viewerPCL->addPointCloud<pcl::PointXYZRGBA> (cloud_blue, "blue cloud");
    viewerPCL->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "blue cloud");
    viewerPCL->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_COLOR, 0,0,255, "blue cloud");

    viewerPCL->addCoordinateSystem(1.0);
    viewerPCL->initCameraParameters();


    //Setting HUD text and camera
    osg::ref_ptr<osg::Geode> textGeode = new osg::Geode;
    osgText::Text* question_txt = createText(osg::Vec3(50.0f, 650.0f, 0.0f), getRandomQuestion().getQuestion(), 20.0f);
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

