using namespace std;
#include "Object.cpp"
#include "TestDetails.cpp"
#include "Question.cpp"
#include "Game.cpp"
#include "Controller.cpp"

int main(int argsc, char** argsv)
{
	// create a point cloud to keep track of the ball's path
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ballPath (new pcl::PointCloud<pcl::PointXYZRGBA>);
    ballPath->height = 1;

		
    //! MIGUEL STUFF
    // ball dynamics state variables
    bool collidedLeft = false, collidedRight = false, collidedFront = false, collidedBack = false, collidedBelow = false, bellowpoint = false;
    float rotation = 0;
    pcl::PointXYZRGBA lookingForY;
    //!MIGUEL STUFF

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    //Load file to PointCloud
    if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(argsv[1], *cloud) == -1){
    //if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>("../clouds/treino/objecto_livro_azul.pcd", *cloud) == -1){
        PCL_ERROR ("Couldn't read file. \n");
        return(-1);
    }
    std::cout << "Loaded " << cloud ->width * cloud ->height << " points" <<std::endl;

    //! MIGUEL STUFF
    double camera_pitch, camera_roll, camera_height;
    estimateCameraPose(cloud, &camera_pitch, &camera_roll, &camera_height);
    //! MIGUEL STUFF



    //! MIGUEL STUFF
    // go through the point cloud and generate a RGB image and a range image
    int size = cloud->height*cloud->width*3;
    unsigned char* data = (unsigned char*)calloc(size,sizeof(unsigned char));
    unsigned char* dataDepth = (unsigned char*)calloc(size,sizeof(unsigned char));
    createImageFromPointCloud(cloud, data, dataDepth);
    // create a texture from the RGB image and use depth data to fill the z-buffer
    osg::ref_ptr<osg::Geode> orthoTextureGeode = new osg::Geode;
    createOrthoTexture(orthoTextureGeode, data, dataDepth, cloud->width, cloud->height);
    //! MIGUEL STUFF


    analyze(cloud);


    //Setting clouds
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_red (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_blue (new pcl::PointCloud<pcl::PointXYZRGBA>);

    cloud_red = cloud_above_plane;
    cloud_blue = cloud;

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

    //! MIGUEL STUFF
    // create an orthographic camera imaging the texture and a perspective camera based on the camera's pose and intrinsics
    osg::ref_ptr<osg::Camera> camera1 = new osg::Camera;
    osg::ref_ptr<osg::Camera> camera2 = new osg::Camera;
    createVirtualCameras(camera1, camera2, orthoTextureGeode, camera_pitch, camera_roll, camera_height);
    //! MIGUEL STUFF


    //Setting OSG Visualizer
    osg::ref_ptr<osg::Group> root = new osg::Group;
    root->addChild( camera1.get() );
    root->addChild( camera2.get() );
    root->addChild( hud_camera);


    //! MIGUEL STUFF
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rotated (new pcl::PointCloud<pcl::PointXYZRGBA>);

	rotatePointCloud(cloud, cloud_rotated, camera_pitch, camera_roll, camera_height);
	
    // create a dynamic ball node alongside its shadow
    osg::ref_ptr<osg::PositionAttitudeTransform> ballTransf = new osg::PositionAttitudeTransform;
    osg::ref_ptr<osg::PositionAttitudeTransform> shadowTransf = new osg::PositionAttitudeTransform;
    CreateBall(ballTransf, shadowTransf);


    // run a controller to allow the user to control the ball with the keyboard
    osg::ref_ptr<BallController> ctrler =  new BallController( ballTransf.get(), &collidedLeft, &collidedRight, &collidedFront, &collidedBack, &collidedBelow, rotation);

    //SQUARE SELECTION
    osg::ref_ptr<osg::PositionAttitudeTransform> selectTransform = new osg::PositionAttitudeTransform;
    osg::ref_ptr<osg::Geode> line = new osg::Geode;

    osg::ref_ptr<osg::ShapeDrawable> cubic = new osg::ShapeDrawable;
    cubic->setShape(new osg::Cylinder(osg::Vec3(0,0,0), 0.5, 15));

    line->addDrawable(cubic.get() );

    osg::Material *materiais = new osg::Material();
    materiais->setDiffuse(osg::Material::FRONT, osg::Vec4(1.0, 1.0, 0, 1.0));
    materiais->setSpecular(osg::Material::FRONT, osg::Vec4(0.0, 0.0, 0.0, 1.0));
    materiais->setAmbient(osg::Material::FRONT, osg::Vec4(0.1, 0.1, 0.1, 1.0));
    materiais->setEmission(osg::Material::FRONT, osg::Vec4(0.0, 0.0, 0.0, 1.0));
    materiais->setShininess(osg::Material::FRONT, 25.0);

    line->getOrCreateStateSet()->setMode ( GL_LIGHT1, osg::StateAttribute::ON ) ;
    line->getOrCreateStateSet()->setAttribute(materiais);
    line->getOrCreateStateSet()->setAttributeAndModes( createPhongShaderProgram().get() );

    selectTransform ->addChild(line);
    selectTransform ->setScale(osg::Vec3(1,1,4));
    selectTransform ->setDataVariance(osg::Object::DYNAMIC );



    //CIRCLE ON POINT
    osg::ref_ptr<osg::PositionAttitudeTransform> SelectPoint = new osg::PositionAttitudeTransform;
    osg::ref_ptr<osg::Geode> sphere = new osg::Geode;

    osg::ref_ptr<osg::ShapeDrawable> aura = new osg::ShapeDrawable;
    aura->setShape(new osg::Cylinder(osg::Vec3(0,0,0), 4, 1));

    sphere->addDrawable(aura.get() );

    osg::Material *materiais1 = new osg::Material();
    materiais1->setDiffuse(osg::Material::FRONT, osg::Vec4(1.0, 1.0, 0, 1.0));
    materiais1->setSpecular(osg::Material::FRONT, osg::Vec4(0.0, 0.0, 0.0, 1.0));
    materiais1->setAmbient(osg::Material::FRONT, osg::Vec4(0.1, 0.1, 0.1, 1.0));
    materiais1->setEmission(osg::Material::FRONT, osg::Vec4(0.0, 0.0, 0.0, 1.0));
    materiais1->setShininess(osg::Material::FRONT, 25.0);

    sphere->getOrCreateStateSet()->setMode ( GL_LIGHT1, osg::StateAttribute::ON ) ;
    sphere->getOrCreateStateSet()->setAttribute(materiais1);
    sphere->getOrCreateStateSet()->setAttributeAndModes( createPhongShaderProgram().get() );

    SelectPoint ->addChild(sphere);
    SelectPoint ->setDataVariance(osg::Object::DYNAMIC );


    // force the perspective camera look at the ball and the shadow
    camera2->addChild( ballTransf );
    //camera2->addChild( shadowTransf );
    camera2->addChild( selectTransform );
    camera2->addChild( SelectPoint );


    //! MIGUEL STUFF


    osgViewer::Viewer viewerOSG;
    viewerOSG.setUpViewInWindow(0, 0, 640, 480);
    viewerOSG.setSceneData( root.get() );


    //! MIGUEL STUFF

    viewerOSG.addEventHandler( ctrler.get() );

    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
    kdtree->setInputCloud (cloud);
    //! MIGUEL STUFF


    while (!viewerPCL->wasStopped ())
    {        
        //! MIGUEL STUFF

        if(isMoving){
            lookingForY = findPointsBellow(ballPath, ballTransf, kdtree, &bellowpoint, cloud);
            //Fazer a procura aqui
            //if(lookingForY.y < 0)
                isPointInObject(lookingForY);
        }
            osg::Vec3 planePos = ballTransf->getPosition();
            //float positionOfSelector = planePos.z() - lookingForY.y - 0.04;

            selectTransform->setPosition(ballTransf->getPosition() + osg::Vec3(0,0,-30));
            
            osg::Vec3d point(planePos.x(), planePos.y(),-lookingForY.y *100 + 2);
            SelectPoint->setPosition(point);
            /*SelectPoint->setPosition(ballTransf->getPosition() + osg::Vec3(0,0,lookingForY.x * 100));
            osg::Vec3 sphereselectorPos = SelectPoint->getPosition();*/
			//SelectPoint->setPosition(osg::Vec3(0,0,lookingForY.y));
			
			
            //! MIGUEL STUFF


			pcl::ModelCoefficients cube_coeff;
			cube_coeff.values.resize (10);
            cube_coeff.values[0] = -SelectPoint->getPosition().x()/100.f;;
            cube_coeff.values[1] = -SelectPoint->getPosition().z()/100.f;;
            cube_coeff.values[2] = -SelectPoint->getPosition().y()/100.f;;
			cube_coeff.values[3] = cube_coeff.values[4] = cube_coeff.values[5] = cube_coeff.values[6] = 0;
            cube_coeff.values[7] = cube_coeff.values[8] = cube_coeff.values[9] = 0.03;
			viewerPCL->removeShape("cube");
			viewerPCL->addCube(cube_coeff, "cube");
		
		    viewerPCL->spinOnce (100);
            viewerOSG.frame();
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));


    }
    
    /*if (ballPath->size() > 0)
    {
        pcl::PCDWriter writer;
        writer.write<pcl::PointXYZRGBA> ("Out/ball_path.pcd", *ballPath, false);
    }*/
}

