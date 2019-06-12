using namespace std;
#include "Object.cpp"
#include "TestDetails.cpp"
#include "Controller.cpp"
#include "Question.cpp"
#include "Game.cpp"

int main(int argsc, char** argsv)
{
    // create a point cloud to keep track of the ball's path
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ballPath (new pcl::PointCloud<pcl::PointXYZRGBA>);
    ballPath->height = 1;



    // ball dynamics state variables
    bool collidedLeft = false, collidedRight = false, collidedFront = false, collidedBack = false, collidedBelow = false, bellowpoint = false;
    float rotation = 0;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    //Load file to PointCloud
    if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(argsv[1], *cloud) == -1){
        PCL_ERROR ("Couldn't read file. \n");
        return(-1);
    }

    double camera_pitch, camera_roll, camera_height;
    estimateCameraPose(cloud, &camera_pitch, &camera_roll, &camera_height);



    // go through the point cloud and generate a RGB image and a range image
    int size = cloud->height*cloud->width*3;
    unsigned char* data = (unsigned char*)calloc(size,sizeof(unsigned char));
    unsigned char* dataDepth = (unsigned char*)calloc(size,sizeof(unsigned char));
    createImageFromPointCloud(cloud, data, dataDepth);
    // create a texture from the RGB image and use depth data to fill the z-buffer
    osg::ref_ptr<osg::Geode> orthoTextureGeode = new osg::Geode;
    createOrthoTexture(orthoTextureGeode, data, dataDepth, cloud->width, cloud->height);



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
    Question question = getRandomQuestion();

    //Text da pergunta
    osg::ref_ptr<osg::Geode> textGeode = new osg::Geode;
    osgText::Text* question_txt = createText(osg::Vec3(50.0f, 650.0f, 0.0f), question.getQuestion(), 20.0f);
    question_txt->setDataVariance(osg::Object::DYNAMIC);
    textGeode->addDrawable(question_txt);
    //Text da Pontuação
    osg::ref_ptr<osg::Geode> scoreGeode = new osg::Geode;
    int score = 0;
    std::string score_str("Score: ");
    std::stringstream score_stream;
    score_stream << score_str << score;
    osgText::Text* score_txt = createText(osg::Vec3(50.0f, 50.0f, 0.0f), score_stream.str(), 30.0f);
    score_txt->setDataVariance(osg::Object::DYNAMIC);
    scoreGeode->addDrawable(score_txt);

    osg::Camera* hud_camera = createHUDCamera(0, 1024, 0, 768);
    hud_camera->addChild(textGeode.get());
    hud_camera->addChild(scoreGeode.get());
    hud_camera->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);


    // create an orthographic camera imaging the texture and a perspective camera based on the camera's pose and intrinsics
    osg::ref_ptr<osg::Camera> camera1 = new osg::Camera;
    osg::ref_ptr<osg::Camera> camera2 = new osg::Camera;
    createVirtualCameras(camera1, camera2, orthoTextureGeode, camera_pitch, camera_roll, camera_height);



    //Setting OSG Visualizer
    osg::ref_ptr<osg::Group> root = new osg::Group;
    root->addChild( camera1.get() );
    root->addChild( camera2.get() );
    root->addChild( hud_camera);



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


    //creation of selection circle
    createGeneralSelectionCircle(ballTransf);


    // force the perspective camera look at the ball and the shadow
    camera2->addChild( ballTransf );
    camera2->addChild(Selectionaire);
    camera2->addChild( selectTransform );
    camera2->addChild( SelectPoint );
    camera2->addChild(QuestSelect);





    osgViewer::Viewer viewerOSG;
    viewerOSG.setUpViewInWindow(0, 0, 640, 480);
    viewerOSG.setSceneData( root.get() );




    viewerOSG.addEventHandler( ctrler.get() );

    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
    kdtree->setInputCloud (cloud);



    while (!viewerPCL->wasStopped ())
    {


        if(isMoving){
            lookingForY = findPointsBellow(ballPath, ballTransf, kdtree, &bellowpoint, cloud);
        }
        if(pressed_J){
            //Fazer a procura aqui
            Object obj = isPointInObject(lookingForY);
            if(obj.isInitialized() && question.isCorrectObject(obj.getClusterIndice())){
                QuestSelect ->setScale(osg::Vec3(0,0,0));
                Selectionaire ->setScale(osg::Vec3(0,0,0));
                question = getRandomQuestion();
                question_txt->setText(question.getQuestion());
                score++;
                std::stringstream new_score_stream;
                new_score_stream << score_str << score;
                score_txt->setText(new_score_stream.str());
                std::cout << "FOUND CORRECT OBJECT !!" << std::endl;
            }
            pressed_J = false;
        }

            osg::Vec3 planePos = ballTransf->getPosition();

            selectTransform->setPosition(ballTransf->getPosition() + osg::Vec3(0,0,-30));

            osg::Vec3d point(planePos.x(), planePos.y(),-lookingForY.y *100 + 2);
            SelectPoint->setPosition(point);



            pcl::ModelCoefficients cube_coeff;
            cube_coeff.values.resize (10);
            cube_coeff.values[0] = -ballTransf->getPosition().x()/100.f;;
            cube_coeff.values[1] = -ballTransf->getPosition().z()/100.f;;
            cube_coeff.values[2] = -ballTransf->getPosition().y()/100.f;;
            cube_coeff.values[3] = cube_coeff.values[4] = cube_coeff.values[5] = cube_coeff.values[6] = 0;
            cube_coeff.values[7] = cube_coeff.values[8] = cube_coeff.values[9] = 0.03;
            viewerPCL->removeShape("cube");
            viewerPCL->addCube(cube_coeff, "cube");

            viewerPCL->spinOnce (100);
            viewerOSG.frame();
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));


    }
}

