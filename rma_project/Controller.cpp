#include <osg/Camera>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgViewer/Viewer>
#include <osg/Texture2D>
#include <osg/MatrixTransform>
#include <osg/GraphicsContext>
#include <osg/Depth>
#include <osg/Material>
#include <osg/Texture2D>
#include <osg/PositionAttitudeTransform>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/BlendFunc>
#include <osg/AlphaFunc>
#include <osgGA/GUIEventHandler>
#include <osg/ShapeDrawable>


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <string>
#include <algorithm>
#include <sstream>

pcl::PointXYZRGBA lookingForY;
bool isMoving = false;
osg::ref_ptr<osg::PositionAttitudeTransform> Selectionaire = new osg::PositionAttitudeTransform;
osg::ref_ptr<osg::PositionAttitudeTransform> QuestSelect = new osg::PositionAttitudeTransform;
Object selectedOBJ; Object questOBJ;
Object found; bool initializedQuest = false;
bool pressed_J = false;

static const char* textureVertexSource = {
    "varying vec3 normal;\n"
    "void main()\n"
    "{\n"
    "    gl_Position = gl_ProjectionMatrix * gl_ModelViewMatrix * gl_Vertex;\n"
    "}\n"
};

static const char* ballVertexSource = {
    "void main()\n"
    "{\n"
    "    gl_TexCoord[0] = gl_MultiTexCoord0;\n"
    "    gl_Position = gl_ProjectionMatrix * gl_ModelViewMatrix * gl_Vertex;\n"
    "}\n"
};

static const char* textureFramentSource = {
    "uniform sampler2D texture;\n"
    "uniform sampler2D textureDepth;\n"
    "void main()\n"
    "{\n"
    "   vec2 uv = vec2(gl_FragCoord.x/640.0, gl_FragCoord.y/480.0);\n"
    "   gl_FragColor = texture2D(texture, uv);\n"
    "   gl_FragDepth = (texture2D(textureDepth, uv)[2]);\n"
    "}\n"
};

static const char* ballFragmentSource = {
    "uniform sampler2D texture;\n"
    "void main()\n"
    "{\n"
    "    gl_FragColor = texture2D(texture, gl_TexCoord[0].st);\n"
    "    gl_FragDepth = (1.0/gl_FragCoord.w)/1000.0;\n"
    "}\n"
};

void newSelect(Object obj){
    Selectionaire ->setScale(osg::Vec3((obj.getWidth() + 1)/2,(obj.getWidth() + 1)/2, 1 ));
    Selectionaire ->setPosition(osg::Vec3(-obj.getCentroid().x*100,-obj.getCentroid().z*100,-obj.getCentroid().y*100));

}

pcl::PointXYZRGBA findPointsBellow(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ballPath, osg::ref_ptr<osg::PositionAttitudeTransform> ballTransf,
                      pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdtree,
                      bool *bellowpoint, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rotated)
{
    pcl::PointXYZRGBA pointy;
    pointy.y = 10000;

    std::vector<int> search_indexes;
    std::vector<float> search_radiuses;

    std::cout << "Started new cycle" << std::endl;
    //45.
    pcl::PointXYZRGBA ball;

    ball.x = -ballTransf->getPosition().x()/100.f;

    ball.y = -ballTransf->getPosition().z()/100.f;

    ball.z = -ballTransf->getPosition().y()/100.f;

    //46.
    pcl::PointXYZRGBA ball_view;

    ball_view.x = -ballTransf->getPosition().x()/100.f;

    ball_view.y = ballTransf->getPosition().z()/100.f;

    ball_view.z = ballTransf->getPosition().y()/100.f;

    ballPath->width++;

    ballPath->push_back(ball_view);

    for(double i =0; i <= 1; i += 0.02){

        //47.
        pcl::PointXYZRGBA ballNeighborPoint;

        ballNeighborPoint.x = ball.x;

        ballNeighborPoint.y = ball.y + i;

        ballNeighborPoint.z = ball.z;

        //48.
        kdtree->radiusSearch (ballNeighborPoint, 0.03, search_indexes, search_radiuses);

        //49.
        if (search_indexes.size() == 0){
            *bellowpoint = false;
        }
        else{
            for(int j= 0; j < search_indexes.size(); ++j) {
                if(cloud_rotated->points[ search_indexes[j] ].y < pointy.y){
                    pointy = cloud_rotated->points [search_indexes[j] ];
                }
            }
            *bellowpoint = true;
            break;
        }
    }
    std::cout << "point found x: " << pointy.x << " " << std::endl;
    std::cout << "point found y: " << pointy.y << " " << std::endl;
    std::cout << "point found z: " << pointy.z << " " << std::endl;
    isMoving = false;
    return pointy;
}



class BallCallback : public osg::NodeCallback
{
public:
    BallCallback(osg::PositionAttitudeTransform *_shadowTransf) :
    shadowTransf(_shadowTransf){}

    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv);
protected:
    osg::PositionAttitudeTransform* shadowTransf;
};

void BallCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{

    osg::PositionAttitudeTransform* ballTransf = static_cast<osg::PositionAttitudeTransform*>(node);
    osg::Vec3 v = ballTransf->getPosition();
    shadowTransf->setPosition(osg::Vec3(v.x(), v.y(), v.z()-3));
}


void	CreateBall(osg::ref_ptr<osg::PositionAttitudeTransform> ballTransf,
                   osg::ref_ptr<osg::PositionAttitudeTransform> shadowTransf)
{
    osg::ref_ptr<osg::Shader> vertShader2 =
    new osg::Shader( osg::Shader::VERTEX, ballVertexSource );
    osg::ref_ptr<osg::Shader> fragShader2 =
    new osg::Shader( osg::Shader::FRAGMENT, ballFragmentSource );
    osg::ref_ptr<osg::Program> program2 = new osg::Program;
    program2->addShader( fragShader2.get() );
    program2->addShader( vertShader2.get() );

    osg::Texture2D *ballTexture = new osg::Texture2D;
    ballTexture->setDataVariance(osg::Object::DYNAMIC);
    ballTexture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
    ballTexture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
    ballTexture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP);
    ballTexture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP);
    osg::Image *ballImage = osgDB::readImageFile("../Data/PlatonicSurface_Color.jpg");
    ballTexture->setImage(ballImage);

    osg::ref_ptr<osg::Node> ballNode = osgDB::readNodeFile("../Data/soccer_ball.obj");
    ballNode->getOrCreateStateSet()->setMode( GL_LIGHT1, osg::StateAttribute::ON );
    osg::StateSet *ballStateSet = ballNode->getOrCreateStateSet();
    ballStateSet->setTextureAttributeAndModes(0, ballTexture, osg::StateAttribute::ON);
    ballStateSet->setAttributeAndModes( program2.get() );
    ballStateSet->addUniform(new osg::Uniform("texture", 0 ));

    osg::Image *shadowImage = osgDB::readImageFile("../Data/shadow.png");
    osg::ref_ptr<osg::Texture2D> shadowTexture = new osg::Texture2D(shadowImage);
    osg::ref_ptr<osg::StateSet> shadowStateSet = new osg::StateSet();
    shadowStateSet->setTextureAttributeAndModes(0, shadowTexture, osg::StateAttribute::ON);
    shadowStateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    osg::ref_ptr<osg::BlendFunc> texture_blending_function = new osg::BlendFunc();
    shadowStateSet->setAttributeAndModes(texture_blending_function.get(), osg::StateAttribute::ON);
    shadowStateSet->setAttributeAndModes( program2.get() );
    shadowStateSet->addUniform(new osg::Uniform("texture", 0 ));

    osg::ref_ptr<osg::Drawable> shadowQuad =
    osg::createTexturedQuadGeometry( osg::Vec3(0.5f, 0.5f, 0.0f), osg::Vec3(-1.0f, 0.0f, 0.0f), osg::Vec3(0.0f, -1.0f, 0.0f) );
    shadowQuad->setStateSet(shadowStateSet.get());
    osg::ref_ptr<osg::Geode> shadowGeode = new osg::Geode;
    shadowGeode->addDrawable( shadowQuad.get() );

    shadowTransf->addChild(shadowGeode);
    shadowTransf->setPosition(osg::Vec3(0,-100,2));
    shadowTransf->setDataVariance( osg::Object::DYNAMIC );
    shadowTransf->setScale(osg::Vec3(8,8,2));

    ballTransf->addChild(ballNode);
    ballTransf->setScale(osg::Vec3(.05,.05,.05));
    ballTransf->setPosition(osg::Vec3(0,-100,5));
    ballTransf->setAttitude(osg::Quat(0.0f, osg::Y_AXIS, 0.0f, osg::Z_AXIS, 0.0f, osg::X_AXIS));
    ballTransf->setDataVariance( osg::Object::DYNAMIC );

    static_cast<osg::Node*>(ballTransf)->setUpdateCallback( new BallCallback(shadowTransf.get()) );

}


class BallController : public osgGA::GUIEventHandler
{
public:
    BallController( osg::PositionAttitudeTransform* node, bool *_collidedLeft, bool *_collidedRight,
                   bool *_collidedFront, bool *_collidedBack, bool *_collidedBelow, double _rotation)
    : _ball(node), collidedLeft(_collidedLeft),
    collidedRight(_collidedRight), collidedFront(_collidedFront), collidedBack(_collidedBack), collidedBelow(_collidedBelow), rotation(_rotation) {}
    virtual bool handle( const osgGA::GUIEventAdapter& ea,
                        osgGA::GUIActionAdapter& aa );
protected:
    osg::ref_ptr<osg::PositionAttitudeTransform> _ball;
    bool *collidedLeft, *collidedRight, *collidedFront, *collidedBack, *collidedBelow;
    double rotation;
};

bool BallController::handle( const osgGA::GUIEventAdapter& ea,
                            osgGA::GUIActionAdapter& aa )
{
    if ( !_ball )
        return false;

    //39.
    osg::Vec3 v = _ball->getPosition();

    osg::Quat q = _ball->getAttitude();

    switch ( ea.getEventType() )
    {

        case osgGA::GUIEventAdapter::KEYDOWN:
            switch ( ea.getKey() )
        {
            case 'j': case 'J':
                    found = isPointInObject(lookingForY);
                    if(found.isInitialized())
                        newSelect(found);
                    pressed_J = true;
                break;
            case 'q': case 'Q':
                    v.z() += 1.0;
                break;
            case 'e': case 'E':
                if (!(*collidedBelow)){
                    v.z() -= 1.0;
                }
                break;
            case 'a': case 'A':
                if (!(*collidedLeft)){

                    q *= osg::Quat(0, osg::X_AXIS, 0, osg::Y_AXIS, osg::PI/6, osg::Z_AXIS);
                    rotation = rotation + osg::PI/6;

                }
                break;
            case 'd': case 'D':
                if (!(*collidedRight)){

                    q *= osg::Quat(0, osg::X_AXIS, 0, osg::Y_AXIS, -osg::PI/6, osg::Z_AXIS);
                    rotation = rotation - osg::PI/6;

                }
                break;
            case 'w': case 'W':
                if (!(*collidedFront)){
                    v.x() -= cos(0)*sin(rotation);
                    v.y() += cos(0)*cos(rotation);
                    isMoving= true;
                }
                break;
            case 's': case 'S':
                if (!(*collidedBack)){
                    v.x() += cos(0)*sin(rotation);
                    v.y() -= cos(0)*cos(rotation);
                    isMoving= true;
                }
                break;

            default:
                break;
        }

            //42.
            _ball->setPosition(v);

            _ball->setAttitude(q);

            break;
        default:
            break;
    }
    return false;

}






void estimateCameraPose(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in, double *pitch, double *roll, double *height)
{
    //4
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_centre (new pcl::PointCloud<pcl::PointXYZRGBA>);

    cloud_centre->height=1; int step = 2; long index;

    for (int row=0;row<cloud_in->height/4;row+=step){

        for (int col=cloud_in->width/2-50;col<cloud_in->width/2+50;col+=step){

            index = (cloud_in->height-row-1)*cloud_in->width + col;

            cloud_centre->points.push_back(cloud_in->points[index]);

            cloud_centre->width++;

        }

    }

    //5.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;

    seg.setModelType (pcl::SACMODEL_PLANE);

    seg.setMethodType (pcl::SAC_RANSAC);

    seg.setMaxIterations (10000);

    seg.setDistanceThreshold (0.05);

    seg.setInputCloud (cloud_centre);

    seg.segment (*inliers, *coefficients);


    //6.
    double c_a = coefficients->values[0];

    double c_b = coefficients->values[1];

    double c_c = coefficients->values[2];

    double c_d = coefficients->values[3];

    std::cout << "Coefficients a: " << c_a << " b: " << c_b << " c: " << c_c << " d: " << c_d << "." << std::endl;

    //7.
    double norm = sqrt(c_a*c_a+c_b*c_b+c_c*c_c);

    c_a = c_a/norm;

    c_b = c_b/norm;

    c_c = c_c/norm;

    std::cout << "Coefficients a: " << c_a << " b: " << c_b << " c: " << c_c << " d: " << c_d << " norm: " << norm << std::endl;

    //8.A.
    if (c_c<0){

        c_a*=-1; c_b*=-1; c_c*=-1;

    }

    //8.
    *pitch = asin(c_c);

    *roll = -acos(c_b/cos(*pitch));

    *height = fabs(c_d);

    //8.B.

    if (c_a<0)

        (*roll) *= -1;


    //9.
    std::cout << "Camera pitch: " << *pitch * 180/M_PI << " [deg]; Camera roll: " << *roll * 180/M_PI << " [deg]." << std::endl;

    std::cout << "Camera height: " << *height << std::endl;

}


void rotatePointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in,
                      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rotated,
                      double camera_pitch, double camera_roll, double camera_height)
{

    //11.
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_voxelised (new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::VoxelGrid<pcl::PointXYZRGBA> voxel_grid;

    voxel_grid.setInputCloud (cloud_in);

    voxel_grid.setLeafSize (0.01, 0.01, 0.01);

    voxel_grid.filter (*cloud_voxelised);

    //12.
    Eigen::Affine3f t1 = pcl::getTransformation (0.0, -camera_height, 0.0, 0.0, 0.0, 0);

    Eigen::Affine3f t2 = pcl::getTransformation (0.0, 0.0, 0.0, -camera_pitch, 0.0, 0.0);

    Eigen::Affine3f t3 = pcl::getTransformation (0.0, 0.0, 0.0, 0.0, 0.0, -camera_roll);

    pcl::transformPointCloud(*cloud_voxelised, *cloud_rotated, t1*t2*t3);

    /*pcl::PCDWriter writer;

    writer.write<pcl::PointXYZRGBA> ("Out/out_rotated.pcd", *cloud_rotated, false);*/

}

void createImageFromPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in, unsigned char* data, unsigned char* dataDepth)
{
    long index1, index2;

    std::cout << "cloud height == " << cloud_in->height << std::endl;
    std::cout << "cloud width == " << cloud_in->width << std::endl;

    for (int row=0;row<cloud_in->height;row++)
    {
        for (int col=0;col<cloud_in->width;col++)
        {
            index1 = 3*(row*cloud_in->width + col);
            index2 = (cloud_in->height-row-1)*cloud_in->width + col;

            //15.
    data[index1] = cloud_in->points[index2].r;

    data[index1+1] = cloud_in->points[index2].g;

    data[index1+2] = cloud_in->points[index2].b;

            //16.
    if (isnan(cloud_in->points[index2].x)){

        dataDepth[index1] = dataDepth[index1+1] = dataDepth[index1+2] = 0;

    }else{

        dataDepth[index1] = ((cloud_in->points[index2].x+2)/6.0)*255.0;

        dataDepth[index1+1] = ((cloud_in->points[index2].y+2)/6.0)*255.0;

        dataDepth[index1+2] = (cloud_in->points[index2].z/10.0)*255.0;

}



        }
    }
}

void createVirtualCameras(osg::ref_ptr<osg::Camera> camera1, osg::ref_ptr<osg::Camera> camera2, osg::ref_ptr<osg::Geode> orthoTexture,
                          double camera_pitch, double camera_roll, double camera_height)
{

    //25.
    camera1->setCullingActive( false );

    camera1->setClearMask( 0 );

    camera1->setAllowEventFocus( false );

    camera1->setReferenceFrame( osg::Camera::ABSOLUTE_RF );

    camera1->setRenderOrder( osg::Camera::POST_RENDER, 0);

    camera1->setProjectionMatrix( osg::Matrix::ortho(0.0, 1.0, 0.0, 1.0, 0.5 , 1000) );

    camera1->addChild( orthoTexture.get() );

    osg::StateSet* ss = camera1->getOrCreateStateSet();

    ss->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

    //26.
    camera2->setCullingActive( false );

    camera2->setClearMask( 0 );

    camera2->setAllowEventFocus( false );

    camera2->setReferenceFrame( osg::Camera::ABSOLUTE_RF );

    camera2->setRenderOrder( osg::Camera::POST_RENDER, 1);

    camera2->setProjectionMatrixAsPerspective(50, 640./480., 0.5, 1000);

    //27.
    osg::Matrixd cameraRotation2;

    cameraRotation2.makeRotate(osg::DegreesToRadians(camera_pitch*180/M_PI), osg::Vec3(1,0,0),

                                                         osg::DegreesToRadians(0.0), osg::Vec3(0,1,0),

                                                         osg::DegreesToRadians(0.0), osg::Vec3(0,0,1));

    //28.
    osg::Matrixd cameraRotation3;

    cameraRotation3.makeRotate(osg::DegreesToRadians(0.0), osg::Vec3(1,0,0),

    osg::DegreesToRadians(camera_roll*180/M_PI), osg::Vec3(0,1,0),

    osg::DegreesToRadians(0.0), osg::Vec3(0,0,1));

    //29.
    osg::Matrixd cameraTrans;

    cameraTrans.makeTranslate(0, 0, camera_height*100.0);

    //30.
    osg::Matrix matrix_view;

    matrix_view.makeLookAt(osg::Vec3(0, 0, 0), osg::Vec3(0, -1, 0), osg::Vec3(0, 0, 1));

    osg::Matrixd cameraRotation;

    cameraRotation.makeRotate(osg::DegreesToRadians(180.0), osg::Vec3(0,0,1), osg::DegreesToRadians(-90.0), osg::Vec3(1,0,0), osg::DegreesToRadians(0.0), osg::Vec3(0,1,0) );



    //31.
    camera2->setViewMatrix(osg::Matrix::inverse(cameraRotation*cameraRotation3*cameraRotation2*cameraTrans));

    //32.
    camera1->setViewport(0, 0, 640, 480);

    camera2->setViewport(0, 0, 640, 480);
}



osg::ref_ptr<osg::Program> createPhongShaderProgram(){
    static const char* phongVertexSource = {
        "varying vec3 varnormal;\n"
        "varying vec4 varposition;\n"
        "void main()\n"
        "{\n"
        "varnormal = normalize(gl_NormalMatrix * gl_Normal);\n"
        "varposition =  gl_ModelViewMatrix * (gl_Vertex);\n"
        "gl_Position = gl_ModelViewProjectionMatrix * (gl_Vertex);\n"
        "}\n"
    };
    static const char* phongFragmentSource = {
        "varying vec3 varnormal;\n"
        "varying vec4 varposition;\n"
        "void main(void){\n"
            "vec3 normal, lightDir, viewVector, halfVector;\n"
            "vec4 diffuse, ambient, globalAmbient, specular;\n"
            "float NdotL,NdotHV;\n"
            "normal = normalize(varnormal);   \n"
            "lightDir = normalize(vec3(gl_LightSource[0].position.xyz - varposition.xyz)); \n"
            "NdotL = max(dot(normal, lightDir), 0.0);        \n"
            "diffuse = gl_FrontMaterial.diffuse * gl_LightSource[0].diffuse;  \n"
            "ambient = gl_FrontMaterial.ambient * gl_LightSource[0].ambient;  \n"
            "NdotHV = max(dot(normal, normalize(gl_LightSource[0].halfVector.xyz)),0.0);      \n"
            "specular = gl_FrontMaterial.specular * gl_LightSource[0].specular * pow(NdotHV,gl_FrontMaterial.shininess);  \n"
            "gl_FragColor = clamp(NdotL * diffuse + ambient + specular, 0.0, 1.0);\n"
            "gl_FragDepth = (1.0/gl_FragCoord.w)/1000.0;\n"
        "}\n"
    };
    osg::ref_ptr<osg::Shader> phongVertShader = new osg::Shader( osg::Shader::VERTEX, phongVertexSource );
    osg::ref_ptr<osg::Shader> phongFragShader = new osg::Shader( osg::Shader::FRAGMENT, phongFragmentSource );
    osg::ref_ptr<osg::Program> phongShaderProgram = new osg::Program;
    phongShaderProgram->addShader( phongVertShader.get() );
    phongShaderProgram->addShader( phongFragShader.get() );
    return phongShaderProgram;
}




void	createOrthoTexture(osg::ref_ptr<osg::Geode> orthoTextureGeode,
                           unsigned char* data, unsigned char* dataDepth, int width, int height)
{
    //18.
    osg::Image *image = new osg::Image;

    osg::Image *imageDepth = new osg::Image;

    image->setOrigin(osg::Image::TOP_LEFT);

    imageDepth->setOrigin(osg::Image::TOP_LEFT);

    image->setImage(width, height, 1 , GL_RGB, GL_RGB, GL_UNSIGNED_BYTE,data, osg::Image::NO_DELETE);

    imageDepth->setImage(width, height, 1 , GL_RGB, GL_RGB, GL_UNSIGNED_BYTE, dataDepth, osg::Image::NO_DELETE);

    //19.
    osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D;

    osg::ref_ptr<osg::Texture2D> texture2 = new osg::Texture2D;

    texture->setImage( image );

    texture2->setImage(imageDepth);

    //20.
    osg::ref_ptr<osg::Drawable> quad = osg::createTexturedQuadGeometry( osg::Vec3(), osg::Vec3(1.0f, 0.0f, 0.0f), osg::Vec3(0.0f, 1.0f, 0.0f) );

    quad->getOrCreateStateSet()->setTextureAttributeAndModes(0, texture.get() );

    quad->getOrCreateStateSet()->setTextureAttributeAndModes(1, texture2.get() );

    //21.
    osg::ref_ptr<osg::Shader> vertShader = new osg::Shader( osg::Shader::VERTEX, textureVertexSource );

    osg::ref_ptr<osg::Shader> fragShader =  new osg::Shader( osg::Shader::FRAGMENT, textureFramentSource );

    osg::ref_ptr<osg::Program> program = new osg::Program;

    program->addShader( fragShader.get() );

    program->addShader( vertShader.get() );

    //22.
    quad->setDataVariance(osg::Object::DYNAMIC);

    quad->getOrCreateStateSet()->setAttributeAndModes( program.get() );

    quad->getOrCreateStateSet()->addUniform(new osg::Uniform("texture", 0 ));

    quad->getOrCreateStateSet()->addUniform(new osg::Uniform("textureDepth", 1 ));

    orthoTextureGeode->addDrawable( quad.get() );
}

//creating square
osg::ref_ptr<osg::PositionAttitudeTransform> createSquare(){
    osg::ref_ptr<osg::PositionAttitudeTransform> selectTransform = new osg::PositionAttitudeTransform;
    osg::ref_ptr<osg::Geode> line = new osg::Geode;

    osg::ref_ptr<osg::ShapeDrawable> cubic = new osg::ShapeDrawable;
    cubic->setShape(new osg::Cylinder(osg::Vec3(0,0,0), 0.2, 15));

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
}




/*
void detectCollisions(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ballPath, osg::ref_ptr<osg::PositionAttitudeTransform> ballTransf,
                      pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdtree,
                      bool *collidedLeft, bool *collidedRight, bool *collidedFront, bool *collidedBack, bool *collidedBelow)
{

    std::vector<int> search_indexes;
    std::vector<float> search_radiuses;


    //45.
    pcl::PointXYZRGBA ball;

    ball.x = -ballTransf->getPosition().x()/100.f;

    ball.y = -ballTransf->getPosition().z()/100.f;

    ball.z = -ballTransf->getPosition().y()/100.f;

    //46.
    pcl::PointXYZRGBA ball_view;

    ball_view.x = -ballTransf->getPosition().x()/100.f;

    ball_view.y = ballTransf->getPosition().z()/100.f;

    ball_view.z = ballTransf->getPosition().y()/100.f;

    ballPath->width++;

    ballPath->push_back(ball_view);

    //47.
    pcl::PointXYZRGBA ballNeighborPoint;

    ballNeighborPoint.x = ball.x - 0.05;

    ballNeighborPoint.y = ball.y - 0.05;

    ballNeighborPoint.z = ball.z;

    //48.
    kdtree->radiusSearch (ballNeighborPoint, 0.05, search_indexes, search_radiuses);


    //49.
    if (search_indexes.size() == 0)

        *collidedLeft = false;

        else

        *collidedLeft = true;

    }
*/


void createGeneralSelectionCircle(osg::ref_ptr<osg::PositionAttitudeTransform> plane_transf){
    //Selection Sphere

    osg::ref_ptr<osg::Geode> pointer = new osg::Geode;

    osg::ref_ptr<osg::ShapeDrawable> aura = new osg::ShapeDrawable;
    //aura->setShape(new osg::Cylinder(osg::Vec3(obj.getCentroid().x,obj.getCentroid().y,obj.getCentroid().z), obj.getWidth() + 2 , 0.5));
    aura->setShape(new osg::Cylinder(osg::Vec3(0,0,0), 2 , 0.2));

    pointer->addDrawable(aura.get() );

    osg::Material *materiais = new osg::Material();
    materiais->setDiffuse(osg::Material::FRONT, osg::Vec4(0.1, 1, 0.1, 1));
    materiais->setSpecular(osg::Material::FRONT, osg::Vec4(0.0, 0.0, 0.0, 1.0));
    materiais->setAmbient(osg::Material::FRONT, osg::Vec4(0.1, 0.1, 0.1, 1.0));
    materiais->setEmission(osg::Material::FRONT, osg::Vec4(0.0, 0.0, 0.0, 1.0));
    materiais->setShininess(osg::Material::FRONT, 25.0);

    pointer->getOrCreateStateSet()->setMode ( GL_LIGHT1, osg::StateAttribute::ON ) ;
    pointer->getOrCreateStateSet()->setAttribute(materiais);
    pointer->getOrCreateStateSet()->setAttributeAndModes( createPhongShaderProgram().get() );

    Selectionaire ->addChild(pointer);
    Selectionaire ->setDataVariance(osg::Object::DYNAMIC );
    Selectionaire ->setPosition(plane_transf->getPosition());
    Selectionaire ->setScale(osg::Vec3(0,0,0));
}


void createMoveQuestCircle(Object obj){
    //Selection Sphere
    if(!initializedQuest){
        osg::ref_ptr<osg::Geode> pointer = new osg::Geode;

        osg::ref_ptr<osg::ShapeDrawable> aura = new osg::ShapeDrawable;
        //aura->setShape(new osg::Cylinder(osg::Vec3(obj.getCentroid().x,obj.getCentroid().y,obj.getCentroid().z), obj.getWidth() + 2 , 0.5));
        aura->setShape(new osg::Cylinder(osg::Vec3(0,0,0), 2 , 0.2));

        pointer->addDrawable(aura.get() );

        osg::Material *materiais = new osg::Material();
        materiais->setDiffuse(osg::Material::FRONT, osg::Vec4(0.1,0.1,1,1));
        materiais->setSpecular(osg::Material::FRONT, osg::Vec4(0.0, 0.0, 0.0, 1.0));
        materiais->setAmbient(osg::Material::FRONT, osg::Vec4(0.1, 0.1, 0.1, 1.0));
        materiais->setEmission(osg::Material::FRONT, osg::Vec4(0.0, 0.0, 0.0, 1.0));
        materiais->setShininess(osg::Material::FRONT, 25.0);

        pointer->getOrCreateStateSet()->setMode ( GL_LIGHT1, osg::StateAttribute::ON ) ;
        pointer->getOrCreateStateSet()->setAttribute(materiais);
        pointer->getOrCreateStateSet()->setAttributeAndModes( createPhongShaderProgram().get() );

        QuestSelect ->addChild(pointer);
        QuestSelect ->setDataVariance(osg::Object::DYNAMIC );
        QuestSelect ->setScale(osg::Vec3((obj.getWidth())/2,(obj.getWidth())/2, 0.9 ));
        QuestSelect ->setPosition(osg::Vec3(-obj.getCentroid().x*100,-obj.getCentroid().z*100,-obj.getCentroid().y*100 - obj.getHeight()/2));
        initializedQuest = true;
    }else{
        QuestSelect ->setScale(osg::Vec3((obj.getWidth() + 1)/2,(obj.getWidth() + 1)/2, 1 ));
        QuestSelect ->setPosition(osg::Vec3(-obj.getCentroid().x*100,-obj.getCentroid().z*100,-obj.getCentroid().y*100 - obj.getHeight()/2));
    }
}

