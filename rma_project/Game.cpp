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
#include <string>
#include <stdlib.h>

//std::vector<Object> objects_in_file;

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

    int r = rand() % question_vec.size();
    return question_vec[r];
}

