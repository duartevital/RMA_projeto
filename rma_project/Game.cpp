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


Question getRandomQuestion(){
    int r1 = rand() % 7 + 1;
    int r2 = rand() % 2 + 1;

    Question question(r1, r2);

    return question;
}

