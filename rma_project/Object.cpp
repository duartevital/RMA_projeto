#include <iostream>

class Object{

public:
    std::string getType(){  return type; }
    std::string getColor(){  return color; }
    int getHeight(){    return height; }
    int getWidth(){    return width; }
    int getRed(){    return red; }
    int getGreen(){    return green; }
    int getBlue(){    return blue; }
    int getArea(){    return area; }

    void setType(std::string type){  this->type=type; }
    void setColor(std::string color){  this->color=color; }
    void setHeight(int height){ this->height=height; }
    void setWidth(int width){ this->width=width; }
    void setRed(int red){ this->red=red; }
    void setGreen(int green){ this->green=green; }
    void setBlue(int blue){ this->blue=blue; }
    void setArea(int area){ this->area=area; }

    std::string toString();

    Object(std::string type, std::string color, int height, int width, int red, int green, int blue, int area);


private:
    std::string type, color;
    int height, width;
    int red, green, blue;
    int area;
};

Object::Object(std::string type, std::string color, int height, int width, int red, int green, int blue, int area){
    this->type=type; this->color=color; this->height=height; this->width=width;
    this->red=red; this->green=green; this->blue=blue;
    this->area=area;
}
std::string Object::toString(){
    std::stringstream s;
    s <<  type << " " << color << " " << height << " " << width << " " << red << " " << green << " " << blue << " " << area;
    return s.str();
}
