using namespace std;

#include "ObjectAnalyzer.cpp"

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

    analyze(cloud);
}

