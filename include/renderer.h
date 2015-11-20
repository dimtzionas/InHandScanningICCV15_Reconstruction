// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#ifndef RENDERER_H
#define RENDERER_H

#include <pcl/visualization/pcl_visualizer.h>

#include <sequence.h>

class Renderer
{
public:
    Renderer();
    Renderer( Sequence* sequence );
              Sequence* sequence;

    void renderMeshes_2_PCL( boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, int animNumb, bool renderHands, bool renderObject, bool renderNormals );
    void renderSkeletons(    boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, int animNumb );
};

#endif // RENDERER_H
