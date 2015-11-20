// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#ifndef CAMERASET_H
#define CAMERASET_H

#include <QDebug>
#include <QString>
#include <QFile>
#include <QTextStream>

#include <eigen3/Eigen/Core>

#include <iostream>


struct OpenGL_Stuff
{
    int nearPlane;
    int farPlane;
};


struct cameraMatricesStruct
{
        Eigen::MatrixXd KKK;
        Eigen::MatrixXd RRR;
        Eigen::MatrixXd TTT;
        Eigen::MatrixXd RT_3x4;
        Eigen::MatrixXd RT_4x4;
        Eigen::MatrixXd P_3x4;
};



class CameraSet
{



public:

    CameraSet();
    CameraSet( QString INPUT_FINAL_PATH__Cameras );

    int totalCameras;
    int currentCameraID;

    QVector< cameraMatricesStruct >  cameras;

    bool switchCamera( int newCurrCamID );

    Eigen::Vector2d     project__3D_Vertex__to__2D_Point(const Eigen::Vector3d &vert_3x1, const int camID);


private:

    void setupCameras( QString INPUT_FINAL_PATH__Cameras );
    void print();

};

#endif // CAMERASET_H
