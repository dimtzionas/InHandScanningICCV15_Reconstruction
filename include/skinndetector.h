// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#ifndef SKINNDETECTOR_H
#define SKINNDETECTOR_H


#include <QVector>
#include <QString>

#include <eigen3/Eigen/Core>

#include <iostream>
#include <opencv2/opencv.hpp>



class Mixture
{
public:
    Eigen::Vector3d mean;
    Eigen::Vector3d invCovariance; // inverse of the diagonal Covariance matrix
    double          finalWeight;   // mixture weight / ( (2*pi)^3/2 * sqrt( |cov| ) ); // see page 10 @ the paper "Statistical Color Models with Application to Skin Detection" by M.Jones and J.Rehg

    double computeProbability( cv::Vec3b pixelBGRB );

    void print();
};



class SkinnDetector
{
public:
    SkinnDetector();

    int    PARAM_NUMBBB_Mixture_Components;
    double PARAM_THRESH_Ratio;

    QVector<Mixture> modelSkinn_POS;
    QVector<Mixture> modelSkinn_NEG;

    void createSkinModel_CUSTOM();
    void createSkinModel_JONES_REHG();

    void    SET_PARAM_THRESH_Ratio( double PARAM_THRESH_Ratio_IN );


    cv::Mat compute_SkinnBinMap_4_Image( const cv::Mat   &currFrame_RGB_RAW_CV, const cv::Mat &currFrame_DDD_RAW_CV, double PARAM_THRESH_Ratio_IN, bool skipInvalidDepthFLAG, bool printFLAG, bool displayFLAG );
    cv::Mat compute_LogRatioMap_4_Image( const cv::Mat   &currFrame_RGB_RAW_CV, const cv::Mat &currFrame_DDD_RAW_CV,                               bool skipInvalidDepthFLAG, bool printFLAG, bool displayFLAG );
    double  compute_LogRatioMap_4_Pixel( const cv::Vec3b  pixelBGR,                                                                                                           bool printFLAG                   );
};

#endif // SKINNDETECTOR_H
