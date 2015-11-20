// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#include "skinndetector.h"



void Mixture::print()
{
    Eigen::Matrix3d diag = invCovariance.asDiagonal();

    std::cout << "######################################## MEAN"               << std::endl;        std::cout << mean          << std::endl;
    std::cout << "######################################## INVERSE COVARIANCE" << std::endl;        std::cout << diag          << std::endl;
    std::cout << "######################################## FINAL WEIGHT"       << std::endl;        std::cout << finalWeight   << std::endl << std::endl;
}



double Mixture::computeProbability( cv::Vec3b BGR )
{
    double prob = finalWeight * exp( -0.5 * ( pow((double)BGR(2)-mean(0),2) * invCovariance(0) + // BGR->RGB
                                              pow((double)BGR(1)-mean(1),2) * invCovariance(1) +
                                              pow((double)BGR(0)-mean(2),2) * invCovariance(2)
                                            )
                                   );
    return prob;
}
