// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#include <registrator.h>




void Registrator::my_KeyPoint_Detector_ISS3D( const pcl::PointCloud<TYPE_Point_PostPr>::Ptr &cloudIN_PN,
                                                    double                                  &modelResolutionIN,
                                                    pcl::PointCloud<TYPE_Point_PostPr>::Ptr &keyPointCloudOUT_PN    )
{

        pcl::ISSKeypoint3D< TYPE_Point_PostPr, TYPE_Point_PostPr> keyPointDetector;
        pcl::search::KdTree<TYPE_Point_PostPr>::Ptr tree (new pcl::search::KdTree<TYPE_Point_PostPr> );
        keyPointDetector.setSearchMethod(    tree);
        keyPointDetector.setSalientRadius(   PARAM_KEYP_coeffSalientRadius * modelResolutionIN);
        keyPointDetector.setNonMaxRadius(    PARAM_KEYP_coeffNonMaxRadiuss * modelResolutionIN);
        keyPointDetector.setThreshold21(     0.975);
        keyPointDetector.setThreshold32(     0.975);                    // Set the upper bound on the ratio between the third and the second eigenvalue.
        keyPointDetector.setMinNeighbors(    5);                        // Set the minimum number of neighbors that has to be found while applying the non maxima suppression algorithm.
        keyPointDetector.setNumberOfThreads( 4);                        // Initialize the scheduler and set the number of threads to use.

        keyPointDetector.setInputCloud(  cloudIN_PN );
        keyPointDetector.compute(       *keyPointCloudOUT_PN );

        //////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////
        if (keyPointCloudOUT_PN->is_dense == false)
        {

                for (int kkk=keyPointCloudOUT_PN->size()-1; kkk>=0; kkk--)
                {

                        if (   pcl::isFinite( (*keyPointCloudOUT_PN)[kkk] ) == false   ) // A number is finite if it is neither NaN (Not a Number) nor positive or negative infinity.
                        {
                            (*keyPointCloudOUT_PN).erase(  (*keyPointCloudOUT_PN).begin() + kkk  );

                            std::cout << "---- Deleted NAN KeyPoint - " << kkk << std::endl;
                        }

                } // for

        } // if
        //////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////

}


