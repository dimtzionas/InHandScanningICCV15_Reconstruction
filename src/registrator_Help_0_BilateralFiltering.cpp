// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#include <registrator.h>



void Registrator::my_Bilateral_Filtering( pcl::PointCloud<TYPE_Point_Sensor>::Ptr &cloud_ORGan_P_, double resolution )
{
                                          pcl::PointIndices                                                           removedPointIndices;
                  my_Bilateral_Filtering(                                          cloud_ORGan_P_,        resolution, removedPointIndices );
}
void Registrator::my_Bilateral_Filtering( pcl::PointCloud<TYPE_Point_Sensor>::Ptr &cloud_ORGan_P_, double resolution, pcl::PointIndices &removedPointIndices )
{

        pcl::FastBilateralFilterOMP<TYPE_Point_Sensor> bilFILTER;
        bilFILTER.setInputCloud( cloud_ORGan_P_ );
        bilFILTER.setSigmaS( PARAM_BILATERAL_SigmaS * resolution );
        bilFILTER.setSigmaR( PARAM_BILATERAL_SigmaR * resolution );
        bilFILTER.filter(       *cloud_ORGan_P_ );

        bilFILTER.getRemovedIndices( removedPointIndices );

        ///////////////////////////////////////////////////////////////
        restore_NANs___dueToPCLbugInBILATERAL( cloud_ORGan_P_, false );
        ///////////////////////////////////////////////////////////////
}



void Registrator::restore_NANs___dueToPCLbugInBILATERAL( pcl::PointCloud<TYPE_Point_Sensor>::Ptr& cloud_ORGan_P_, bool shouldPrint_countYesN_NANs )
{
    if (shouldPrint_countYesN_NANs)     std::cout << "restore_NANs___dueToPCLbugInBILATERAL" << "\t\t" << "cloud_ORGan_P_->height = " << cloud_ORGan_P_->height << std::endl;
    if (shouldPrint_countYesN_NANs)     std::cout << "restore_NANs___dueToPCLbugInBILATERAL" << "\t\t" << "cloud_ORGan_P_->width  = " << cloud_ORGan_P_->width  << std::endl;

    double minZ = +std::numeric_limits<float>::infinity();
    double maxZ = -std::numeric_limits<float>::infinity();

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    cv::Mat testNANsIMG = cv::Mat(cloud_ORGan_P_->height,cloud_ORGan_P_->width,CV_8UC1,255);
    ////////////////////////////////////////////////////////////////////////////////////////
    int countNANs_YES = 0;
    int countNANs_NOO = 0;
    //////////////////////
    for     (int iii=0; iii<cloud_ORGan_P_->height; ++iii)
    {   for (int jjj=0; jjj<cloud_ORGan_P_->width;  ++jjj)
        {
            ///////////////////////////////////////////////////
            if (  pcl_isnan( (*cloud_ORGan_P_)(jjj,iii).x )  ||
                  pcl_isnan( (*cloud_ORGan_P_)(jjj,iii).y )  ||
                  pcl_isnan( (*cloud_ORGan_P_)(jjj,iii).z )   )
            ///////////////////////////////////////////////////
            {

                ///////////////////////////////////////////////////////////////////////
                (*cloud_ORGan_P_)(jjj,iii).x = std::numeric_limits<float>::quiet_NaN();
                (*cloud_ORGan_P_)(jjj,iii).y = std::numeric_limits<float>::quiet_NaN();
                (*cloud_ORGan_P_)(jjj,iii).z = std::numeric_limits<float>::quiet_NaN();
                ///////////////////////////////////////////////////////////////////////

                countNANs_YES++;
                testNANsIMG.at<uchar>(iii,jjj) = 0;
            }
            else
            {
                countNANs_NOO++;
            }

            if (minZ > (*cloud_ORGan_P_)(jjj,iii).z)
                minZ = (*cloud_ORGan_P_)(jjj,iii).z;

            if (maxZ < (*cloud_ORGan_P_)(jjj,iii).z)
                maxZ = (*cloud_ORGan_P_)(jjj,iii).z;
        }
    }
    if (shouldPrint_countYesN_NANs)     std::cout << "testNANs - yes NANs" << "\t\t" << countNANs_YES << "\t\t" << "minZ="<<minZ << "\t\t" << "maxZ="<<maxZ << std::endl;
    if (shouldPrint_countYesN_NANs)     std::cout << "testNANs - noo NANs" << "\t\t" << countNANs_NOO << "\t\t" << "minZ="<<minZ << "\t\t" << "maxZ="<<maxZ << std::endl << std::endl;

}
