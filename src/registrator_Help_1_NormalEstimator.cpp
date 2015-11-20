// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#include <registrator.h>





void Registrator::my_Normal_Estimator( const pcl::PointCloud<TYPE_Point_Sensor>::Ptr &cloud_ORGan_P_,        // IN
                                             pcl::PointCloud<TYPE_Point_PostPr>::Ptr &cloud_UNorg_PN,        // OUT
                                       const QString                                 &syntheticORrealistic,
                                       const bool                                    &FLAG_kickOut_pointsWith_CoordNAN_NormalNAN )
{

        Eigen::Vector4f                          centroid;
        pcl::compute3DCentroid( *cloud_ORGan_P_, centroid );

        pcl::PointCloud<TYPE_Point_Sensor>::Ptr  cloud_ORGan_p_(new pcl::PointCloud<TYPE_Point_Sensor>);
        pcl::copyPointCloud(  *cloud_ORGan_P_,  *cloud_ORGan_p_ );


        /////////////////////////////////////////////////////////////////////////////////////
        pcl::PointCloud<pcl::Normal>::Ptr cloud_ORGan__N( new pcl::PointCloud<pcl::Normal> );
        /////////////////////////////////////////////////////////////////////////////////////


        if (syntheticORrealistic.contains("realistic"))
        {
                                                                                            for (int ppp=0; ppp<cloud_ORGan_p_->points.size(); ppp++) // combats "is not from a projective device"
                                                                                            {
                                                                                                cloud_ORGan_p_->points[ppp].x /= 1000;
                                                                                                cloud_ORGan_p_->points[ppp].y /= 1000;
                                                                                                cloud_ORGan_p_->points[ppp].z /= 1000;
                                                                                            }
                pcl::IntegralImageNormalEstimation<TYPE_Point_Sensor,pcl::Normal> ne;
                ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
                ne.setMaxDepthChangeFactor( PARAM_NORMAL_MaxDepthChangeFactor );  // The depth change threshold for computing object borders.
                ne.setNormalSmoothingSize(  PARAM_NORMAL_NormalSmoothingSize  );  // Set the normal smoothing size = the size of the area used to smooth normals
                ne.setViewPoint(   0,0,0  );
                ne.setInputCloud(  cloud_ORGan_p_ );
                ne.compute(       *cloud_ORGan__N );
        }
        else
        {
                pcl::NormalEstimationOMP<TYPE_Point_Sensor,pcl::Normal>  ne;
                pcl::search::KdTree<TYPE_Point_Sensor>::Ptr tree (new pcl::search::KdTree<TYPE_Point_Sensor>);
                ne.setSearchMethod( tree );
                ne.setRadiusSearch( PARAM_NORMAL_NormalSmoothingSize );


                if (syntheticORrealistic.contains("realistic"))     ne.setViewPoint(  0,0,0  );
                else                                                ne.setViewPoint(  centroid[0],
                                                                                      centroid[1],
                                                                                      centroid[2]);
                ne.setInputCloud(  cloud_ORGan_P_ );
                ne.compute(       *cloud_ORGan__N );

                if (syntheticORrealistic == "synthetic")
                {
                        for    (int i=0; i<cloud_ORGan__N->size(); i++)    {    cloud_ORGan__N->points[i].normal_x *= -1;
                                                                                cloud_ORGan__N->points[i].normal_y *= -1;
                                                                                cloud_ORGan__N->points[i].normal_z *= -1;     }
                }
        }


        pcl::concatenateFields( *cloud_ORGan_P_,
                                *cloud_ORGan__N,
                                *cloud_UNorg_PN ); // TO-BE UNorgANIZED


        //////////////////////////////////////////////////////////////////////////////////////////////////////////
        if (FLAG_kickOut_pointsWith_CoordNAN_NormalNAN)   kickOut_pointsWith_CoordNAN_NormalNAN( cloud_UNorg_PN );
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

}



void Registrator::kickOut_pointsWith_CoordNAN_NormalNAN( pcl::PointCloud<TYPE_Point_PostPr>::Ptr &cloud_PN )
{

        std::vector<int>                                           dummyIndicesNAN1;
        std::vector<int>                                           dummyIndicesNAN2;
        pcl::removeNaNFromPointCloud(        *cloud_PN, *cloud_PN, dummyIndicesNAN1);
        pcl::removeNaNNormalsFromPointCloud( *cloud_PN, *cloud_PN, dummyIndicesNAN2);

}



void Registrator::kickOut_pointsWith_CoordNAN( pcl::PointCloud<TYPE_Point_Sensor>::Ptr &cloud_P_ )
{

        std::vector<int>                                           dummyIndicesNAN1;
        pcl::removeNaNFromPointCloud(        *cloud_P_, *cloud_P_, dummyIndicesNAN1);

}





