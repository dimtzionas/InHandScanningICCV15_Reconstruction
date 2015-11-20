// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#include <touch.h>



double Touch::getDistance_SKIN_to_PCL_from_sensor(  pcl::search::KdTree<pcl::PointXYZRGBNormal> &tree, pcl::PointXYZRGBNormal &cloudPt_SKIN )
{
    std::vector<int>   indices(      1 ); // dummy
    std::vector<float> sqr_distance( 1 ); // Joint to PCL
    int                treeNodeDist;                                                  //Joint to PCL
                       treeNodeDist  = tree.nearestKSearch( cloudPt_SKIN, 1, indices, sqr_distance   );

    return sqrt( sqr_distance[0] );
}



double Touch::getDistance_JOINT_to_PCL_from_sensor( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudPtr_PCLsensor, pcl::PointXYZ &bone_Joint_PclPointXYZ )
{
    pcl::search::KdTree<pcl::PointXYZ> tree;
                                       tree.setInputCloud( cloudPtr_PCLsensor );

    std::vector<int>   indices(      1 ); // dummy
    std::vector<float> sqr_distance( 1 ); // Joint to PCL
    int                treeNodeDist;                                                            //Joint to PCL
                       treeNodeDist  = tree.nearestKSearch( bone_Joint_PclPointXYZ, 1, indices, sqr_distance   );

    return sqrt( sqr_distance[0] );
}



double Touch::getDistance_JOINT_to_SpherePrimitive( SphereCoff &sphereCoff, pcl::PointXYZ &bone_Joint_PclPointXYZ )
{
    double signedDIST   =   sqrt( pcl::squaredEuclideanDistance( sphereCoff.center_PointXYZ, bone_Joint_PclPointXYZ ) )   -   sphereCoff.radius;
    return signedDIST;
}



double Touch::getDistance_SKIN_to_SpherePrimitive(  SphereCoff &sphereCoff, pcl::PointXYZRGBNormal &cloudPt_SKIN )
{
    double signedDIST   =   sqrt( pcl::squaredEuclideanDistance( sphereCoff.center_PointXYZ, cloudPt_SKIN ) )   -   sphereCoff.radius;
    return signedDIST;
}
