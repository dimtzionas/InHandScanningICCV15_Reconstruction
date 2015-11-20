// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#include <registrator.h>




void Registrator::my_Filtering_Voxelgrid( pcl::PointCloud<TYPE_Point_PostPr>::Ptr &accumAlignedClouds_UNorg_P_ )
{

        pcl::PointCloud<TYPE_Point_PostPr>::Ptr tmpCloudVoxeled( new pcl::PointCloud<TYPE_Point_PostPr> );

        pcl::VoxelGrid< TYPE_Point_PostPr> voxelFilter;
        voxelFilter.setInputCloud(     accumAlignedClouds_UNorg_P_  );
        voxelFilter.setLeafSize(       PARAM_VOXELFILTER_leafSize, PARAM_VOXELFILTER_leafSize, PARAM_VOXELFILTER_leafSize );
        voxelFilter.filter(           *tmpCloudVoxeled              );

        ////////////////////////////////////////////////
        *accumAlignedClouds_UNorg_P_ = *tmpCloudVoxeled;
        ////////////////////////////////////////////////
}

