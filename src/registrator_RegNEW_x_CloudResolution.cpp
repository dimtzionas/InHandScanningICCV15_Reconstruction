// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#include <registrator.h>



double Registrator::computeCloudResolution(const pcl::PointCloud<TYPE_Point_PostPr>::ConstPtr &cloud)
{
      double res      = 0.0;
      int    n_points = 0;
      int    nres;

      pcl::search::KdTree<TYPE_Point_PostPr> tree;

      tree.setInputCloud (cloud);

      for (int iii=0; iii<cloud->size(); ++iii)
      {
          ///////////////////////////////////////////////////////////////
          ///////////////////////////////////////////////////////////////
          if (  !pcl_isfinite( (*cloud)[iii].x )         ||     /////////
                !pcl_isfinite( (*cloud)[iii].y )         ||     /////////
                !pcl_isfinite( (*cloud)[iii].z )         ||     /////////
                !pcl_isfinite( (*cloud)[iii].normal_x )  ||     /////////
                !pcl_isfinite( (*cloud)[iii].normal_y )  ||     /////////
                !pcl_isfinite( (*cloud)[iii].normal_z )   )     continue;
          ///////////////////////////////////////////////////////////////
          ///////////////////////////////////////////////////////////////

          std::vector<int>   indices(       2 ); // dummy
          std::vector<float> sqr_distances( 2 );

          //Considering the second neighbor since the first is the point itself !!!
          nres = tree.nearestKSearch(iii, 2, indices, sqr_distances);
          if (nres == 2)
          {
            res += sqrt (sqr_distances[1]);
            ++n_points;
          }
      }

      //////////////////////////////////////
      if (n_points != 0)    res /= n_points;
      //////////////////////////////////////
      if (n_points == 0)    std::cout << std::endl << "computeCloudResolution - (n_points == 0) " << std::endl << std::endl;

      return res;
}





double Registrator::computeCloudResolution(const pcl::PointCloud<TYPE_Point_Sensor>::ConstPtr &cloud)
{
      double res      = 0.0;
      int    n_points = 0;
      int    nres;

      pcl::search::KdTree<TYPE_Point_Sensor> tree;

      tree.setInputCloud (cloud);

      for (int iii=0; iii<cloud->size(); ++iii)
      {
          ///////////////////////////////////////////////////////
          ///////////////////////////////////////////////////////
          if (  !pcl_isfinite( (*cloud)[iii].x )  ||    /////////
                !pcl_isfinite( (*cloud)[iii].y )  ||    /////////
                !pcl_isfinite( (*cloud)[iii].z )  )     continue;
          ///////////////////////////////////////////////////////
          ///////////////////////////////////////////////////////

          //std::cout << "computeCloudResolution - TYPE_Point_Sensor" << "\n"
          //          << (*cloud)[iii].x                              << "\t\t"
          //          << (*cloud)[iii].y                              << "\t\t"
          //          << (*cloud)[iii].z                              << "\t\t"
          //          << std::endl;

          std::vector<int>   indices(       2 ); // dummy
          std::vector<float> sqr_distances( 2 );

          //Considering the second neighbor since the first is the point itself !!!
          nres = tree.nearestKSearch(iii, 2, indices, sqr_distances);
          if (nres == 2)
          {
            res += sqrt (sqr_distances[1]);
            ++n_points;
          }
      }

      //////////////////////////////////////
      if (n_points != 0)    res /= n_points;
      //////////////////////////////////////

      return res;
}


